// Alexandre Kaspar <akaspar@mit.edu>

#define PWM0_ENABLED 1

#include <Arduino.h>

int dbgPin = A5;

/**
 * Sine table to avoid computing sine in IRQ
 */
#define SEQ_LENGTH 256   
static uint16_t sine256[SEQ_LENGTH];

/**
 * Chirp function
 *  s(t) = sin(2pi f(t) f)
 *  f(t) = f0 + (f1-f0) t
 *  
 *  => s(t) = sin(2pi (f0 + (f1-f0)t) t )
 */

typedef struct Chirp {
  // frequency information
  const uint16_t  f_start;
  const uint16_t  f_end;
  const int16_t   f_delta;
  const uint32_t  f_clock;
  // dds information
  const double  ref_clk;
  const double  n_samples;
  const double  ref_period;
  // pwm cycle information
  const uint16_t      cycle_length;
  volatile uint16_t   cycle_duty;
  const uint32_t      cycles_per_chirp;
  const uint32_t      cycles_per_half_chirp;
  const unsigned long cycle_duration;
  // dds phase information
  volatile unsigned long phaccu;
  volatile unsigned long tuning_word;
  const unsigned long    tuning_word_first;
  const unsigned long    tuning_word_delta;
  // duration information
  const uint32_t  duration_on;  
  const uint32_t  duration_off; 
  const uint32_t  duration;
  // internal
  volatile long   start_us;

  Chirp(uint16_t f0, uint16_t f1, uint32_t dur, uint32_t fc, double mea_clk = 0)
  : f_start(f0), f_end(f1), f_delta(f1 - f0), f_clock(fc),  // freq
    ref_clk(mea_clk ? mea_clk : fc / SEQ_LENGTH / 2),       // dds reference clock, defaults to 31250Hz for 16MHz clock
    n_samples(pow(2, 32)), ref_period(1e6/ref_clk),         // dds samples
    cycle_length(SEQ_LENGTH), cycle_duty(0),                // pwm
    cycles_per_chirp(ceil(ref_clk * float(dur / 1e6))),
    cycles_per_half_chirp(cycles_per_chirp/2),
    cycle_duration(round(double(1e6) / (dur * ref_clk))),
    tuning_word(n_samples * f0 / ref_clk),
    tuning_word_first(tuning_word),
    tuning_word_delta(round(f_delta / (ref_clk * dur / 1e6) * n_samples / ref_clk * 2)),
    duration_on(dur), duration_off(dur),                    // duration
    duration(duration_on + duration_off)
  {
  }

  inline uint8_t prescaler() const {
    switch(f_clock){
      case 16000000: return PWM_PRESCALER_PRESCALER_DIV_1;
      case 8000000: return PWM_PRESCALER_PRESCALER_DIV_2;
      case 4000000: return PWM_PRESCALER_PRESCALER_DIV_4;
      case 2000000: return PWM_PRESCALER_PRESCALER_DIV_8;
      case 1000000: return PWM_PRESCALER_PRESCALER_DIV_16;
      case 500000:  return PWM_PRESCALER_PRESCALER_DIV_32;
      case 125000:  return PWM_PRESCALER_PRESCALER_DIV_128;
      case 250000:
      default:      return PWM_PRESCALER_PRESCALER_DIV_64;
    }
  }
  
} chirp_t;

// chirp instance
chirp_t chirp(1100, 1000, 1e6, 16000000, 30920);

extern "C" {

// takes ~9us to ~15us
void PWM0_IRQHandler(void){
  // check the event is a period end
  NRF_GPIO->OUTSET = 1 << A1;
  NRF_GPIO->OUT ^= 1 << A3;
  if(NRF_PWM0->EVENTS_PWMPERIODEND != 0){
    NRF_PWM0->EVENTS_PWMPERIODEND = 0; // clear interrupt

    static uint16_t sampleIdx = 0;

    // time information
    long current = micros();
    // long current = xTaskGetTickCountFromISR(); // better version for IRQ handler?
    long dt = current - chirp.start_us;

    // chirp on/off
    if(dt < chirp.duration_on){

      // 32bits phase accumulator
      if(sampleIdx++ >= chirp.cycles_per_half_chirp){
        chirp.tuning_word -= chirp.tuning_word_delta;
      } else {
        chirp.tuning_word += chirp.tuning_word_delta;
      }
      chirp.phaccu += chirp.tuning_word;
  
      // frequency debug
      static uint8_t last_idx = 0;
  
      // use most significant 8 bits as frequency information
      uint8_t sine_idx = chirp.phaccu >> 24;
      if(sine_idx < last_idx){
        NRF_GPIO->OUT ^= 1 << A2; // toggle to get period / frequency
      }
      last_idx = sine_idx;
      
  
      // update duty cycle from sine table
      chirp.cycle_duty = sine256[sine_idx];
      
    } else {
      
      // out of chirp
      if(current + chirp.ref_period / 2 >= chirp.start_us + chirp.duration){
        // we should restart chirp on next iteration
        chirp.phaccu = 0;
        chirp.start_us = current;
        chirp.tuning_word = chirp.tuning_word_first;
        sampleIdx = 0;
        // debug signal
        NRF_GPIO->OUT ^= 1 << A4;
      } else {
        // off-chirp
        chirp.cycle_duty = 0 | (1 << 15);
      }
    }

    // update DMA
    NRF_PWM0->TASKS_SEQSTART[0] = 1;
  }
  NRF_GPIO->OUTCLR = 1 << A1;
}

}

void init_sinetable(float amplitude = 1.0){
  // compute sine table
  uint16_t sine_mid = amplitude * (SEQ_LENGTH / 2 - 1);
  for(int i = 0; i < SEQ_LENGTH; ++i){
    sine256[i] = round(sine_mid + sine_mid * sin(2 * M_PI * i / SEQ_LENGTH));
  }
}

/**************************************************************************/
/*!
    @brief  The setup function runs once when reset the board
*/
/**************************************************************************/
void setup()
{
  Serial.begin(9600);

  pinMode(dbgPin, OUTPUT);
  digitalWrite(dbgPin, LOW);
  
  Serial.println("Generating sine table");
  init_sinetable(1);
  
  Serial.println("Chirp info:");
  #define CHIRP_INFO(name) {Serial.print("- " #name ": "); Serial.println(chirp.name);}
  {
    CHIRP_INFO(f_start);
    CHIRP_INFO(f_end);
    CHIRP_INFO(f_delta);
    CHIRP_INFO(f_clock);
    CHIRP_INFO(ref_clk);
    CHIRP_INFO(n_samples);
    CHIRP_INFO(ref_period);
    CHIRP_INFO(cycle_length);
    CHIRP_INFO(cycles_per_chirp);
    CHIRP_INFO(cycle_duration);
    CHIRP_INFO(duration_on);
    CHIRP_INFO(duration_off);
    CHIRP_INFO(duration);
    CHIRP_INFO(tuning_word);
    CHIRP_INFO(tuning_word_delta);
  }
  

  Serial.println("Initializing GPIO");
  NRF_GPIO->DIRSET = 1 << A1; // IRQ timing (to check sanity)
  NRF_GPIO->DIRSET = 1 << A2; // phase 8-bit overflow (to measure frequency)
  NRF_GPIO->DIRSET = 1 << A3; // real pwm frequency (to tune reference frequency)
  NRF_GPIO->DIRSET = 1 << A4; // chirp duration

  Serial.println("Initializing PWM");
  NRF_PWM0->PSEL.OUT[0] = (A0 << PWM_PSEL_OUT_PIN_Pos) | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
  NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
  NRF_PWM0->MODE = (PWM_MODE_UPDOWN_UpAndDown << PWM_MODE_UPDOWN_Pos);
  Serial.print("Prescaler: ");
  Serial.println(chirp.prescaler());
  NRF_PWM0->PRESCALER = (chirp.prescaler() << PWM_PRESCALER_PRESCALER_Pos);
  NRF_PWM0->COUNTERTOP = (chirp.cycle_length << PWM_COUNTERTOP_COUNTERTOP_Pos); //1 msec
  NRF_PWM0->LOOP = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
  NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Common << PWM_DECODER_LOAD_Pos) | (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
  NRF_PWM0->SEQ[0].PTR = ((uint32_t)(&chirp.cycle_duty) << PWM_SEQ_PTR_PTR_Pos);
  NRF_PWM0->SEQ[0].CNT = (1 <<  PWM_SEQ_CNT_CNT_Pos);
  NRF_PWM0->SEQ[0].REFRESH = 0;
  NRF_PWM0->SEQ[0].ENDDELAY = 0;
  // set second sequence to null
  NRF_PWM0->SEQ[1].PTR = 0;
  NRF_PWM0->SEQ[1].CNT = 0;
  NRF_PWM0->SEQ[1].REFRESH = 0;
  NRF_PWM0->SEQ[1].ENDDELAY = 0;

  Serial.println("Initializing events");
  NRF_PWM0->EVENTS_PWMPERIODEND = 0;
  NRF_PWM0->INTENSET = (PWM_INTENSET_PWMPERIODEND_Enabled << PWM_INTENSET_PWMPERIODEND_Pos);

  Serial.println("Initializing IRQ");
  NVIC_SetPriority(PWM0_IRQn, 0); //low priority
  NVIC_ClearPendingIRQ(PWM0_IRQn);
  NVIC_EnableIRQ(PWM0_IRQn);
  
  Serial.println("Starting chirp");
  chirp.cycle_duty = sine256[0];
  chirp.start_us = micros();
  
  // send task
  NRF_PWM0->TASKS_SEQSTART[0] = 1;

}

/**************************************************************************/
/*!
    @brief  The loop function runs over and over again forever
*/
/**************************************************************************/
void loop()
{
  // CHIRP_INFO(tuning_word);
}





