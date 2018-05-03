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
 * Frequency-Shift Keying chirp
 */

typedef struct Chirp {
  // frequency information
  const uint16_t  f_0;
  const uint16_t  f_1;
  const uint32_t  f_clock;
  // dds information
  const double  ref_clk;
  const double  n_samples;
  const double  ref_period;
  const unsigned long tuning_word_0;
  const unsigned long tuning_word_1;
  volatile unsigned long phaccu;
  // pwm cycle information
  const uint16_t     cycle_period;
  volatile uint16_t  cycle_duty;
  const uint32_t     cycles_per_chirp;
  // duration information
  const uint32_t  duration_on;  
  const uint32_t  duration_off; 
  const uint32_t  duration;
  // fsk pulses
  const uint32_t   fsk_code;
  const uint8_t    fsk_code_length;
  const uint16_t   fsk_scale;
  volatile uint8_t fsk_code_bit;
  volatile uint16_t fsk_code_counter;
  // internal
  volatile long   start_us;

  Chirp(uint16_t f0, uint16_t f1, uint32_t code, uint32_t dur, uint32_t fc, double mea_clk = 0, uint8_t codelen = 32, uint16_t scale = 0)
  : f_0(f0), f_1(f1), f_clock(fc),  // freq
    ref_clk(mea_clk ? mea_clk : fc / SEQ_LENGTH / 2),       // dds reference clock, defaults to 31250Hz for 16MHz clock
    n_samples(pow(2, 32)), ref_period(1e6/ref_clk),         // dds samples
    tuning_word_0(n_samples * f0 / ref_clk),
    tuning_word_1(n_samples * f1 / ref_clk),
    cycle_period(SEQ_LENGTH), cycle_duty(0),                // pwm
    cycles_per_chirp(ceil(ref_clk * float(dur / 1e6))),
    duration_on(dur), duration_off(dur),                    // duration
    duration(duration_on + duration_off),
    fsk_code(code), fsk_code_length(codelen),
    fsk_scale(scale ? scale : ceil(cycles_per_chirp / float(codelen))),
    fsk_code_bit(0), fsk_code_counter(0)
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

// codes:
// - 0b01011001011110001110011010001011
// - 0b11111100000011110000111000110010 = [1,2,3,4,6].map(n => { let str = ''; for(let i = 0; i < n; ++i) str += '1'; return str + str.replace(/1/g, '0'); }).reverse().join('')

// chirp instance
chirp_t chirp(1000, 1500, 0b11111100000011110000111000110010, 1e6, 16000000, 30920, 32);

extern "C" {

// takes ~9us to ~15us
void PWM0_IRQHandler(void){
  // check the event is a period end
  NRF_GPIO->OUTSET = 1 << A1;
  NRF_GPIO->OUT ^= 1 << A3;
  if(NRF_PWM0->EVENTS_PWMPERIODEND != 0){
    NRF_PWM0->EVENTS_PWMPERIODEND = 0; // clear interrupt

    // time information
    long current = micros();
    // long current = xTaskGetTickCountFromISR(); // better version for IRQ handler?
    long dt = current - chirp.start_us;

    // chirp on/off
    if(dt < chirp.duration_on){

      // 32bits phase accumulator
      // tuning word depends on code bit
      uint8_t fsk_bit = uint8_t(chirp.fsk_code >> chirp.fsk_code_bit) & 0x01;
      unsigned long tuning_word;
      if(fsk_bit){
        NRF_GPIO->OUTSET = 1 << A5;
        tuning_word = chirp.tuning_word_1;
      }else{
        NRF_GPIO->OUTCLR = 1 << A5;
        tuning_word = chirp.tuning_word_0;
      }
      chirp.phaccu += tuning_word;
  
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
      
      // fsk counters
      chirp.fsk_code_counter += 1;
      if(chirp.fsk_code_counter >= chirp.fsk_scale){
        // move to next fsk code bit
        chirp.fsk_code_bit += 1;
        chirp.fsk_code_counter = 0;
      }
      
    } else {
      
      // out of chirp
      if(current + chirp.ref_period / 2 >= chirp.start_us + chirp.duration){
        // we should restart chirp on next iteration
        chirp.phaccu = 0;
        chirp.start_us = current;
        
        // restart counters
        chirp.fsk_code_bit = 0;
        chirp.fsk_code_counter = 0;
        
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
    CHIRP_INFO(f_0);
    CHIRP_INFO(f_1);
    CHIRP_INFO(f_clock);
    CHIRP_INFO(ref_clk);
    CHIRP_INFO(n_samples);
    CHIRP_INFO(ref_period);
    CHIRP_INFO(cycle_period);
    CHIRP_INFO(cycles_per_chirp);
    CHIRP_INFO(duration_on);
    CHIRP_INFO(duration_off);
    CHIRP_INFO(duration);
    CHIRP_INFO(tuning_word_0);
    CHIRP_INFO(tuning_word_1);
    CHIRP_INFO(fsk_code);
    CHIRP_INFO(fsk_code_length);
    CHIRP_INFO(fsk_scale);
  }
  

  Serial.println("Initializing GPIO");
  NRF_GPIO->DIRSET = 1 << A1; // IRQ timing (to check sanity)
  NRF_GPIO->DIRSET = 1 << A2; // phase 8-bit overflow (to measure frequency)
  NRF_GPIO->DIRSET = 1 << A3; // real pwm frequency (to tune reference frequency)
  NRF_GPIO->DIRSET = 1 << A4; // chirp duration
  NRF_GPIO->DIRSET = 1 << A5; // fsk code

  Serial.println("Initializing PWM");
  NRF_PWM0->PSEL.OUT[0] = (A0 << PWM_PSEL_OUT_PIN_Pos) | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
  NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
  NRF_PWM0->MODE = (PWM_MODE_UPDOWN_UpAndDown << PWM_MODE_UPDOWN_Pos);
  Serial.print("Prescaler: ");
  Serial.println(chirp.prescaler());
  NRF_PWM0->PRESCALER = (chirp.prescaler() << PWM_PRESCALER_PRESCALER_Pos);
  NRF_PWM0->COUNTERTOP = (chirp.cycle_period << PWM_COUNTERTOP_COUNTERTOP_Pos); //1 msec
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





