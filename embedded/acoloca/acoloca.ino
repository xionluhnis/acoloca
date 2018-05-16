// Alexandre Kaspar <akaspar@mit.edu>

#define USE_FIXED_POINT 0

#include <Arduino.h>
#include "chirp.h"
#include "filters.h"

// constants
const float PLL_GAIN = 0.1;
const float REF_FREQ = 1050;
const float REF_OMEGA = 2.0 * M_PI * REF_FREQ;
const float SAMPLE_RATE = 8704;
const float SAMPLE_PERIOD = 1.0 / SAMPLE_RATE;
const double N_SAMPLES = 1ULL << 32;
const unsigned long TUNING_DELTA = N_SAMPLES * REF_FREQ / SAMPLE_RATE;
const float LOCK_THRESHOLD = 0.2;

// filters
NormalizationFilter<25> norm_filter;
BiquadFilter<float> output_lowpass = BiquadFilter<float>::lowpass(10, SAMPLE_RATE);
BiquadFilter<float> lock1_lowpass  = BiquadFilter<float>::lowpass(10, SAMPLE_RATE);
BiquadFilter<float> lock2_lowpass  = BiquadFilter<float>::lowpass(10, SAMPLE_RATE);
Difference<int8_t> output_sign_diff;
Difference<int8_t> logic_lock_diff;

// pll signals
float pll_integral = 0;
float pll_loop_control = 0;
float pll_output = 0;
float pll_lock1 = 0;
float pll_lock2 = 0;
bool  pll_logic_lock = false;

// reference signals
float ref_phase  = 0;
float ref_signal = 0;
float ref_quad   = 0;
unsigned long ref_phase_accu = 0;

// demodulation data
typedef unsigned long timestamp_t;
uint8_t zx_count = 0;
timestamp_t zx_times[2];
FilterBuffer<timestamp_t, 10> timestamps;

/**************************************************************************/
/*!
    @brief  The setup function runs once when reset the board
*/
/**************************************************************************/
void setup()
{
  Serial.begin(230400);

  Serial.println("################");
  Serial.println("Starting ACOLOCA");
  Serial.println("################");
  
  Serial.println("Generating sin/cos tables");
  init_sinetables(1);

  /*
  Serial.println("Setup chirp");
  chirp_setup();
  */
  
  Serial.println("Initializing GPIO");
  NRF_GPIO->DIRSET = 1 << A1; // IRQ timing (to check sanity)
  NRF_GPIO->DIRSET = 1 << A2; // phase 8-bit overflow (to measure frequency)
  NRF_GPIO->DIRSET = 1 << A3; // real pwm frequency (to tune reference frequency)
  NRF_GPIO->DIRSET = 1 << A4; // chirp duration
  NRF_GPIO->OUTCLR = 1 << A1 | 1 << A2 | 1 << A3 | 1 << A4;

  Serial.println("Output lowpass:");
  output_lowpass.debug();
  Serial.println("Lock lowpass:");
  lock1_lowpass.debug();
}

/**************************************************************************/
/*!
    @brief  The loop function runs over and over again forever
*/
/**************************************************************************/
void loop()
{
  // read data
  uint8_t data = Serial.read();
  if(!data)
    return;

  // record time of arrival
  timestamp_t now = micros();

  // send header
  Serial.write(uint8_t(0xFF));
  Serial.write(uint8_t(0x01));
  
  // filter data
  NRF_GPIO->OUTSET = 1 << A1;
#if USE_FIXED_POINT
  // SQ2x13 = 8-13us
  SQ2x13 out = norm_filter(data);
#else
  // float  = 2-8us
  float out = norm_filter(data);
#endif
  NRF_GPIO->OUTCLR = 1 << A1;

  // PLL (~3.8us)
  NRF_GPIO->OUTSET = 1 << A2;
  {
    // pase detector and filters
    pll_loop_control = out * ref_signal * PLL_GAIN; // ~250ns
    pll_output = output_lowpass(pll_loop_control); // ~500ns

    // reference signal using phase accumulator (~400ns)
    unsigned long tuning_word = TUNING_DELTA * (1.0f + pll_loop_control);
    ref_phase_accu += tuning_word;
    uint8_t ref_phase_idx = ref_phase_accu >> 24;
    ref_signal = fsin256[ref_phase_idx];

    // quadrature signal
    ref_quad = fcos256[ref_phase_idx];

    // lock computations
    pll_lock1 = lock1_lowpass(-ref_quad   * out);
    pll_lock2 = lock2_lowpass(-ref_signal * out);
    pll_logic_lock = max(pll_lock1, pll_lock2) > LOCK_THRESHOLD;
  }
  NRF_GPIO->OUTCLR = 1 << A2;

  // state information
  int8_t output_change = output_sign_diff(pll_output >= 0);
  int8_t logic_change  = logic_lock_diff(pll_logic_lock ? 1 : 0);

  // Demodulation
  //- naive: detect the two zero-crossings of output sign difference series
  if(logic_change == 1){
    // lock just started
    zx_count = 0;
    
  } else if(logic_change == -1){
    // lock just ended

    // did we get a valid estimate?
    if(zx_count == 2){
      timestamp_t chirp_center = (zx_times[0] + zx_times[1]) / 2; // XXX avoid loosing 1/2 precision
      timestamps.push(chirp_center);
    }
    zx_count = 0;
    
  } else if(pll_logic_lock){
    // we are locked
    
    // output_change in {-1, 0, +1}
    if(output_change != 0){
      
      // count change events
      ++zx_count;

      // store timing
      uint8_t which = (output_change + 1) / 2;
      zx_times[which] = now;
    }
  }
  
  out = zx_count * 0.1f;

  // send result back
  uint16_t uout = SQ2x13(out).getInternal();
  Serial.write(uout >> 8);    // MSB
  Serial.write(uout & 0xFF);  // LSB
}





