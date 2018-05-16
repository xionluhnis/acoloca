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
NormalizationFilter<25> normFilter;
BiquadFilter<float> output_lowpass = BiquadFilter<float>::lowpass(10, SAMPLE_RATE);
BiquadFilter<float> lock1_lowpass  = BiquadFilter<float>::lowpass(10, SAMPLE_RATE);
BiquadFilter<float> lock2_lowpass  = BiquadFilter<float>::lowpass(10, SAMPLE_RATE);

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
float ref_time   = 0;
float ref_quad   = 0;
unsigned long ref_phase_accu = 0;

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

  // send header
  Serial.write(uint8_t(0xFF));
  Serial.write(uint8_t(0x01));
  
  // filter data
  NRF_GPIO->OUTSET = 1 << A1;
#if USE_FIXED_POINT
  // SQ2x13 = 8-13us
  SQ2x13 out = normFilter(data);
#else
  // float  = 2-8us
  float out = normFilter(data);
#endif
  NRF_GPIO->OUTCLR = 1 << A1;

  // PLL
  NRF_GPIO->OUTSET = 1 << A2;
  {
    // pase detector and filters
    pll_loop_control = out * ref_signal * PLL_GAIN; // ~250ns
    pll_output = output_lowpass(pll_loop_control); // ~500ns
    

    // FM integral
    
    pll_integral += pll_loop_control * SAMPLE_PERIOD; // ~100ns
    
    // reference signal
    /*
    NRF_GPIO->OUTSET = 1 << A3;
    ref_time += SAMPLE_PERIOD; // ~100ns
    ref_phase = REF_OMEGA * (ref_time + pll_integral); // ~100ns
    NRF_GPIO->OUTCLR = 1 << A3;
    
    NRF_GPIO->OUTSET = 1 << A4;
    ref_signal = sin(ref_phase);
    NRF_GPIO->OUTCLR = 1 << A4;
    */

    // reference signal using phase accumulator (~400ns)
    unsigned long tuning_word = TUNING_DELTA * (1.0f + pll_loop_control);
    ref_phase_accu += tuning_word;
    uint8_t ref_phase_idx = ref_phase_accu >> 24;
    ref_signal = fsin256[ref_phase_idx];
    
    /*
    Serial.print("Tuning word: ");
    Serial.println(tuning_word);
    Serial.println("Compare:");
    Serial.println(ref_signal, 6);
    Serial.println(ref_signal2, 6);
    // */

    // quadrature signal
    ref_quad = fcos256[ref_phase_idx];

    // lock computations
    pll_lock1 = lock1_lowpass(-ref_quad   * out);
    pll_lock2 = lock2_lowpass(-ref_signal * out);
    pll_logic_lock = max(pll_lock1, pll_lock2) > LOCK_THRESHOLD;

    out = max(pll_lock1, pll_lock2);
  }
  NRF_GPIO->OUTCLR = 1 << A2;

  // send result back
  uint16_t uout = SQ2x13(out).getInternal();
  Serial.write(uout >> 8);    // MSB
  Serial.write(uout & 0xFF);  // LSB
}





