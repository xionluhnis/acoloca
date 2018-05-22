// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

#include <Arduino.h>
#include "filters.h"
#include "sine.h"

// types
typedef unsigned long timestamp_t;

// constants
constexpr const float PLL_GAIN = 1;
constexpr const float REF_FREQ = 1050;
constexpr const float REF_OMEGA = 2.0 * M_PI * REF_FREQ;
constexpr const float SAMPLE_RATE = 58.5e3; // 8704
constexpr const float SAMPLE_PERIOD = 1.0 / SAMPLE_RATE;
constexpr const double N_SAMPLES = 1ULL << 32; // /!\ needs 1ULL, 1UL overflows
constexpr const double TUNING_MICRO_FACTOR = N_SAMPLES / 1e6 * REF_FREQ;
constexpr const unsigned long TUNING_DELTA = N_SAMPLES * REF_FREQ / SAMPLE_RATE;
constexpr const float LOCK_THRESHOLD = 0.2;
constexpr const uint16_t NORM_SAMPLES = ceil(3 * SAMPLE_RATE / REF_FREQ);

// filters
NormalizationFilter<NORM_SAMPLES> norm_filter;
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
float   ref_phase = 0;
uint8_t ref_phase_idx = 0;
float   ref_signal = 0;
float   ref_quad = 0;
unsigned long ref_phase_accu = 0;

// demodulation data
uint8_t zx_count = 0;
timestamp_t zx_times[2];
FilterBuffer<timestamp_t, 10> timestamps;
bool timestamp_new = false;

/**
 * Initialize PLL
 */
void pll_setup() {
  Serial.println("Output lowpass:");
  output_lowpass.debug();
  Serial.println("Lock lowpass:");
  lock1_lowpass.debug();
}

unsigned long last_pll_time = 0;

/**
 * PLL loop iteration
 * 
 * @param input normalized input signal
 */
void pll_run(float input, timestamp_t now){

  // PLL (~3.8us)
  // NRF_GPIO->OUTSET = 1 << A2;
  // pase detector and filters
  pll_loop_control = input * ref_signal * PLL_GAIN; // ~250ns
  pll_output = output_lowpass(pll_loop_control); // ~500ns

  // reference signal using phase accumulator (~400ns)
  // compute precise tuning delta (for varying clocks)
  /*timestamp_t dt = now - last_pll_time;
  last_pll_time = now;
  timestamp_t tuning_delta;
  if(dt > 1000){
    // very large time change
    // => assume single period
    tuning_delta = TUNING_DELTA;
  } else {
    tuning_delta = TUNING_MICRO_FACTOR * dt;
  }
  */
  // compute phase
  timestamp_t tuning_word = TUNING_DELTA * (1.0f + pll_loop_control);
  ref_phase_accu += tuning_word;
  uint8_t ref_phase_idx = ref_phase_accu >> 24;
  ref_signal = fsin256[ref_phase_idx];

  // quadrature signal
  ref_quad = fcos256[ref_phase_idx];

  // lock computations
  pll_lock1 = lock1_lowpass(-ref_quad   * input);
  pll_lock2 = lock2_lowpass(-ref_signal * input);
  pll_logic_lock = max(pll_lock1, pll_lock2) > LOCK_THRESHOLD;

  if(pll_logic_lock)
    NRF_GPIO->OUTSET = 1 << A3;
  else
    NRF_GPIO->OUTCLR = 1 << A3;
  // NRF_GPIO->OUTCLR = 1 << A2;
  
}

/**
 * PLL demodulation
 * 
 * @param now current sample's acquisition timestamp
 * @return whether demodulation found a chirp timing
 */
bool pll_demod(const timestamp_t &now){

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
      timestamp_new = true;
    }
    zx_count = 0;

    return zx_count >= 2;
    
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

  return false;
}

bool pll_update(uint8_t sample, timestamp_t now){
  // filter data (~2us, peaks at 6.5us)
  float out = norm_filter(sample);
  
  // run PLL loop (4.6us)
  pll_run(out, now);
  
  // demodulation (1.2us)
  return pll_demod(now);
}

