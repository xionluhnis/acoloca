// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

#include <Arduino.h>
#include "filters.h"
#include "sine.h"

/**
 * Pulse function: decreasing ramp
 */
typedef struct Chirp {
  // frequency information
  const uint16_t factor = 2;
  const uint16_t shift  = 1;
  // clock information
  const unsigned long f_clock;
  const double  ref_clk;
  const double  ref_period;
  // pwm cycle information
  const uint16_t      cycle_length;
  volatile uint16_t   cycle_duty;
  const unsigned long cycle_duration;
  // duration information
  const uint32_t  duration_on;  
  const uint32_t  duration_off; 
  const uint32_t  duration;
  // internal
  volatile long   start_us;

  Chirp(uint32_t dur, double fc, double mea_clk = 0)
  : f_clock(fc),
    ref_clk(mea_clk ? mea_clk : fc / SEQ_LENGTH / 2), ref_period(1e6/ref_clk),
    cycle_length(SEQ_LENGTH), cycle_duty(0),                // pwm
    cycle_duration(round(double(1e6) / (dur * ref_clk))),
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
chirp_t chirp(1e6, 16000000, 30920);

void sound_setup(){
  Serial.println("Initializing PWM");
  NRF_PWM0->PSEL.OUT[0] = (A1 << PWM_PSEL_OUT_PIN_Pos) | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
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
  NRF_PWM0->EVENTS_SEQSTARTED[0] = 0;
  NRF_PWM0->EVENTS_PWMPERIODEND  = 0;
  NRF_PWM0->INTENSET = (PWM_INTENSET_PWMPERIODEND_Enabled << PWM_INTENSET_PWMPERIODEND_Pos)
                     | (PWM_INTENSET_SEQSTARTED0_Enabled << PWM_INTENSET_SEQSTARTED0_Pos);

  Serial.println("Initializing IRQ");
  NVIC_SetPriority(PWM0_IRQn, 0); //low priority
  NVIC_ClearPendingIRQ(PWM0_IRQn);
  NVIC_EnableIRQ(PWM0_IRQn);
}

void (*chirp_start_callback)();
void (*chirp_end_callback)();

void sound_start(void (*start_callback)(), void (*end_callback)()) {

  // store callback
  chirp_start_callback = start_callback;
  chirp_end_callback = end_callback;
  
  Serial.println("Starting chirp");
  chirp.cycle_duty = 255; // sin256[0];
  chirp.start_us = 0; // micros();
  
  NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

void sound_end() {

  NRF_PWM0->TASKS_STOP = 1;

  Serial.println("Chirp ended");

  chirp_end_callback();
}

extern "C" {

// takes ~9us to ~15us
void PWM0_IRQHandler(void){

  // time information
  long now = micros();
  
  // check the event is a period end
  //NRF_GPIO->OUTSET = 1 << A1;
  //NRF_GPIO->OUT ^= 1 << A3;
  if(NRF_PWM0->EVENTS_PWMPERIODEND != 0){
    NRF_PWM0->EVENTS_PWMPERIODEND = 0; // clear interrupt

    chirp.cycle_duty = max(0.0f, float(chirp.cycle_duty) / chirp.factor - chirp.shift);
    if(chirp.cycle_duty > 0){
      NRF_PWM0->TASKS_SEQSTART[0] = 1;
    } else {
      sound_end();
    }
    
  } else if(NRF_PWM0->EVENTS_SEQSTARTED[0] != 0) {
    NRF_PWM0->EVENTS_SEQSTARTED[0] = 0;

    if(chirp.start_us == 0){
      // only first time
      chirp_start_callback();

      // set chirp start
      chirp.start_us = now;
    }
  }
  //NRF_GPIO->OUTCLR = 1 << A1;
}

}


// pulse decoding
typedef unsigned long timestamp_t;

constexpr const float SPEED_OF_SOUND = 343.0f;
constexpr const float RANGE_DISTANCE_MAX = 10;
constexpr const float RANGE_DISTANCE_MIN = 0.020;
constexpr const unsigned long RANGE_TIME_MIN = (RANGE_DISTANCE_MIN / SPEED_OF_SOUND) * 1e6;
constexpr const unsigned long RANGE_TIME_MAX = (RANGE_DISTANCE_MAX / SPEED_OF_SOUND) * 1e6;


Difference<int16_t> pulse_delta;

bool        pulse_started = false;
timestamp_t pulse_sync_start = 0;
timestamp_t pulse_time  = 0;
int16_t     pulse_value = 0;
uint16_t    pulse_count1 = 0, pulse_count2 = 0;

FilterBuffer<timestamp_t, 10> timestamps;
volatile bool timestamp_new = false;

void pulse_init(timestamp_t now){
  // Serial.println("pulse init");
  pulse_sync_start = now;
  pulse_time  = 0;
  pulse_value = 0;
  pulse_started = true;
  pulse_count1 = pulse_count2 = 0;
}


int16_t pulse_samples[2000];
timestamp_t pulse_times[2000];


bool pulse_update(int16_t sample, timestamp_t now){

  int16_t delta = pulse_delta(sample);

  if(!pulse_started || now <= pulse_sync_start)
    return false; // skip

  ++pulse_count1;

  // compute distance up to now
  timestamp_t time_delta = now - pulse_sync_start;

  if(time_delta <= RANGE_TIME_MIN)
    return false; // skip

  pulse_samples[pulse_count2 % 2000] = sample;
  pulse_times[pulse_count2 % 2000] = now;

  ++pulse_count2;

  // Serial.print("sample="); Serial.println(sample, DEC);
  // Serial.print("delta="); Serial.println(delta, DEC);
    
  if(abs(delta) > abs(pulse_value)){
    pulse_time = now;
    pulse_value = delta;
  }

  if(time_delta >= RANGE_TIME_MAX){
    if(pulse_time > 0){
      timestamps.push(pulse_time);
      timestamp_new = true;
    }

    /*
    Serial.print("pulse_value="); Serial.println(pulse_value, DEC);
    Serial.print("now="); Serial.println(now, DEC);
    Serial.print("pulse_time="); Serial.println(pulse_time, DEC);
    */
    
    pulse_started = false;
    // else we failed at detecting anything
    return true;
  } else {
    return false;
  }
}
