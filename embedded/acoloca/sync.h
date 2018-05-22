// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

#include <Arduino.h>

void sync_noop(){
}

bool sync_started = false;
int  sync_pin = A2;

uint32_t sync_config_emit   = 0;
uint32_t sync_config_listen = 0;

void (*sync_onstart_callback)();
void (*sync_onend_callback)();

void sync_setup(void (*callback)()) {

  // store synchronization start callback
  sync_onstart_callback = callback;
  sync_onend_callback = &sync_noop;

  // create both emit+listen configurations with pin selection
  sync_config_listen = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos)
                     | (sync_pin << GPIOTE_CONFIG_PSEL_Pos)
                     | (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos);
  sync_config_emit   = (GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos)
                     | (sync_pin << GPIOTE_CONFIG_PSEL_Pos)
                     | (GPIOTE_CONFIG_POLARITY_None << GPIOTE_CONFIG_POLARITY_Pos);

  // use listen configuration by default
  NRF_GPIOTE->CONFIG[0] = sync_config_listen;


  // setup precise timing
  // timer3 accumulate microseconds
  // timer2 pushes microseconds from timer3 to its cc register
  NRF_TIMER2->PRESCALER = 0;   // 16 MHz base frequency
  NRF_TIMER2->MODE = 0;        // Timer
  NRF_TIMER2->BITMODE = 0;     // 16 Bit timer
  NRF_TIMER2->CC[0] = 16;      // roll back every microsecond
  NRF_TIMER2->SHORTS = 1;      // CC0 clears counter
  NRF_TIMER2->TASKS_CLEAR = 1; // clear timer for now

  NRF_TIMER3->PRESCALER = 4;   // 1 MHz base frequency
  NRF_TIMER3->MODE = 0;
  NRF_TIMER3->BITMODE = 3;     // 32 Bit timer
  NRF_TIMER3->TASKS_CLEAR = 1;

  // setup PPI
  NRF_PPI->CH[1].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0];
  NRF_PPI->CH[1].TEP = (uint32_t)&NRF_TIMER3->TASKS_CAPTURE[0];
  NRF_PPI->CHENSET   = PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos;

  // start us timer
  NRF_TIMER2->TASKS_START = 1;
  NRF_TIMER3->TASKS_START = 1;
}

typedef unsigned long timestamp_t;
timestamp_t sync_micros() {
  return NRF_TIMER3->CC[0];
}

void sync_start(){
  // disable events
  NVIC_ClearPendingIRQ(GPIOTE_IRQn);
  NRF_GPIOTE->INTENCLR = (GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos);
  NRF_GPIOTE->CONFIG[0] = sync_config_emit;
  sync_onend_callback = &sync_noop;

  // set gpio
  NRF_GPIO->DIRSET = 1 << sync_pin;
  NRF_GPIO->OUTSET = 1 << sync_pin;
}

void sync_end() {
  NRF_GPIO->OUTCLR = 1 << sync_pin;
}

void sync_listen(void (*callback)()) {

  sync_started = false;
  // store callback
  sync_onend_callback = callback;

  // set gpio
  NRF_GPIO->DIRCLR = 1 << sync_pin;
  NRF_GPIO->PIN_CNF[sync_pin] |= GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos;
  
  // enable events
  NVIC_ClearPendingIRQ(GPIOTE_IRQn);
  NVIC_EnableIRQ(GPIOTE_IRQn);
  // setup interrupts
  NRF_GPIOTE->EVENTS_IN[0] = 0;
  NRF_GPIOTE->CONFIG[0] = sync_config_listen;
  NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos);

}

extern "C" {

void GPIOTE_IRQHandler(void){
  
  if(NRF_GPIOTE->EVENTS_IN[0] != 0){
    NRF_GPIOTE->EVENTS_IN[0] = 0;

    if(!sync_started){
      // triggered up
      sync_started = true;
      sync_onstart_callback();

    } else {
      // triggered down
      sync_started = false;
      sync_onend_callback();
    }

  }

}

}

