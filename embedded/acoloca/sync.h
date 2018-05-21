// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

#include <Arduino.h>

void sync_noop(){
}

bool sync_started = false;
int  sync_pin = A2;

void (*sync_onstart_callback)();
void (*sync_onend_callback)();

void sync_setup(void (*callback)()) {

  // store synchronization start callback
  sync_onstart_callback = callback;
  sync_onend_callback = &sync_noop;

  // select pin
  NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos)
                        | (sync_pin << GPIOTE_CONFIG_PSEL_Pos)
                        | (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos);
  
}

void sync_start(){
  // disable events
  NVIC_ClearPendingIRQ(GPIOTE_IRQn);
  NRF_GPIOTE->INTENCLR = (GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos);
  NRF_GPIOTE->CONFIG[0] &= GPIOTE_CONFIG_POLARITY_None << GPIOTE_CONFIG_POLARITY_Pos;
  sync_onend_callback = &sync_noop;

  // sync_end();
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
  
  // Serial.println("Starting SYNC");
  
  NVIC_ClearPendingIRQ(GPIOTE_IRQn);
  NVIC_EnableIRQ(GPIOTE_IRQn);

  // setup interrupts
  NRF_GPIOTE->EVENTS_IN[0] = 0;
  NRF_GPIOTE->CONFIG[0] |= GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos;
  NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos);

}

extern "C" {

void GPIOTE_IRQHandler(void){
  
  if(NRF_GPIOTE->EVENTS_IN[0] != 0){
    NRF_GPIOTE->EVENTS_IN[0] = 0;

    if(!sync_started){
      // NRF_GPIO->IN & (A3 << 1))
      // triggered up
      sync_started = true;
      sync_onstart_callback();

    } else {
      // triggered down
      sync_onend_callback();
      sync_started = false;
    }

  }

}

}

