// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

#if defined(USE_CHIRP)
#include "pll.h"
#define sample_update pll_update
#elif defined(USE_PULSE)
#include "pulse.h"
#define sample_update pulse_update
#endif

#define SAADC_USE_EVENTS 1
#define SAADC_ASAP 0
#define SAADC_USE_PPI_FORK 1

// constants
constexpr const unsigned long DESIRED_SAMPLE_RATE = 60000; // 100kHz
constexpr const unsigned long ACQUISITION_TIME    = SAADC_CH_CONFIG_TACQ_10us;

// data
uint16_t saadc_buffer = 0;
volatile uint8_t  saadc_sample = 0;
volatile uint32_t saadc_sample_count = 0;

/**
 * Setup SAADC registers
 * 
 * Gain: 
 *    Input range = (0.6 V) / (1/6) = 3.6 V
 *    
 * Acquisition time:
 *    TACQ | max srcR [kOhm]
 *    3    | 10
 *    5    | 40
 *    10   | 100
 *    15   | 200
 *    20   | 400
 *    40   | 800
 *    
 * Tasks:
 *    START  = start ADC and prepare the result buffer in RAM
 *    SAMPLE = take one ADC sample
 *    STOP   = stop ADC and terminate on-going conversions
 *    
 * Frequency divider:
 *    Sampling frequency = 16 MHz / 160 = 100kHz
 *    Divider from 80 to 2047
 * 
 * Events:
 *    STARTED    = ADC is started
 *    END        = ADC has filled up result buffer
 *    DONE       = one conversion has completed (might need multiple)
 *    RESULTDONE = one result is ready to transfer to RAM
 *    STOPPED    = ADC has stopped
 */
void saadc_setup() {
  // setup
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_8bit;
  NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass;
  
  // unselect all pins
  for (uint8_t i = 0; i < 8; i++) {
    NRF_SAADC->CH[i].PSELN = SAADC_CH_PSELP_PSELP_NC;
    NRF_SAADC->CH[i].PSELP = SAADC_CH_PSELP_PSELP_NC;
  }

  // setup DMA pointer
  NRF_SAADC->RESULT.PTR = (uint32_t)&saadc_buffer;
  NRF_SAADC->RESULT.MAXCNT = 1; // One sample

  // select pin
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELP_PSELP_AnalogInput0;
  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput0;

  // set analog pin configuration
  // according to errata 74 this must be the last thing before enable
  NRF_SAADC->CH[0].CONFIG = 
        ((SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos)   & SAADC_CH_CONFIG_RESP_Msk)
      | ((SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESN_Pos)   & SAADC_CH_CONFIG_RESN_Msk)
      | ((SAADC_CH_CONFIG_GAIN_Gain1_6    << SAADC_CH_CONFIG_GAIN_Pos)   & SAADC_CH_CONFIG_GAIN_Msk)
      | ((SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) & SAADC_CH_CONFIG_REFSEL_Msk)
      | ((ACQUISITION_TIME                << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
      | ((SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk)
      | ((SAADC_CH_CONFIG_BURST_Disabled  << SAADC_CH_CONFIG_BURST_Pos)  & SAADC_CH_CONFIG_BURST_Msk);

  // @see http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.nrf52832.Rev1.errata%2Fanomaly_832_74.html
  NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);

#if SAADC_USE_EVENTS

  // setup interrupt handler
  NVIC_SetPriority(SAADC_IRQn, 0); //low priority
  NVIC_ClearPendingIRQ(SAADC_IRQn);
  NVIC_EnableIRQ(SAADC_IRQn);

#if SAADC_ASAP

  // setup PPI
  NRF_PPI->CH[0].EEP = (uint32_t)&NRF_SAADC->EVENTS_END;
  NRF_PPI->CH[0].TEP = (uint32_t)&NRF_SAADC->TASKS_START;
#if SAADC_USE_PPI_FORK
  NRF_PPI->FORK[0].TEP = (uint32_t)&NRF_SAADC->TASKS_SAMPLE;
#endif
  NRF_PPI->CHENSET   = PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos;

#else

  // setup timer
  /*
  NRF_SAADC->SAMPLERATE = (SAADC_SAMPLERATE_MODE_Timers   << SAADC_SAMPLERATE_MODE_Pos)
                        | (16000000 / DESIRED_SAMPLE_RATE << SAADC_SAMPLERATE_CC_Pos);
  */
  // with PPI from TIMER1
  NRF_TIMER1->PRESCALER = 0;   // 16 MHx base frequency
  NRF_TIMER1->MODE = 0;      // Timer
  NRF_TIMER1->BITMODE = 0;     // 16 Bit timer
  NRF_TIMER1->CC[0] = 16000000 / DESIRED_SAMPLE_RATE; 
  NRF_TIMER1->SHORTS = 1;      // CC0 clears counter
  NRF_TIMER1->TASKS_CLEAR = 1;

  // setup PPI
  NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER1->EVENTS_COMPARE[0];
  NRF_PPI->CH[0].TEP = (uint32_t)&NRF_SAADC->TASKS_START;
#if SAADC_USE_PPI_FORK
  NRF_PPI->FORK[0].TEP = (uint32_t)&NRF_SAADC->TASKS_SAMPLE;
#endif
  NRF_PPI->CHENSET   = PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos;
#endif

#endif

  /* Blocking mode:
  NRF_SAADC->TASKS_START = 0x01UL;

  while (!NRF_SAADC->EVENTS_STARTED);
  NRF_SAADC->EVENTS_STARTED = 0x00UL;

  NRF_SAADC->TASKS_SAMPLE = 0x01UL;

  while (!NRF_SAADC->EVENTS_END);
  NRF_SAADC->EVENTS_END = 0x00UL;

  NRF_SAADC->TASKS_STOP = 0x01UL;

  while (!NRF_SAADC->EVENTS_STOPPED);
  NRF_SAADC->EVENTS_STOPPED = 0x00UL;
  
  NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos);
  */
}

void (*saadc_callback)();

unsigned long saadc_count = 0;

void saadc_start(void (*callback)()) {

  saadc_count = 0;

  // store callback
  saadc_callback = callback;
  
  Serial.println("Starting SAADC");
  
  NVIC_ClearPendingIRQ(SAADC_IRQn);
  NVIC_EnableIRQ(SAADC_IRQn);

  // setup interrupts
  NRF_SAADC->EVENTS_END = 0;
  NRF_SAADC->INTENSET = (SAADC_INTENSET_END_Enabled << SAADC_INTENSET_END_Pos);
#if !SAADC_USE_PPI_FORK
  NRF_SAADC->INTENSET = (SAADC_INTENSET_STARTED_Enabled << SAADC_INTENSET_STARTED_Pos);
#endif

#if SAADC_ASAP
  NRF_SAADC->TASKS_START = 1;
  while (!NRF_SAADC->EVENTS_STARTED);
  NRF_SAADC->EVENTS_STARTED = 0x00UL;

  NRF_SAADC->TASKS_SAMPLE = 0x01UL;
#else
  NRF_TIMER1->TASKS_START = 1;
#endif
  NRF_PPI->CHENSET   = PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos;
}

uint8_t saadc_read() {
  NRF_SAADC->TASKS_START = 0x01UL;

  while (!NRF_SAADC->EVENTS_STARTED);
  NRF_SAADC->EVENTS_STARTED = 0x00UL;

  NRF_SAADC->TASKS_SAMPLE = 0x01UL;

  while (!NRF_SAADC->EVENTS_END);
  NRF_SAADC->EVENTS_END = 0x00UL;

  /*NRF_SAADC->TASKS_STOP = 0x01UL;

  while (!NRF_SAADC->EVENTS_STOPPED);
  NRF_SAADC->EVENTS_STOPPED = 0x00UL;
  */
  return saadc_buffer;
}

void saadc_end() {
  NVIC_ClearPendingIRQ(SAADC_IRQn);
  
  NRF_SAADC->TASKS_STOP = 0x01UL;
  NRF_PPI->CHENCLR   = PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos;

  saadc_callback();
}

extern "C" {

void SAADC_IRQHandler(void){
  
  if(NRF_SAADC->EVENTS_END != 0){
    // NRF_GPIO->OUTSET = 1 << A3;
    // NRF_GPIO->OUT ^= 1 << A3;

    unsigned long now = micros();

    // get sample
    uint8_t sample = saadc_buffer;

    NRF_GPIO->OUTSET = 1 << A3;

    ++saadc_count;

    // update depends on method
    if(sample_update(sample, now)){
      saadc_end();
      /*
      Serial.print("End: ");
      Serial.println(saadc_count, DEC);
      Serial.print("pulse end: c1=");
      Serial.print(pulse_count1, DEC);
      Serial.print(", c2=");
      Serial.println(pulse_count2, DEC);
      */
      saadc_count = 0;
    }
    // clear interrupt only if there's more
    NRF_SAADC->EVENTS_END = 0;

    NRF_GPIO->OUTCLR = 1 << A3;

    // ask for more samples
    // NRF_SAADC->TASKS_START = 0x01UL;
    // NRF_SAADC->TASKS_SAMPLE = 0x01UL;

    // NRF_GPIO->OUTCLR = 1 << A3;
  }
#if !SAADC_USE_PPI_FORK
  else if(NRF_SAADC->EVENTS_STARTED != 0){
    NRF_SAADC->EVENTS_STARTED = 0;

    NRF_SAADC->TASKS_SAMPLE = 1;
  }
#endif

}

}

