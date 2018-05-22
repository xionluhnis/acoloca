// Alexandre Kaspar <akaspar@mit.edu>

#define CLOCK_ON(pin)  { NRF_GPIO->OUTSET = 1 << (pin); }
#define CLOCK_OFF(pin) { NRF_GPIO->OUTCLR = 1 << (pin); }

#define USE_FIXED_POINT 0
#define USE_PULSE

#include <Arduino.h>

#include "filters.h"

#if defined(USE_CHIRP)
#include "chirp.h"
#include "pll.h"
#elif defined(USE_PULSE)
#include "pulse.h"
#else
error "You must define a type of sound: USE_CHIRP or USE_PULSE"
#endif

#include "saadc.h"
#include "sync.h"

constexpr const uint16_t LED1 = 17;
constexpr const uint16_t LED2 = 19;

void on_sync_start();
void on_sync_end();

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
  init_sinetables(0.1);

  Serial.println("Initializing Chirp/PWM");
  sound_setup();

  // initialize SAADC
  Serial.println("Initializing SAADC");
  saadc_setup();

  // initialize synchronization
  Serial.println("Initializing Sync");
  sync_setup(&on_sync_start);

  // GPIO for debug
  NRF_GPIO->DIRSET = 1 << A3;
  NRF_GPIO->DIRSET = 1 << A4;
  NRF_GPIO->DIRSET = 1 << A5;
  NRF_GPIO->OUTCLR = 1 << A3 | 1 << A4 | 1 << A5;

  // start directly to listen for sync
  sync_listen(&on_sync_end);
}

enum State {
  IDLE,
  LISTENING,
  EMITTING
};
volatile State curr_state = IDLE;
volatile State last_state = IDLE;

unsigned long state_start = 0;
unsigned long sync_timestamp = 0;
unsigned long sync_count = 0;

void reset_to_idle(){
  // might be called multiple times
  if(curr_state == IDLE)
    return;

  // save previous state
  last_state = curr_state;
  curr_state = IDLE;

  // display timing automatically if available
  if(last_state == LISTENING && timestamp_new){
    Serial.print("Time delta: ");
    Serial.print(timestamps.first() - sync_timestamp, DEC);
    Serial.print(" of ");
    Serial.print(pulse_count2, DEC);
    Serial.println(" samples");

    /*
    for(int i = 0; i < 2000; ++i){
          Serial.print(pulse_samples[i], DEC);
          Serial.print(" ");
    }
    Serial.println();
    for(int i = 0; i < 2000; ++i){
          Serial.print(pulse_times[i] - pulse_times[0], DEC);
          Serial.print(" ");
    }
    Serial.println();
    */
  }

  // restart listening for sync
  sync_listen(&on_sync_end);
}

/**************************************************************************/
/*!
    @brief  The loop function runs over and over again forever
*/
/**************************************************************************/
void loop()
{
  unsigned long now = sync_micros();
  if(curr_state != IDLE){

    if(curr_state == LISTENING){
      NRF_GPIO->OUTCLR = 1 << LED1;
      NRF_GPIO->OUT   ^= 1 << LED2;
    } else if(curr_state == EMITTING){
      NRF_GPIO->OUTCLR = 1 << LED2;
      NRF_GPIO->OUT   ^= 1 << LED1;
    }

    // switch back to idle if waiting for too long
    if(now - state_start >= 2000000UL) {
      if(curr_state == LISTENING)
        saadc_end();
      else if(curr_state == EMITTING)
        sound_end();
      reset_to_idle();

      Serial.println("Switched back to idle");
      
    } else
      __WFE();
    
  } else {

    NRF_GPIO->OUTCLR = 1 << LED1 | 1 << LED2;

    // listen for commands
    char c = Serial.read();
    switch(c){
      
      case 'c':
      case 'e':
      case '!':
        // start synchronization
        sync_start();
        // some delay, quite important:
        // - to allow a signal to be generated (otherwise pulse is not visible)
        // - to allow other nodes to catch the signal
        delay(100);
        
        // switch to chirp mode
        curr_state = EMITTING;
        sound_start(&sync_end, &reset_to_idle);
        state_start = now;
        break;
        
      case 'l':
      case '?':

        // switch to listening mode
        curr_state = LISTENING;
        saadc_start(&reset_to_idle);
        state_start = now;
        break;
        
      case 'g':
      case '.':
        // send timing information
        Serial.println("Receiving time:");
        Serial.print("- start: ");
        Serial.println(sync_timestamp, DEC);
        Serial.print("- time: ");
        Serial.println(timestamps.first(), DEC);
        Serial.print("- delta: ");
        Serial.println(timestamps.first() - sync_timestamp, DEC);
        Serial.print("- valid: ");
        Serial.println(timestamp_new ? "true" : "false");
        timestamp_new = false;
        break;

      case 't':
        // send synchronization information
        Serial.println(sync_timestamp, DEC);
        break;

      case 'T':
        // send synchronization count
        Serial.println(sync_count, DEC);
        break;

      case 'd':
        // send debug information
        for(int i = 0; i < 2000; ++i){
          Serial.print(pulse_samples[i], DEC);
          Serial.print(" ");
        }
        Serial.println();
        break;

        case 'D':
        // send debug information
        for(int i = 0; i < 2000; ++i){
          Serial.print(pulse_times[i], DEC);
          Serial.print(" ");
        }
        Serial.println();
        break;

      case 'm':
        Serial.print("Time: ");
        Serial.println(sync_micros());
        break;

    }
    
  }
}

void on_sync_start() {
  // possible RACE condition with state change from UART?
  if(curr_state == IDLE){
    state_start = sync_micros();
    curr_state = LISTENING;
    saadc_start(&reset_to_idle);
  }
}

void on_sync_end() {
  // record starting time
  sync_timestamp = sync_micros();
  ++sync_count;
#if defined(USE_PULSE)
  pulse_init(sync_timestamp);
#endif
}



