// Alexandre Kaspar <akaspar@mit.edu>

#define CLOCK_ON(pin)  { NRF_GPIO->OUTSET = 1 << (pin); }
#define CLOCK_OFF(pin) { NRF_GPIO->OUTCLR = 1 << (pin); }

#define USE_FIXED_POINT 0

#include <Arduino.h>
#include "chirp.h"
#include "filters.h"
#include "pll.h"
#include "saadc.h"

constexpr const uint16_t LED1 = 17;
constexpr const uint16_t LED2 = 19;

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

  Serial.println("Initializing Chirp/PWM");
  chirp_setup();

  // initialize SAADC
  Serial.println("Initializing SAADC");
  saadc_setup();

  // initialize pll
  Serial.println("Initializing PLL");
  pll_setup();
}

enum State {
  IDLE,
  LISTENING,
  EMITTING
};
volatile State curr_state = IDLE;
volatile State last_state = IDLE;

void reset_to_idle(){
  // might be called multiple times
  if(curr_state == IDLE)
    return;

  // save previous state
  last_state = curr_state;
  curr_state = IDLE;
}

unsigned long state_start = 0;

/**************************************************************************/
/*!
    @brief  The loop function runs over and over again forever
*/
/**************************************************************************/
void loop()
{
  unsigned long now = micros();
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
        chirp_end();
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
        // switch to chirp mode
        curr_state = EMITTING;
        chirp_start(&reset_to_idle);
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
        if(last_state == EMITTING){
          Serial.println("Emitting time:");
          Serial.println(chirp.start_us, DEC);
        }else if(last_state == LISTENING){
          Serial.println("Receiving time:");
          Serial.println(timestamps.first(), DEC);
        } else {
          Serial.println("Only idle");
        }
        break;
    }
    
  }
}





