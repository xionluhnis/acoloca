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

// filters
NormalizationFilter<25> norm_filter;

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

  // initialize SAADC
  Serial.println("Initializing SAADC");
  saadc_setup();

  // initialize pll
  Serial.println("Initializing PLL");
  pll_setup();

  Serial.println("Start SAADC");
  saadc_start();
}

/**************************************************************************/
/*!
    @brief  The loop function runs over and over again forever
*/
/**************************************************************************/
void loop()
{
  // wait for some event
  __WFE();

  NRF_GPIO->OUT ^= 1 << LED1;
  //Serial.print("Sample: "); Serial.println(saadc_sample);
  //Serial.print("Count:  "); Serial.println(saadc_sample_count);
}





