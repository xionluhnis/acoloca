// Alexandre Kaspar <akaspar@mit.edu>

#define USE_FIXED_POINT 0

#include <Arduino.h>
#include "chirp.h"
#include "filters.h"
#include "pll.h"

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

  // initialize pll
  pll_setup();
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
  float out = norm_filter(data);
  
  // run PLL loop
  pll_run(out);

  // demodulation
  pll_demod(now);
  
  out = zx_count * 0.1f;

  // send result back
  uint16_t uout = SQ2x13(out).getInternal();
  Serial.write(uout >> 8);    // MSB
  Serial.write(uout & 0xFF);  // LSB
}





