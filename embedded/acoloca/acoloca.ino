// Alexandre Kaspar <akaspar@mit.edu>

#define USE_FIXED_POINT 0

#include <Arduino.h>
#include "chirp.h"
#include "filters.h"

#define SAMPLE_RATE (8704)

NormalizationFilter<25> normFilter;

/**************************************************************************/
/*!
    @brief  The setup function runs once when reset the board
*/
/**************************************************************************/
void setup()
{
  Serial.begin(230400);

  /*
  Serial.println("################");
  Serial.println("Starting ACOLOCA");
  Serial.println("################");
  
  Serial.println("Generating sin/cos tables");
  init_sinetables(1);
  */

  /*
  Serial.println("Setup chirp");
  chirp_setup();
  */
  /*
  Serial.println("Initializing GPIO");
  */
  NRF_GPIO->DIRSET = 1 << A1; // IRQ timing (to check sanity)
  NRF_GPIO->DIRSET = 1 << A2; // phase 8-bit overflow (to measure frequency)
  NRF_GPIO->DIRSET = 1 << A3; // real pwm frequency (to tune reference frequency)
  NRF_GPIO->DIRSET = 1 << A4; // chirp duration
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

  // send result back
  uint16_t uout = SQ2x13(out).getInternal();
  Serial.write(uout >> 8);    // MSB
  Serial.write(uout & 0xFF);  // LSB
}





