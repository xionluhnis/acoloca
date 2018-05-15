// Alexandre Kaspar <akaspar@mit.edu>

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
}

/**************************************************************************/
/*!
    @brief  The loop function runs over and over again forever
*/
/**************************************************************************/
void loop()
{
  // CHIRP_INFO(tuning_word);
}





