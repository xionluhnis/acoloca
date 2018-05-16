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

  if(data){
    
    // filter data
    NRF_GPIO->OUTSET = 1 << A1;
    SQ2x13 out = normFilter(data);
    NRF_GPIO->OUTCLR = 1 << A1;

    /*
    typedef SQ2x13 QNumber;
    QNumber out = -1.5;
    */

    // send result back
    uint16_t uout = out.getInternal();

    /*

    Serial.println("Scale: ");
    Serial.println(QNumber::Scale * 1.0);
    Serial.println("IdentityMask");
    Serial.println(QNumber::IdentityMask, BIN);
    Serial.println("Inverse IdentityMask");
    Serial.println(~QNumber::IdentityMask, BIN);
    Serial.print("MSB=");
    Serial.println(uout >> 8, BIN);
    Serial.print("LSB=");
    Serial.println(uout & 0xFF, BIN);
    // */

    // /*
    // send header
    Serial.write(uint8_t(0xFF));
    Serial.write(uint8_t(0x01));

    // send data
    Serial.write(uout >> 8);    // MSB
    Serial.write(uout & 0xFF);  // LSB
  }
}





