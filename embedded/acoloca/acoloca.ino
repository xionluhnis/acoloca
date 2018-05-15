// Alexandre Kaspar <akaspar@mit.edu>

#include <Arduino.h>
#include "chirp.h"
#include "filters.h"

/**************************************************************************/
/*!
    @brief  The setup function runs once when reset the board
*/
/**************************************************************************/
void setup()
{
  Serial.begin(9600);
  
  Serial.println("Generating sin/cos tables");
  init_sinetables(1);

  /*
  Serial.println("Setup chirp");
  chirp_setup();
  */
  
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





