#include <nrf.h>

#define PIN_CLK  (15)
#define PIN_DIN  (16)


int main(void)
{
  Serial.begin(230400);
  // Serial.begin(1000000);

  int16_t sample = 0;
  while (1) {
    sample = analogRead(A0);
    // Serial.println(sample);
    // Serial.println(sample);
    // send directly the 2 bytes
    // Serial.write(sample >> 2); // ADC = 9bits real value => shift by 1 to keep 8 msbs
    uint16_t samp = sample;
    Serial.write(samp >> 8);
    Serial.write(samp & 0xFF);
  }
}
