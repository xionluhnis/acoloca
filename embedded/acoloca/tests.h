// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

void test_SQ2x13(){

    typedef SQ2x13 QNumber;
    QNumber out = -1.5;
    uint16_t uout = out.getInternal();

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
    
}

void test_pll(){
  
  // read data
  uint8_t data = Serial.read();
  if(!data)
    return;

  // record time of arrival
  timestamp_t now = micros();

  // send header
  Serial.write(uint8_t(0xFF));
  Serial.write(uint8_t(0x01));

  CLOCK_ON(A3);
  
  // filter data (~2us, peaks at 6.5us)
  float out = norm_filter(data);
  
  // run PLL loop (4.6us)
  pll_run(out);
  
  // demodulation (1.2us)
  pll_demod(now);

  CLOCK_OFF(A3);
  
  out = zx_count * 0.1f;

  // send result back
  uint16_t uout = SQ2x13(out).getInternal();
  Serial.write(uout >> 8);    // MSB
  Serial.write(uout & 0xFF);  // LSB
  
}

timestamp_t last = 0;

void test_saadc_default(){
  // analogReadResolution(8); // in setup
  
  timestamp_t now = micros();

  last = now;

  NRF_GPIO->OUTSET = 1 << A1;
  uint8_t data = analogRead(A5);
  NRF_GPIO->OUTCLR = 1 << A1;
}

void test_saadc_optimized(){
  // analogReadResolution(8); // in setup
  
  timestamp_t now = micros();

  last = now;

  NRF_GPIO->OUTSET = 1 << A1;
  uint8_t data = saadc_read();
  NRF_GPIO->OUTCLR = 1 << A1;
}

