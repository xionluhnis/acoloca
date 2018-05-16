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