// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

/**
 * Integer tables
 */
#define SEQ_LENGTH 256   
static uint16_t sin256[SEQ_LENGTH];
static uint16_t cos256[SEQ_LENGTH];

/**
 * Floating point tables
 */
static float fsin256[SEQ_LENGTH];
static float fcos256[SEQ_LENGTH];

/**
 * Initialize the sine tables
 */
void init_sinetables(float amplitude = 1.0){
  // compute sine table
  uint16_t sine_mid = amplitude * (SEQ_LENGTH / 2 - 1);
  for(int i = 0; i < SEQ_LENGTH; ++i){
    float theta = 2 * M_PI * i / SEQ_LENGTH;
    sin256[i] = round(sine_mid + sine_mid * sin(theta));
    cos256[i] = round(sine_mid + sine_mid * cos(theta));
    fsin256[i] = sin(theta);
    fcos256[i] = cos(theta);
  }
}