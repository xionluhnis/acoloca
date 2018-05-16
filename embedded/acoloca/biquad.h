// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

#include <Arduino.h>

#define INV_SQRT2 (0.7071067811865475)

/**
 * Biquadratic filter
 */
 template <typename Type>
struct BiquadFilter {

  explicit BiquadFilter(Type a0 = 1, Type a1 = 0, Type a2 = 0, Type b0 = 1, Type b1 = 0, Type b2 = 0)
  : A{ a0, a1, a2 }, B{ b0, b1, b2 } {}

  Type operator()(const Type& x_new) {
    // compute new output
    Type y_new = B[0] * x_new + B[1] * x[0] + B[2] * x[1] - A[1] * y[0] - A[2] * y[1];
    
    // shift buffers
    y[1] = y[0];
    y[0] = y_new;
    x[1] = x[0];
    x[0] = x_new;

    // return new output
    return y_new;
  }

  static BiquadFilter lowpass(float freq, float srate, float Q = INV_SQRT2) {
    float omega = 2 * M_PI * freq / srate;
    float sn = sin(omega);
    float cs = cos(omega);
    float alpha = sn / (2.0 * Q);

    Type A[3] = { 1+alpha, -2*cs, 1-alpha };
    Type B[3] = { (1-cs)/2, 1-cs, (1-cs)/2 };
    
    return BiquadFilter(1.0f, A[1] / A[0], A[2] / A[0], B[0] / A[0], B[1] / A[0], B[2] / A[0]);
  }

  static BiquadFilter highpass(float freq, float srate, float Q = INV_SQRT2) {
    float omega = 2 * M_PI * freq / srate;
    float sn = sin(omega);
    float cs = cos(omega);
    float alpha = sn / (2.0 * Q);

    Type A[3] = { 1+alpha, -2*cs, 1-alpha };
    Type B[3] = { (1+cs)/2, -(1+cs), (1+cs)/2 };
    
    return BiquadFilter(1.0f, A[1] / A[0], A[2] / A[0], B[0] / A[0], B[1] / A[0], B[2] / A[0]);
  }

  static BiquadFilter bandpass(float freq, float srate, float Q = INV_SQRT2) {
    float omega = 2 * M_PI * freq / srate;
    float sn = sin(omega);
    float cs = cos(omega);
    float alpha = sn / (2.0 * Q);

    Type A[3] = { 1+alpha, -2*cs, 1-alpha };
    Type B[3] = { alpha, 0, -alpha };
    
    return BiquadFilter(1.0f, A[1] / A[0], A[2] / A[0], B[0] / A[0], B[1] / A[0], B[2] / A[0]);
  }

  static BiquadFilter identity() {
    return BiquadFilter();
  }

  void debug() {
    Serial.print("A: ");
    for(uint8_t i = 0; i < 3; ++i){
      Serial.print(A[i], 6);
      Serial.print(" ");
    }
    Serial.print("\nB: ");
    for(uint8_t i = 0; i < 3; ++i){
      Serial.print(B[i], 6);
      Serial.print(" ");
    }
    Serial.println();
  }

private:
  const Type A[3];
  const Type B[3];
  Type x[2];
  Type y[2];
};
