// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

#include <Arduino.h>
#include "fixed/FixedPoints.h"
#include "fixed/FixedPointsCommon.h"

#include "buffer.h"
#include "movavg.h"


FIXED_POINTS_BEGIN_NAMESPACE
using SQ2x13 = SFixed<2, 13>;
using UQ7x1  = UFixed<7, 1>;
FIXED_POINTS_END_NAMESPACE

/**
 * Running normalization filter (of uint8_t signal)
 * 
 * - finds the running mean
 * - finds the running range
 * 
 * @return the normalized signal
 */
 template <int Size>
struct NormalizationFilter {

  // type of pointer (uint8_t or uint16_t)
  typedef typename std::conditional<Size <= UINT8_MAX,
                                    uint8_t,
                                    typename std::conditional<Size <= UINT16_MAX, uint16_t, uint32_t>::type>::type CounterType;

  NormalizationFilter() {
    count[0] = Size;
  }

  SQ2x13 operator()(const uint8_t &x){
    // update buffer
    uint8_t last = buffer.last();
    buffer.push(x);

    // update count information
    count[x] += 1;
    count[last] -= 1;
    
    // update maximum
    if(x > max_value){
      max_value = x;
    } else if(count[max_value] == 0){
      while(count[--max_value] == 0);
    }
    // update minimum
    if(x < min_value){
      min_value = x;
    } else if(count[min_value] == 0){
      while(count[++min_value] == 0);
    }
    
    // mean-centering
    UQ8x8 x_mean = mavg(UQ8x8(x));
    SQ15x16 x_cent = SQ15x16(x) - SQ15x16(x_mean);

    // range normalization
    uint8_t range = max_value - min_value;
    if(range){
      // UQ7x1 amplitude(range >> 1, range & 1); // = range / 2
      SQ15x16 amplitude(range >> 1, (range & 1) << 15);
      return SQ2x13(x_cent / amplitude);
    } else {
      return SQ2x13(x_cent);
    }
  }
  
private:
  MovingAverage<UQ8x8, UQ16x16, Size> mavg;
  FilterBuffer<uint8_t, Size> buffer;
  CounterType count[256];
  uint8_t min_value, max_value;
};