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
  // computation types
  typedef uint8_t InputType;
  typedef SQ15x16 AverageType;
  typedef SQ15x16 HigherSType;
  typedef SQ2x13  OutputType;

  NormalizationFilter() {
    count[0] = Size;
  }

  OutputType operator()(const InputType &x){
    // update buffer
    InputType last = buffer.last();
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
    AverageType x_mean = mavg(InputType(x));
    HigherSType x_cent = HigherSType(x) - HigherSType(x_mean);

    // range normalization
    InputType range = max_value - min_value;
    if(range){
      // UQ7x1 amplitude(range >> 1, range & 1); // = range / 2
      HigherSType amplitude(range >> 1, (range & 1) << (HigherSType::FractionSize - 1));
      return OutputType(x_cent / amplitude);
    } else {
      return OutputType(x_cent);
    }
  }
  
private:
  MovingAverage<InputType, AverageType, Size> mavg;
  FilterBuffer<InputType, Size> buffer;
  CounterType count[256];
  InputType min_value, max_value;
};
