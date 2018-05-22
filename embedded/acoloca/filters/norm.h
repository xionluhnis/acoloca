// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

#include <Arduino.h>

#include "buffer.h"
#include "movavg.h"

#ifndef USE_FIXED_POINT
#define USE_FIXED_POINT 0
#endif

// #if USE_FIXED_POINT
#include "../fixed/FixedPoints.h"
#include "../fixed/FixedPointsCommon.h"

FIXED_POINTS_BEGIN_NAMESPACE
using SQ2x13 = SFixed<2, 13>;
using UQ7x1  = UFixed<7, 1>;
FIXED_POINTS_END_NAMESPACE

// #endif

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
#if USE_FIXED_POINT
  typedef SQ15x16 AverageType;
  typedef SQ15x16 HigherSType;
  typedef SQ2x13  OutputType;
#else
  // typedef SQ15x16 AverageType;
  typedef float AverageType;
  typedef float HigherSType;
  typedef float  OutputType;
#endif

  NormalizationFilter() {
    count[0] = Size;
  }

  InputType range() {
    return max_value - min_value;
  }

  OutputType operator()(const InputType &x){
    // SQ2x13 = 8-13us
    // float  = 2-8us
    // NRF_GPIO->OUTSET = 1 << A1;
    
    //- update buffer
    // NRF_GPIO->OUTSET = 1 << A2;
    InputType last = buffer.last();
    buffer.push(x);
    // NRF_GPIO->OUTCLR = 1 << A2;

    // update count information
    count[x] += 1;
    count[last] -= 1;

    //- update bounds
    // NRF_GPIO->OUTSET = 1 << A3;
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
    // NRF_GPIO->OUTCLR = 1 << A3;
    
    //- mean-centering
    // NRF_GPIO->OUTSET = 1 << A4;
    AverageType x_mean = mavg(InputType(x));
    // NRF_GPIO->OUTCLR = 1 << A4;
    
    HigherSType x_cent = HigherSType(x) - HigherSType(x_mean);

    // range normalization
    InputType range = max_value - min_value;
    if(range){
      return OutputType(x_cent * 2 / range);
    } else {
      return OutputType(x_cent);
    }

    // NRF_GPIO->OUTCLR = 1 << A1;
  }
  
private:
  MovingAverage<InputType, AverageType, Size> mavg;
  FilterBuffer<InputType, Size> buffer;
  CounterType count[256];
  InputType min_value, max_value;
};
