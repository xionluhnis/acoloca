// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

#include <cassert>
#include <cstdint>
#include <limits>
#include <type_traits>

#include <Arduino.h>
#include "fixed/FixedPoints.h"
#include "fixed/FixedPointsCommon.h"

/**
 * Buffer for filtering
 */
template <typename Type, uint16_t Size>
struct FilterBuffer {
  // type of pointer (uint8_t or uint16_t)
  typedef typename std::conditional<Size <= UINT8_MAX, uint8_t, uint16_t>::type PointerType;

  inline Type &last() {
    return data[pointer]; // returns 0 until buffer is full
  }
  inline void push(const Type &p){
    data[pointer] = p;
    if(++pointer >= Size) {
      pointer = 0;
    }
  }
  inline void reset(Type value = 0) {
    memset(data, value, Size);
  }

private:
  Type data[Size];
  PointerType pointer;
};

/**
 * Moving average filter
 */
template <typename Type, typename SumType, uint16_t Size>
struct MovingAverage {

  SumType operator()(const Type &x_new) {
    // fill buffer
    SumType x_last = buffer.last();
    buffer.push(x_new);

    // compute running average
    return sum = (sum + SumType(x_new) / Size) - x_last / Size;
  }

  inline void reset(){
    buffer.reset();
    sum = 0;
  }

private:
  FilterBuffer<Type, Size> buffer;
  SumType sum;
};

/**
 * Weighted moving average filter
 */
template <typename Type, typename SumType, uint16_t Size>
struct WeightedMovingAverage {

  SumType operator()(const Type &x_new) {
    // fill buffer
    SumType x_last = buffer.last();
    buffer.push(x_new);

    // compute running average
    SumType total_m = total;
    total = total + x_new - x_last;
    numerator = numerator + Size * x_new - total_m;
    return numerator / ((Size-1)*Size/2);
  }

  inline void reset(){
    buffer.reset();
    total = 0;
    numerator = 0;
  }

private:
  FilterBuffer<Type, Size> buffer;
  SumType total, numerator;
};

using SQ2x13 = SFixed<2, 13>;
using UQ7x1  = UFixed<7, 1>;

/**
 * Running normalization filter (of uint8_t signal)
 * 
 * - finds the running mean
 * - finds the running range
 * 
 * @return the normalized signal
 */
 template <uint8_t Size>
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

    // range information
    uint8_t range = max_value - min_value;
    
    // mean-centering
    UQ8x8 x_mean = mavg(UQ8x8(x_new));
    SQ15x16 x_cent = SQ15x16(x_new) - x_mean;

    // range normalization
    if(range){
      UQ7x1 amplitude(range, 2);
      return SQ2x13(x_cent / amplitude);
    } else {
      return SQ2x13(x_cent);
    }
  }
  
private:
  MovingAverage<UQ8x8, UQ16,16, Size> mavg;
  FilterBuffer<uint8_t, Size> buffer;
  CounterType count[256];
  uint8_t min_value, max_value;
};

/**
 * Biquadratic filter
 */
struct BiquadFilter {
  
};

