// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

#include "buffer.h"

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
