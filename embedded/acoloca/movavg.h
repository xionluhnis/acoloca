// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

#include "buffer.h"

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
