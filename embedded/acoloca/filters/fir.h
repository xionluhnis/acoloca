// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

#include "buffer.h"

/**
 * Finite inpulse response filter
 */
template <typename Type, uint8_t Size>
struct FIRFilter {

  // @see https://stackoverflow.com/questions/5549524/how-do-i-initialize-a-member-array-with-an-initializer-list
  template <typename... T>
  explicit FIRFilter(T... w) : weights{w...} {}

  Type operator()(const Type &current) {
    // compute filter output
    Type output = current * weights[0];
    for(uint8_t i = 0; i < Size-1; ++i)
      output += buffer[i] * weights[i+1];
      
    // update buffer
    buffer.push(current);
    
    return output;
  }

  inline void reset(){
    buffer.reset();
  }

private:
  FilterBuffer<Type, Size-1> buffer;
  const Type weights[Size];
};
