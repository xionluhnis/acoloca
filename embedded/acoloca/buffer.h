// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

#include <Arduino.h>
#include <type_traits>

/**
 * Buffer for online filtering
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
