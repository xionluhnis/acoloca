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

  inline const Type &operator[](PointerType i) const {
    return data[(Size - 1 - i + pointer) % Size];
  }
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

// partial specialization for single unit buffer
template <typename Type>
struct FilterBuffer<Type, 1> {

  inline const Type &operator[](uint8_t i) const {
    return data;
  }
  inline Type &last() {
    return data;
  }
  inline void push(const Type &p){
    data = p;
  }
  inline void reset(Type value = 0) {
    data = 0;
  }
private:
  Type data;
};

