// Alexandre Kaspar <akaspar@mit.edu>
#pragma once

/**
 * Difference filter
 */
template <typename Type>
struct Difference {

  Type operator()(const Type &current) {
    Type previous = buffer;
    buffer = current;

    // compute difference
    return current - previous;
  }

  inline void reset(){
    buffer = 0;
  }

private:
  Type buffer;
};
