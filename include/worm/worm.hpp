#pragma once
#ifndef WORM_WORM_HPP
#define WORM_WORM_HPP

#include <cstddef>
#include <utility>

namespace worm {

namespace detail {

enum class Direction { clockwise, counterclockwise };

enum class Neighborhood : int {
  NW = 0,
  N = 1,
  NE = 2,
  E = 3,
  SE = 4,
  S = 5,
  SW = 6,
  W = 7
};

struct RotationPolicy {};

class Clockwise : RotationPolicy {
protected:
  static inline Neighborhood next(const Neighborhood current) {
    return static_cast<Neighborhood>((static_cast<int>(current) + 1) % 8);
  }
};

class CounterClockwise : RotationPolicy {
private:
  static inline int remainder(int a, int b) {
    int m = a % b;
    if (m < 0) {
      m = (b < 0) ? m - b : m + b;
    }
    return m;
  }

protected:
  static inline Neighborhood next(const Neighborhood current) {
    return static_cast<Neighborhood>(
        remainder(static_cast<int>(current) - 1, 8));
  }
};

} // namespace detail

class TraceBoundary {};

} // namespace worm

#endif
