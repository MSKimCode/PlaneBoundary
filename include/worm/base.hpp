#pragma once
#ifndef WORM_BASE_HPP
#define WORM_BASE_HPP

#include <cstddef>
#include <limits>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace worm {

namespace detail {

/*
 * Rotation Policy
 */

enum class Neighborhood : int {
  NW = 0,
  N = 1,
  NE = 2,
  E = 3,
  SE = 4,
  S = 5,
  SW = 6,
  W = 7,
  C = 0
};

/*
Neighborhood inv(int dx, int dy) {
  using Way = Neighborhood;
  if (dx == -1) {
    if (dy == -1)
      return Way::NW;
    if (dy == 0)
      return Way::W;
    if (dy == 1)
      return Way::SW;
  }
  if (dx == 0) {
    if (dy == -1)
      return Way::N;
    if (dy == 1)
      return Way::S;
  }
  if (dx == 1) {
    if (dy == -1)
      return Way::NE;
    if (dy == 0)
      return Way::E;
    if (dy == 1)
      return Way::SE;
  }
  return Way::C;
}
*/

Neighborhood inv(int dx, int dy) {
  using Way = Neighborhood;
  if (dx == 1) {
    if (dy == -1)
      return Way::NW;
    if (dy == 0)
      return Way::W;
    if (dy == 1)
      return Way::SW;
  }
  if (dx == 0) {
    if (dy == -1)
      return Way::N;
    if (dy == 1)
      return Way::S;
  }
  if (dx == -1) {
    if (dy == -1)
      return Way::NE;
    if (dy == 0)
      return Way::E;
    if (dy == 1)
      return Way::SE;
  }
  return Way::C;
}

struct RotationPolicy {};

struct Clockwise : RotationPolicy {
public:
  static inline Neighborhood next(const Neighborhood current) {
    return static_cast<Neighborhood>((static_cast<int>(current) + 1) % 8);
  }
};

struct CounterClockwise : RotationPolicy {
private:
  static inline int remainder(int a, int b) {
    int m = a % b;
    if (m < 0) {
      m = (b < 0) ? m - b : m + b;
    }
    return m;
  }

public:
  static inline Neighborhood next(const Neighborhood current) {
    return static_cast<Neighborhood>(
        remainder(static_cast<int>(current) - 1, 8));
  }
};

} // namespace detail

using detail::Clockwise;
using detail::CounterClockwise;

} // namespace worm

#endif
