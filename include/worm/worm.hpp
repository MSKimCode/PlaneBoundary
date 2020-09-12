#pragma once
#ifndef WORM_WORM_HPP
#define WORM_WORM_HPP

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
  W = 7
};

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

template <typename Image, typename RotationPolicy> class TraceBoundary {

public:
  using index_t = typename Image::index_t;
  using Point = std::pair<size_t, size_t>;

  TraceBoundary() = delete;
  TraceBoundary(const TraceBoundary &) = delete;
  TraceBoundary(TraceBoundary &&) noexcept = delete;

  TraceBoundary &operator=(const TraceBoundary &) = delete;
  TraceBoundary &operator=(TraceBoundary &&) noexcept = delete;

  TraceBoundary(const Image &, const Point, const long long int);

  const std::vector<Point> &run();

private:
  const Image &image;
  std::vector<Point> boundaries;
  const index_t x_max, y_max;
  const long long int max_iter;
  const Point pStart;

private:
  using Way = detail::Neighborhood;
  using Rot = RotationPolicy;

  bool in_boundary(Point) const;
  static Point move_point(Point, Way);
  bool initial_check() const;
  bool get(Point) const;
};

template <typename Image, typename RotationPolicy>
TraceBoundary<Image, RotationPolicy>::TraceBoundary(const Image &i,
                                                    const Point p,
                                                    const long long int l)
    : image{i}, pStart{p},
      max_iter(l > 0 ? l : std::numeric_limits<long long int>::max()),
      boundaries{}, x_max{i.size().first}, y_max{i.size().second} {}

template <typename Image, typename RotationPolicy>
auto TraceBoundary<Image, RotationPolicy>::in_boundary(Point p) const -> bool {
  auto [x, y] = p;
  return 0 <= x < x_max && 0 <= y < y_max;
}

template <typename Image, typename RotationPolicy>
auto TraceBoundary<Image, RotationPolicy>::move_point(Point pCenter, Way w)
    -> Point {
  switch (w) {
  case Way::NW:
    pCenter.first--;
    pCenter.second++;
    break;
  case Way::N:
    pCenter.second--;
    break;
  case Way::NE:
    pCenter.first++;
    pCenter.second--;
    break;
  case Way::E:
    pCenter.first++;
    break;
  case Way::SE:
    pCenter.first++;
    pCenter.second++;
    break;
  case Way::S:
    pCenter.second++;
    break;
  case Way::SW:
    pCenter.first--;
    pCenter.second++;
    break;
  case Way::W:
    pCenter.first--;
    break;
  default:
    break;
  }
  return pCenter;
}

template <typename Image, typename RotationPolicy>
auto TraceBoundary<Image, RotationPolicy>::initial_check() const -> bool {
  bool result = false;
  result = in_boundary(pStart) && result;
  return result;
}

template <typename Image, typename RotationPolicy>
auto TraceBoundary<Image, RotationPolicy>::get(Point p) const -> bool {
  if (in_boundary(p)) {
    return image(p.first, p.second);
  } else {
    return false;
  }
}

template <typename Image, typename RotationPolicy>
auto TraceBoundary<Image, RotationPolicy>::run() -> const std::vector<Point> & {
  Point pp = pStart, pc, pb;
  long long int iter = 0;

  /* Phase 1
   * Bottom to Top
   * Left to Right
   * Until meet BLACK!
   */

  bool found = false;
  while (!found) {
    if (get(pp)) {
      found = true;
    }

    if (found) {
      break;
    }

    pp.second--;
    bool x_bound = 0 <= pp.first < x_max;
    bool y_bound = 0 <= pp.second < y_max;

    if (!x_bound) {
      break; // Failed
    }

    if (!y_bound) {
      pp.first++;
      pp.second = y_max - 1;
    }
  }

  if (!found) {
    return boundaries;
  } else {
    boundaries.push_back(pp);
    iter++;
  }

  /* Phase 2 */

  pb = pp;
  pb.second++; // From bottom

  Way w = Way::S;
  pc = move_point(pp, w);

  while (true) {
    if (iter >= max_iter) {
      break;
    }

    if (iter >= 2 && pc == pStart) {
      break;
    }

    if (get(pc)) {              // If c is black
      boundaries.push_back(pc); // Insert c to B
      iter++;
      pb = pp; // Let b = p
      pp = pc; // Let p = c
      // Backtrack: Move the current pixel c to the pixel from which p was
      // entered.
      w = Rot::next(w);
      pc = move_point(pp, w); // Next rotated pixel (from b) in M(p).
    } else {
      // Advance the current pixel c to the next rotated pixel in M(p) and
      // update backtrack.
      pb = pc; // Let b = c
      w = Rot::next(w);
      pc = move_point(pp, w);
    }
  }

  return boundaries;
}

} // namespace worm

#endif
