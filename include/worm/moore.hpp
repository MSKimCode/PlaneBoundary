#pragma once
#ifndef WORM_MOORE_HPP
#define WORM_MOORE_HPP

#include "base.hpp"

namespace worm {

template <typename Image, typename RotationPolicy> class Moore {

public:
  using index_t = typename Image::index_t;
  // requires singed(index_t);
  using Point = std::pair<index_t, index_t>;

  Moore() = delete;
  Moore(const Moore &) = delete;
  Moore(Moore &&) noexcept = delete;

  Moore &operator=(const Moore &) = delete;
  Moore &operator=(Moore &&) noexcept = delete;

  Moore(const Image &, const Point, const long long int);

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

  bool in_boundary(const Point &) const;
  static Point move_point(Point, Way);
  bool initial_check() const;
  bool get(Point) const;
};

template <typename Image, typename RotationPolicy>
Moore<Image, RotationPolicy>::Moore(const Image &i, const Point p,
                                    const long long int l)
    : image{i}, pStart{p},
      max_iter(l > 0 ? l : std::numeric_limits<long long int>::max()),
      boundaries{}, x_max{i.size().first}, y_max{i.size().second} {}

template <typename Image, typename RotationPolicy>
auto Moore<Image, RotationPolicy>::in_boundary(const Point &p) const -> bool {
  return 0 <= p.first && p.first < x_max && 0 <= p.second && p.second < y_max;
}

template <typename Image, typename RotationPolicy>
auto Moore<Image, RotationPolicy>::move_point(Point pCenter, Way w) -> Point {
  switch (w) {
  case Way::NW:
    pCenter.first--;
    pCenter.second--;
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
auto Moore<Image, RotationPolicy>::initial_check() const -> bool {
  bool result = false;
  result = in_boundary(pStart) && result;
  return result;
}

template <typename Image, typename RotationPolicy>
auto Moore<Image, RotationPolicy>::get(Point p) const -> bool {
  if (in_boundary(p)) {
    return image(p.first, p.second);
  } else {
    return false;
  }
}

template <typename Image, typename RotationPolicy>
auto Moore<Image, RotationPolicy>::run() -> const std::vector<Point> & {
  Point pp = pStart, pc, pb, ps, pz;
  long long int iter = 0;

  /* Phase 1
   * Bottom to Top
   * Left to Right
   * Until meet BLACK!
   */

  if (get(pp)) {
    return boundaries;
  }

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
    ps = pp;

#ifdef DEBUG
    std::cerr << '(' << ps.first << ", " << ps.second << ") # Start"
              << std::endl;
#endif
    iter++;
  }

  /* Phase 2 */

  pz = pp;
  pz.second++; // From bottom

  Way w = Way::S, w_s;
  bool entered = false;
  pc = move_point(pp, w);

  while (true) {
    if (iter >= max_iter) {
      break;
    }

    if (!in_boundary(pc)) {
#ifdef DEBUG
      std::cerr << "# Met Boundary!" << std::endl;
#endif
      break;
    }

    if (iter >= 2 && pc == ps) {
      break;
    }

    /*
    if (pc == ps) {
      if (entered && w_s == w) {
        break;
      } else {
        entered = true;
        w_s = w;
      }
    }
    */

    if (get(pc)) {              // If c is black
      boundaries.push_back(pc); // Insert c to B
#ifdef DEBUG
      std::cerr << '(' << pc.first << ", " << pc.second << ')' << std::endl;
#endif
      iter++;
      pp = pc; // Let p = c
      // Backtrack: Move the current pixel c to the pixel from which p was
      // entered.
      index_t dx, dy;
      dx = -pz.first + pp.first;
      dy = +pz.second - pp.second;
      w = detail::inv(dx, dy);
    } else {
      // Advance the current pixel c to the next rotated pixel in M(p) and
      // update backtrack.
      pz = pc;
    }
    w = Rot::next(w);
    pc = move_point(pp, w); // Next rotated pixel (from b) in M(p).
  }

  return boundaries;
}

} // namespace worm

#endif
