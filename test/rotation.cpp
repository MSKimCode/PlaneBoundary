#include <gtest/gtest.h>
#include <worm/worm.hpp>
// #include <concepts>

using namespace worm;

namespace wrom_test {

using nb = detail::Neighborhood;

template <typename RotationPolicy> class RotTest : public RotationPolicy {
public:
  using RotationPolicy::next;
};

TEST(rotation_clockwise, next) {
  using rot_policy = detail::Clockwise;
  RotTest<rot_policy> c;

  EXPECT_EQ(c.next(nb::NW), nb::N);
  EXPECT_EQ(c.next(nb::N), nb::NE);
  EXPECT_EQ(c.next(nb::NE), nb::E);
  EXPECT_EQ(c.next(nb::E), nb::SE);
  EXPECT_EQ(c.next(nb::SE), nb::S);
  EXPECT_EQ(c.next(nb::S), nb::SW);
  EXPECT_EQ(c.next(nb::SW), nb::W);
  EXPECT_EQ(c.next(nb::W), nb::NW);
}

TEST(rotation_counterclockwise, next) {
  using rot_policy = detail::CounterClockwise;
  RotTest<rot_policy> c;

  EXPECT_EQ(c.next(nb::NW), nb::W);
  EXPECT_EQ(c.next(nb::N), nb::NW);
  EXPECT_EQ(c.next(nb::NE), nb::N);
  EXPECT_EQ(c.next(nb::E), nb::NE);
  EXPECT_EQ(c.next(nb::SE), nb::E);
  EXPECT_EQ(c.next(nb::S), nb::SE);
  EXPECT_EQ(c.next(nb::SW), nb::S);
  EXPECT_EQ(c.next(nb::W), nb::SW);
}

} // namespace wrom_test
