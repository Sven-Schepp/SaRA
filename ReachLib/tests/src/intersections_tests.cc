#include <gtest/gtest.h>

#include "reach_lib.hpp"

TEST(IntersectionsTest, ParallelVerticalCapsulesWithOverlappingHeights) {
  const reach_lib::Capsule robot(
      reach_lib::Point(0.0, 0.0, 0.25), reach_lib::Point(0.0, 0.0, 1.25), 0.45);
  const reach_lib::Capsule human(
      reach_lib::Point(0.6, 0.0, 0.0), reach_lib::Point(0.6, 0.0, 1.8), 0.35);

  EXPECT_NEAR(reach_lib::intersections::min_segment_distance(robot, human), 0.6, 1e-12);
  EXPECT_TRUE(reach_lib::intersections::capsule_capsule_intersection(robot, human));
}
