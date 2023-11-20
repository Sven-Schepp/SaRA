#include <gtest/gtest.h>
#include <math.h>

#include <vector>
#include <iostream>

#include "body_part_combined_fixture.h"
#include "reach_lib.hpp"

namespace occupancies{
namespace body_parts {
namespace accel {

std::vector<Point> simulate_point_movement(
    Point y_0, Point dy_0, double delta_y, double delta_dy,
    double v_max, double a_max, double t) {
  Point c_vel = y_0;
  double r_vel = delta_y + delta_dy * t;
  double v_0 = Point::norm(dy_0);
  if (v_0 < 1e-12) {
    dy_0 = Point(1.0, 0.0, 0.0);
  } else {
    dy_0 = dy_0 * (1.0/v_0);
  }
  
  std::vector<Point> points;
  double precision_1 = 8.0;
  double precision_2 = 128.0;
  points.reserve((int) ((precision_2 + 1) * (precision_1 + 1)));
  for (double phi = -M_PI; phi <= M_PI; phi += 2.0*M_PI/precision_1) {
    Point point = c_vel + Point(std::cos(phi), std::sin(phi), 0) * r_vel;
    for (double theta = -M_PI; theta <= M_PI; theta += 2*M_PI/precision_2) {
      if (v_0 > v_max) {
        Point new_point = point + dy_0 * (v_0 * t);
        points.push_back(new_point);
        continue;
      }
      Point a = Point(std::cos(theta), std::sin(theta), 0) * a_max;
      double denom = std::pow(Point::norm(a), 2.0);
      double p = 2.0 * v_0 * Point::inner_dot(dy_0, a) / denom;
      double q = (std::pow(v_0, 2.0) * std::pow(Point::norm(dy_0), 2.0) - std::pow(v_max, 2.0)) / denom;
      double t_vmax = - p/2.0 + std::sqrt(std::pow(p/2.0, 2.0) - q);
      t_vmax = std::min(std::max(t_vmax, 0.0), t);
      double t_const = t - t_vmax;
      Point new_point = point + (dy_0 * (v_0 * t_vmax) + a * (1.0/2.0 * std::pow(t_vmax, 2.0))) +
          (dy_0 * v_0 + a * t_vmax) * t_const;
      points.push_back(new_point);
    }
  }
  return points;
}

void test_points(std::vector<Point> points, Capsule cap) {
  for (const auto& point : points) {
    EXPECT_TRUE(Capsule::point_capsule_intersection(cap, {point}, 0.0)) 
      << "Point = [" << point.x << ", " << point.y << ", " << point.z << "]"
      << " not in capsule: p1 = [" << cap.p1_.x << ", " << cap.p1_.y << ", " << cap.p1_.z << "], r = " << cap.r_;
  }
}

void run_test_suite(BodyPartCombined body, Point y_0, Point dy_0, double prediction_time, double delay,
  double measurement_error_pos, double measurement_error_vel, double v_max, double a_max) {
  double time = prediction_time + delay;
  std::vector<Point> simulated_points = simulate_point_movement(
    y_0, dy_0, measurement_error_pos, measurement_error_vel, v_max, a_max, time);
  body.update({y_0, y_0}, {dy_0, dy_0}, 0.0, prediction_time, measurement_error_pos, measurement_error_vel, delay);
  Capsule cap = body.get_occupancy();
  test_points(simulated_points, cap);
}

TEST_F(BodyPartCombinedTest, InitializationTest) {
  EXPECT_DOUBLE_EQ(0, 0);
}

TEST_F(BodyPartCombinedTest, ZeroInitialVelTest) {
  Point dy_0(0.0, 0.0, 0.0);
  Point y_0(0.0, 0.0, 0.0);
  double prediction_time = 0.18;
  double delay = 0.02;
  double time = prediction_time + delay;
  std::vector<Point> simulated_points = simulate_point_movement(
    y_0, dy_0, measurement_error_pos_, measurement_error_vel_, v_max_, a_max_, time);
  body_.update({y_0, y_0}, {dy_0, dy_0}, 0.0, prediction_time, measurement_error_pos_, measurement_error_vel_, delay);
  Capsule cap = body_.get_occupancy();
  test_points(simulated_points, cap);
  EXPECT_DOUBLE_EQ(cap.p1_.x, y_0.x);
  EXPECT_DOUBLE_EQ(cap.p1_.y, y_0.y);
  EXPECT_DOUBLE_EQ(cap.p1_.z, y_0.z);
  EXPECT_DOUBLE_EQ(cap.p2_.x, y_0.x);
  EXPECT_DOUBLE_EQ(cap.p2_.y, y_0.y);
  EXPECT_DOUBLE_EQ(cap.p2_.z, y_0.z);
  EXPECT_DOUBLE_EQ(cap.r_, 
    0.5 * a_max_ * std::pow(v_max_/a_max_, 2.0) + v_max_ * (time - v_max_/a_max_)
    + measurement_error_pos_ + time * measurement_error_vel_ + thickness_/2);
}

TEST_F(BodyPartCombinedTest, MultipleVelTest) {
  Point y_0(0.0, 0.0, 0.0);
  double prediction_time = 0.18;
  double delay = 0.02;
  int max_steps = 20;
  for (int i = 0; i < max_steps; i ++) {
    Point dy_0(v_max_ * float(i+1)/float(max_steps), 0.0, 0.0);
    run_test_suite(body_, y_0, dy_0, prediction_time, delay, measurement_error_pos_, measurement_error_vel_, v_max_, a_max_);
  }
}

TEST_F(BodyPartCombinedTest, MultipleVelTestDiag) {
  Point y_0(0.0, 0.0, 0.0);
  double prediction_time = 0.18;
  double delay = 0.02;
  int max_steps = 20;
  for (int i = 0; i < max_steps; i ++) {
    Point dy_0(v_max_/std::sqrt(3.0) * float(i+1)/float(max_steps), -v_max_/std::sqrt(3.0) * float(i+1)/float(max_steps), v_max_/std::sqrt(3.0) * float(i+1)/float(max_steps));
    run_test_suite(body_, y_0, dy_0, prediction_time, delay, measurement_error_pos_, measurement_error_vel_, v_max_, a_max_);
  }
}

TEST_F(BodyPartCombinedTest, MultipleTimeTestDiag) {
  Point y_0(0.0, 0.0, 0.0);
  Point dy_0(1.0, 0.0, 0.0);
  double prediction_time = 0.0;
  double max_time = 0.2;
  double delay = 0.00;
  int max_steps = 100;
  for (int i = 0; i < max_steps; i ++) {
    prediction_time = max_time * float(i)/float(max_steps);
    run_test_suite(body_, y_0, dy_0, prediction_time, delay, measurement_error_pos_, measurement_error_vel_, v_max_, a_max_);
  }
}

TEST_F(BodyPartCombinedTest, VelNegativeTest) {
  Point y_0(0.0, 0.0, 0.0);
  double prediction_time = 0.18;
  double delay = 0.02;
  int max_steps = 20;
  for (int i = 0; i < max_steps; i ++) {
    Point dy_0(0.0, 0.0, -1.0 * v_max_  * float(i+1)/float(max_steps));
    run_test_suite(body_, y_0, dy_0, prediction_time, delay, measurement_error_pos_, measurement_error_vel_, v_max_, a_max_);
  }
}

TEST_F(BodyPartCombinedTest, VelTooHighTest) {
  Point y_0(0.0, 0.0, 0.0);
  double prediction_time = 0.18;
  double delay = 0.02;
  int max_steps = 1;
  for (int i = 0; i < max_steps; i ++) {
    Point dy_0(2*v_max_, 0.0, 0.0);
    run_test_suite(body_, y_0, dy_0, prediction_time, delay, measurement_error_pos_, measurement_error_vel_, v_max_, a_max_);
  }
}

TEST_F(BodyPartCombinedTest, MeasErrorZeorTest) {
  Point y_0(0.0, 0.0, 0.0);
  double prediction_time = 0.18;
  double delay = 0.02;
  int max_steps = 1;
  for (int i = 0; i < max_steps; i ++) {
    Point dy_0(v_max_, 0.0, 0.0);
    run_test_suite(body_, y_0, dy_0, prediction_time, delay, 0.0, 0.0, v_max_, a_max_);
  }
}
}  // namespace accel
}  // namespace body_parts
}  // namespace occupancies
