/*
This file is part of SaRA.

SaRA is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
TUM, either version 3 of the License, or
(at your option) any later version.

SaRA is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details: https://www.gnu.org/licenses/.
*/

#include <cassert>
#include <string>
#include <vector>

#include "body_part.hpp"
#include "body_part_combined.hpp"
#include "capsule.hpp"
#include "occupancy.hpp"
#include "point.hpp"



namespace occupancies {
namespace body_parts {
namespace accel {

BodyPartCombined::BodyPartCombined(std::string name, double thickness, double max_v1, double max_v2, double max_a1, double max_a2) :
                             BodyPartAccel(name, thickness, max_a1, max_a2),
                             max_v1_(max_v1), max_v2_(max_v2) {
  //  NO TODO
}

Capsule BodyPartCombined::ry(const Point& p, const Point& v,
                          int index, double t_b, double delay,
                          double measurement_error_pos,
                          double measurement_error_vel) {
  Point y;
  y = p;
  Point dy = v;
  double v_0 = Point::norm(dy);

  double a_max = 0.0, v_max = 0.0;
  if (index == 1) {
    a_max = this->max_a1_;
    v_max = this->max_v1_;
  } else {
    a_max = this->max_a2_;
    v_max = this->max_v2_;
  }

  // Entire time of the movement
  double t = t_b + delay;
  // Time to reach maximum velocity from v_0
  double t_up = std::max(std::min((v_max - v_0)/a_max, t), 0.0);
  // Time to reach maximum velocity from 0
  double t_max = std::max(std::min(v_max/a_max, t), 0.0);
  // Time to reach negative maximum velocity from v_0
  double t_down = std::max(std::min((v_max + v_0)/a_max, t), 0.0);
  double radius_big_circle = 0.0;
  if (t_down < t || (t_down >=t && t_up >= t)) {
    // Movement is symetrical
    radius_big_circle = a_max * (t * t_max - 0.5 * std::pow(t_max, 2.0));
  } else {
    // Movement is not symetrical
    radius_big_circle = a_max * std::sqrt(
        1.0/4.0 * (t * (t_up+t_down) - 0.5 * std::pow(std::pow(t_up, 2.0) + std::pow(t_down, 2.0), 2.0)) +
        std::pow(t * t_max - 0.5 * std::pow(t_max, 2.0), 2.0));
  }
  // Caculate the new center of the ball.
  double center_shift = t + 0.5 * a_max/v_0 * (t*(t_up-t_down) - 0.5 * (std::pow(t_up, 2.0) - std::pow(t_down, 2.0)));
  Point center = y + dy * center_shift;
  return Capsule(center, center, radius_big_circle + measurement_error_pos + measurement_error_vel * t);
}
}  // namespace accel
}  // namespace body_parts
}  // namespace occupancies
