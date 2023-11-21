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
#include <iostream>

#include "body_part.hpp"
#include "body_part_combined.hpp"
#include "capsule.hpp"
#include "occupancy.hpp"
#include "point.hpp"



namespace occupancies {
namespace body_parts {
namespace combined {

BodyPartCombined::BodyPartCombined(std::string name, double thickness, double max_v1, double max_v2, double max_a1, double max_a2) :
                             BodyPart(name, thickness),
                             max_v1_(max_v1), max_v2_(max_v2), max_a1_(max_a1), max_a2_(max_a2) {
  //  NO TODO
}

Capsule BodyPartCombined::ry(const Point& p, const Point& v,
                          int index, double t_b, double delay,
                          double measurement_error_pos,
                          double measurement_error_vel) const {
  Point y;
  y = p;
  Point dy = v;
  double v_0 = Point::norm(dy);
  if (v_0 < 1e-12) {
    dy = Point(1.0, 0.0, 0.0);
  } else {
    dy = dy * (1.0/v_0);
  }

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

  if (v_0 > v_max) {
    // Point is already moving faster than v_max
    // We assume that the point does not accelerate further.
    // We use the constant velocity model with v_0 instead of v_max.
    return Capsule(p, p, measurement_error_pos + v_0 * t);
  }
  // Time to reach maximum velocity from v_0
  double t_up = std::max(std::min((v_max - v_0)/a_max, t), 0.0);
  // Time to reach maximum velocity from 0
  double t_max = std::max(std::min(v_max/a_max, t), 0.0);
  // Time to reach negative maximum velocity from v_0
  double t_down = std::max(std::min((v_max + v_0)/a_max, t), 0.0);
  double radius_big_circle = 0.0;
  if (t_down < t || (t_down >= t && t_up >= t)) {
    // Movement is symetrical
    radius_big_circle = a_max * (t * t_max - 0.5 * std::pow(t_max, 2.0));
  } else {
    // Movement is not symetrical
    radius_big_circle = a_max * std::sqrt(
        1.0/4.0 * (t * (t_up+t_down) - 0.5 * std::pow(std::pow(t_up, 2.0) + std::pow(t_down, 2.0), 2.0)) +
        std::pow(t * t_max - 0.5 * std::pow(t_max, 2.0), 2.0));
  }
  // Caculate the new center of the ball.
  double center_shift = t * v_0 + 0.5 * a_max * (t*(t_up-t_down) - 0.5 * (std::pow(t_up, 2.0) - std::pow(t_down, 2.0)));
  Point center = y + dy * center_shift;
  return Capsule(center, center, radius_big_circle + measurement_error_pos + measurement_error_vel * t);
}

bool BodyPartCombined::intersection(std::vector<Point> targets) const {
  return this->occupancy_.intersection(targets);
}

void BodyPartCombined::update(const std::vector<Point>& p,
                           const std::vector<Point>& v,
                           double t_a, double t_b,
                           double measurement_error_pos,
                           double measurement_error_vel,
                           double delay) {
  // The vectors must contain exactly 2 points
  assert(("Vector p must have exactly two entries for BodyPartCombined", size(p) == 2));
  assert(("Vector v must have exactly two entries for BodyPartCombined", size(v) == 2));
  // translate vector entries to proximal and distal joint values
  Point p1 = p[0];
  Point p2 = p[1];
  Point v1 = v[0];
  Point v2 = v[1];

  // Check whether this capsule is a ball
  bool b;
  if (p1 == p2) {
      b = true;
  } else {
      b = false;
  }

  // Calculate the occupancies of the proximal and distal joint up to t_a
  Capsule rp1_t1 = BodyPartCombined::ry(p1, v1, 1, t_a, delay,
                                     measurement_error_pos, measurement_error_vel);
  Capsule rp1_t2 = BodyPartCombined::ry(p1, v1, 1, t_b, delay,
                                     measurement_error_pos, measurement_error_vel);  // was p2
  Capsule b1 = Capsule::ballEnclosure(rp1_t1, rp1_t2);

  if (b) {
    // Add the thickness (as radius)
    rp1_t2.r_ += this->thickness_/2.0;

    this->occupancy_ = rp1_t2;  // was rp1_t1
  } else {
    Capsule rp2_t1 = BodyPartCombined::ry(p2, v2, 2, t_a, delay,
                                     measurement_error_pos, measurement_error_vel);  // was p1
    Capsule rp2_t2 = BodyPartCombined::ry(p2, v2, 2, t_b, delay,
                                     measurement_error_pos, measurement_error_vel);
    Capsule b2 = Capsule::ballEnclosure(rp2_t1, rp2_t2);

    // Add the thickness (as radius) to the ball with the larger radius (determines capsule radius)
    if (b1.r_ >= b2.r_) {
        b1.r_ += this->thickness_/2.0;
    } else {
        b2.r_ += this->thickness_/2.0;
    }
    this->occupancy_ = Capsule::capsuleEnclosure(b1, b2);
  }
}
}  // namespace Combined
}  // namespace body_parts
}  // namespace occupancies
