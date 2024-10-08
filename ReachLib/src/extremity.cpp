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

#include "capsule.hpp"
#include "extremity.hpp"
#include "occupancy.hpp"
#include "point.hpp"


namespace occupancies {
namespace extremities {

Extremity::Extremity() : Occupancy() {
  this->occupancy_p = &this->occupancy_;
}

Extremity::Extremity(std::string name, double thickness,
                     double length, double max_v) :
                     length_(length), max_v_(max_v), Occupancy(name, thickness) {
  //  NO TODO
}

void Extremity::update(const std::vector<Point>& p,
                       const std::vector<Point>& v,
                       double t_a, double t_b,
                       double measurement_error_pos,
                       double measurement_error_vel,
                       double delay) {
  // The vectors must contain exactly 2 points
  assert(("Vector p must have exactly two entries for BodyPartAccel", size(p) == 1));
  // translate vector entries to proximal and distal joint values
  Point pe = p[0];
  //  Total time passed within the desired interval
  assert(t_b >= t_a);
  double delta_t = t_b + delay;
  //  Distance covered at max_v over delta_t
  double dist = this->max_v_ * delta_t;
  // Radius of the ball occupancy (thickness added as radius)
  double radius = dist + this->length_ + this->thickness_/2.0 + measurement_error_pos;
  this->occupancy_ = Capsule(pe, pe, radius);
}

bool Extremity::intersection(std::vector<Point> targets) const {
  return this->occupancy_.intersection(targets);
}

}  // namespace extremities
}  // namespace occupancies
