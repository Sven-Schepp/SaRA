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

#include <map>
#include <string>
#include <utility>
#include <vector>
#include <assert.h>

#include "articulated.hpp"
#include "articulated_pos.hpp"
#include "extremity.hpp"
#include "capsule.hpp"
#include "obstacle.hpp"
#include "system.hpp"


namespace obstacles {
namespace articulated {
namespace pos {

ArticulatedPos::ArticulatedPos(System system, std::map<std::string, jointPair> body_segment_map,
                               const std::vector<double>& thickness,
                               const std::vector<double>& max_v,
                               const std::vector<double>& length) :
                               Articulated(system, body_segment_map) {
  // Create a list of BodyPartsAccel that is later set as occpancy_
  assert(thickness.size() == max_v.size());
  assert(thickness.size() == length.size());
  std::vector<Extremity> body = {};
  int i=0;
  for (const auto& it : body_segment_map) {
    body.push_back(Extremity(it.first, thickness[i], length[i], max_v[i]));
    i++;
  }
  this->occupancy_ = body;
  // Initialize pointers
  for (int i = 0; i < this->occupancy_.size(); i++) {
    this->occupancy_p.push_back(&(this->occupancy_[i]));
  }
}

std::vector<Extremity> ArticulatedPos::update(double t_a, double t_b,
                                              std::vector<Point> p,
                                              std::vector<Point> v) {
  int count = 0;
  for (auto& it : this->occupancy_) {
    int p_id = this->body_segment_map_.at(it.get_name()).first;
    it.update({p[p_id]}, {}, t_a, t_b,
              this->system.measurement_error_pos_, 0.0,
              this->system.delay_);
    this->occupancy_[count] = it;
    count++;
  }
  return this->occupancy_;
}

const bool ArticulatedPos::intersection(std::vector<Point> targets) {
  for (auto& it : this->occupancy_) {
    if (it.intersection(targets)) {
      return true;
    }
  }
  return false;
}
}  // namespace pos
}  // namespace articulated
}  // namespace obstacles
