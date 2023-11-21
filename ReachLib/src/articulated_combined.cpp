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

#include "articulated_combined.hpp"


namespace obstacles {
namespace articulated {
namespace combined {

ArticulatedCombined::ArticulatedCombined(System system, std::map<std::string, jointPair> body_segment_map,
                   const std::map<std::string, double>& thickness,
                   const std::vector<double>& max_v,
                   const std::vector<double>& max_a) :
    Articulated(system, body_segment_map) {
  // Create a list of BodyPartCombined that is later set as occpancy_
  std::vector<BodyPartCombined> body = {};
  for (const auto& it : body_segment_map) {
    body.push_back(BodyPartCombined(it.first, thickness.at(it.first), 
                                    max_v.at(it.second.first), max_v.at(it.second.second),
                                    max_a.at(it.second.first), max_a.at(it.second.second)));
  }
  this->occupancy_ = body;
  // Initialize pointers
  for (int i = 0; i < this->occupancy_.size(); i++) {
    this->occupancy_p.push_back(&(this->occupancy_[i]));
  }
}

std::vector<BodyPartCombined> ArticulatedCombined::update(double t_a, double t_b,
                                                    std::vector<Point> p,
                                                    std::vector<Point> v) {
  // std::cout << "Length: " << this->get_occupancy_().size() << "\n";
  int count = 0;
  for (auto& it : this->occupancy_) {
    int p1_id = this->body_segment_map_.at(it.get_name()).first;
    int p2_id = this->body_segment_map_.at(it.get_name()).second;
    it.update({p[p1_id], p[p2_id]}, {v[p1_id], v[p2_id]}, t_a, t_b,
              this->system.measurement_error_pos_,
              this->system.measurement_error_vel_,
              this->system.delay_);
    this->occupancy_[count] = it;
    count++;
  }
  return this->occupancy_;
}

bool ArticulatedCombined::intersection(std::vector<Point> targets) const {
  for (auto& it : this->occupancy_) {
    if (it.intersection(targets)) {
      return true;
    }
  }
  return false;
}
}  // namespace combined
}  // namespace articulated
}  // namespace obstacles
