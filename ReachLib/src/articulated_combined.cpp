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
namespace accel {

ArticulatedCombined::ArticulatedCombined(System system, std::map<std::string, jointPair> body_segment_map,
                   const std::map<std::string, double>& thickness,
                   const std::vector<double>& max_v,
                   const std::vector<double>& max_a) :
    ArticulatedAccel(system, body_segment_map) {
  // Create a list of BodyPartAccel that is later set as occpancy_
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
}  // namespace accel
}  // namespace articulated
}  // namespace obstacles
