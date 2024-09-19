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


//! This file includes all library headers and
//! provides type definitions that simplify access
//! to classes and functions defined in reach_lib.
//! Including this file in your main project might
//! might lead to overhead if not all contents of
//! reach_lib are used and can reult in conflicting
//! type definitions.
//! Careful inclusion is highly advised!!!


//! All headers contained in reach_lib
#include "articulated.hpp"
#include "articulated_accel.hpp"
#include "articulated_combined.hpp"
#include "articulated_pos.hpp"
#include "articulated_vel.hpp"
#include "body_part.hpp"
#include "body_part_accel.hpp"
#include "body_part_combined.hpp"
#include "body_part_vel.hpp"
#include "capsule.hpp"
#include "cylinder.hpp"
#include "aabb.hpp"
#include "cylinder_perimeter.hpp"
#include "extremity.hpp"
#include "intersections.hpp"
#include "occupancy_container.hpp"
#include "occupancy.hpp"
#include "pedestrian.hpp"
#include "pedestrian_accel.hpp"
#include "pedestrian_vel.hpp"
#include "point.hpp"
#include "obstacle.hpp"
#include "sphere.hpp"
#include "system.hpp"

#include <map>
#include <string>
#include <functional>

#ifndef REACH_LIB_INCLUDE_REACH_LIB_HPP_
#define REACH_LIB_INCLUDE_REACH_LIB_HPP_

//! \namespace Shortcuts to direclty accessible types
namespace reach_lib {
//! Base data types
typedef point::Point Point;
typedef systems::System System;
typedef obstacles::articulated::jointPair jointPair;

//! Occupancy containers
typedef occupancy_containers::capsule::Capsule Capsule;
typedef occupancy_containers::cylinder::Cylinder Cylinder;
typedef occupancy_containers::aabb::AABB AABB;

//! Occupancy models
typedef occupancies::body_parts::combined::BodyPartCombined BodyPartCombined;
typedef occupancies::body_parts::accel::BodyPartAccel BodyPartAccel;
typedef occupancies::body_parts::vel::BodyPartVel BodyPartVel;
typedef occupancies::extremities::Extremity Extremity;
typedef occupancies::cylinder_perimeters::CylinderPerimeter CylinderPerimeter;

//! Articulated models
typedef obstacles::articulated::Articulated Articulated;
typedef obstacles::articulated::combined::ArticulatedCombined ArticulatedCombined;
typedef obstacles::articulated::accel::ArticulatedAccel ArticulatedAccel;
typedef obstacles::articulated::pos::ArticulatedPos ArticulatedPos;
typedef obstacles::articulated::vel::ArticulatedVel ArticulatedVel;

//! Pedestrian models
typedef obstacles::pedestrian::Pedestrian Pedestrian;
typedef obstacles::pedestrian::accel::PedestrianAccel PedestrianAccel;
typedef obstacles::pedestrian::vel::PedestrianVel PedestrianVel;


inline std::vector<Capsule> get_capsules(const ArticulatedCombined& a_comb) {
  std::vector<Capsule> capsules;
  for (const auto& it : a_comb.get_occupancy()) {
    capsules.push_back(it.get_occupancy());
  }
  return capsules;
}

inline std::vector<Capsule> get_capsules(const ArticulatedAccel& a_accel) {
  std::vector<Capsule> capsules;
  for (auto it : a_accel.get_occupancy()) {
    capsules.push_back(it.get_occupancy());
  }
  return capsules;
}

inline std::vector<Capsule> get_capsules(const ArticulatedVel& a_vel) {
  std::vector<Capsule> capsules;
  for (auto& it : a_vel.get_occupancy()) {
    capsules.push_back(it.get_occupancy());
  }
  return capsules;
}

inline std::vector<Capsule> get_capsules(const ArticulatedPos& a_pos) {
  std::vector<Capsule> capsules;
  for (auto& it : a_pos.get_occupancy()) {
    capsules.push_back(it.get_occupancy());
  }
  return capsules;
}

inline std::vector<Cylinder> get_cylinders(const PedestrianAccel& p_accel) {
  std::vector<Cylinder> cylinders;
  for (auto& it : p_accel.get_occupancy()) {
    cylinders.push_back(it.get_occupancy());
  }
  return cylinders;
}

inline std::vector<Cylinder> get_cylinders(const PedestrianVel& p_vel) {
  std::vector<Cylinder> cylinders;
  for (auto& it : p_vel.get_occupancy()) {
    cylinders.push_back(it.get_occupancy());
  }
  return cylinders;
}

// Map the models to the type string
// E.g. "ARICULATED-POS" -> reach_lib::ArticulatedPos
inline std::vector<Capsule> get_capsules_pos_wrapper(const Articulated& a)
{
  return get_capsules(static_cast<const ArticulatedPos&>(a));
}

inline std::vector<Capsule> get_capsules_vel_wrapper(const Articulated& a)
{
  return get_capsules(static_cast<const ArticulatedVel&>(a));
}

inline std::vector<Capsule> get_capsules_accel_wrapper(const Articulated& a)
{
  return get_capsules(static_cast<const ArticulatedAccel&>(a));
}

inline std::vector<Capsule> get_capsules_combined_wrapper(const Articulated& a)
{
  return get_capsules(static_cast<const ArticulatedCombined&>(a));
}

const std::map<std::string, std::function<std::vector<Capsule>(const Articulated&)>> get_capsule_map = {
  {"ARTICULATED-POS", get_capsules_pos_wrapper},
  {"ARTICULATED-VEL", get_capsules_vel_wrapper},
  {"ARTICULATED-ACCEL", get_capsules_accel_wrapper},
  {"ARTICULATED-COMBINED", get_capsules_combined_wrapper}
};


//! Intersection functions
namespace intersections = occupancy_containers::intersections;

}  // namespace reach_lib
#endif  // REACH_LIB_INCLUDE_REACH_LIB_HPP_
