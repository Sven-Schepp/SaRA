/*
This file is part of Reach-RI.

Reach-RI is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
TUM, either version 3 of the License, or
(at your option) any later version.

Reach-RI is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Foobar.  If not, see https://www.gnu.org/licenses/.
*/

#include <string>
#include <tuple>
#include <vector>

#include "reach_lib.hpp"



#ifndef REACHABLE_OCCUPANCY_VALIDATION_HPP_
#define REACHABLE_OCCUPANCY_VALIDATION_HPP_

namespace validation {

//! This class defines the verification algorithm
//! used to determine wether past occupancies
//! cover the current positions
class Validation {
 public:
  //! \brief Empty constructor
  Validation(){}

  //! \brief Constructs an instance of Validation
  //! \param[in] delta_t The interval used to calculate occupancies
  //! \param[in] frame_duration The duration for which our occupancy prediction is not changed
  Validation(double delta_t, double frame_duration = 1.0/60.0, int backlog = 60);

  //! \brief Empty destructor
  ~Validation(){}

  //! \brief Determines whether previous occupancies enclose current Cartesian joint positions.
  //! Based on the ArticulatedAccel model.
  std::vector<bool> validation(reach_lib::ArticulatedAccel oc, double delta_t,
                               std::vector<reach_lib::Point> joints);

  //! \brief Determines whether previous occupancies enclose current Cartesian joint positions
  //! Based on the ArticulatedVel model
  std::vector<bool> validation(reach_lib::ArticulatedVel oc, double delta_t,
                               std::vector<reach_lib::Point> joints);

  //! \brief Determines whether previous occupancies enclose current Cartesian joint positions
  //! Based on the ArticulatedPos model
  std::vector<bool> validation(reach_lib::ArticulatedPos oc, double delta_t,
                               std::vector<reach_lib::Point> joints);

  //! \brief Determines whether previous occupancies enclose current Cartesian joint positions
  //! Based on the PedestrianAccel model
  std::vector<bool> validation(reach_lib::PedestrianAccel oc, double delta_t,
                               std::vector<reach_lib::Point> joints);

  //! \brief Determines whether previous occupancies enclose current Cartesian joint positions
  //! Based on the PedestrianVel model
  std::vector<bool> validation(reach_lib::PedestrianVel oc, double delta_t,
                               std::vector<reach_lib::Point> joints);

  //! \brief Calculates wether indiidual capsules from two sets intersect with any of the other set
  //! \param[in] set1 A set of capsules checked against intersection with set2
  //! \param[in] set2 A set of capsules checked against intersection with set1
  std::vector<bool> capsule_set_intersection(std::vector<reach_lib::Capsule> set1,
                                             std::vector<reach_lib::Capsule> set2);

  //! \brief Calculates wether individual cylinders intersect with any capsule within a set
  //! \param[in] sety A set of type Cylinder
  //! \param[in] seta A set of type Capsule
  std::vector<bool> cylinder_capsule_set_intersction(std::vector<reach_lib::Cylinder> sety,
                                                     std::vector<reach_lib::Capsule> seta);

  //! \brief Returns the frame_duration parameter
  inline double get_frame_duration(){
    return this->frame_duration_;
  }

  //! \brief Returns the delta_t parameter
  inline double get_delta_t(){
    return this->delta_t_;
  }

  //! \brief Alters the delta_t parameter
  //! dleta_t can be continuously updated
  inline double set_delta_t(double delta_t){
    this->delta_t_ = delta_t;
  }

  //! \brief Returns the number of stored occupancies
  inline int get_backlog(){
    return this->backlog_;
  }

  //! \brief Returns the intersection map parameter
  inline std::vector<bool> get_intersect() {
    return this->intersect_;
  }

  //! \brief Returns the list of verified occupancy entries
  inline std::vector<int> get_val_counter() {
    return this->val_counter_;
  }

  //! \brief Returns the list of previous occupancies [ArticulatedAccel]
  std::vector<std::pair<reach_lib::ArticulatedAccel, double>> get_a_accel() {
    return this->a_accel_;
  }

  //! \brief Returns the list of previous occupancies [ArticulatedVel]
  std::vector<std::pair<reach_lib::ArticulatedVel, double>> get_a_vel() {
    return this->a_vel_;
  }

  //! \brief Returns the list of previous occupancies [ArticulatedPos]
  std::vector<std::pair<reach_lib::ArticulatedPos, double>> get_a_pos() {
    return this->a_pos_;
  }

  //! \brief Returns the list of previous occupancies [PedestrianAccel]
  std::vector<std::pair<reach_lib::PedestrianAccel, double>> get_p_accel() {
    return this->p_accel_;
  }

  //! \brief Returns the list of previous occupancies [PedestrianVel]
  std::vector<std::pair<reach_lib::PedestrianVel, double>> get_p_vel() {
    return this->p_vel_;
  }

 private:
  //! \brief The interval used to calculate occupancies
  double delta_t_ = 0.0;

  //! \brief The duration for which our occupancy prediction is not changed in [s]
  double frame_duration_ = 1.0/60.0;

  //! \brief The number of frames that are eing rewound to check against 

  //! \brief The number of previous occupancies kept for reference; suggested: > 20
  int backlog_ = 60;

  //! \brief A map of joints and wether they lie within or outside the occupancy [1: inside, 0: outside]
  std::vector<bool> intersect_ = {};

  //! \brief A per model list of the currently relevant occupancy for validation checking
  //  Order: A_Accel, A_Vel, A_Pos, P_Accel, P_Vel
  std::vector<int> val_counter_ = {0, 0, 0, 0, 0};

  //! \brief Contains previous ArticulatedAccel occupancies to be checked against future positions
  std::vector<std::pair<reach_lib::ArticulatedAccel, double>> a_accel_ = {};

  //! \brief Contains previous ArticulatedVel occupancies to be checked against future positions
  std::vector<std::pair<reach_lib::ArticulatedVel, double>> a_vel_ = {};

  //! \brief Contains previous ArticulatedPos occupancies to be checked against future positions
  std::vector<std::pair<reach_lib::ArticulatedPos, double>> a_pos_ = {};

  //! \brief Contains previous PedestrianAccel occupancies to be checked against future positions
  std::vector<std::pair<reach_lib::PedestrianAccel, double>> p_accel_ = {};

  //! \brief Contains previous PedestrianVel occupancies to be checked against future positions
  std::vector<std::pair<reach_lib::PedestrianVel, double>> p_vel_ = {};

};
}  // namespace validation
#endif  // REACHABLE_OCCUPANCY_VALIDATION_HPP_
