/*
This file is part of Reach-RI.

Reach-RI is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
TUM, either version 3 of the License, or
(at your option) any later version.

Reach-RI is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details: https://www.gnu.org/licenses/.
*/

#include <string>
#include <tuple>
#include <vector>

#include "validation.hpp"

#include "reach_lib.hpp"


namespace validation {

Validation::Validation(double delta_t, double frame_duration, int backlog) :
                       delta_t_(delta_t), frame_duration_(frame_duration), backlog_(backlog) {
  // NO TODO
}

std::vector<bool> Validation::validation(reach_lib::ArticulatedAccel oc, double delta_t,
                                         std::vector<reach_lib::Point> joints) {
  this->intersect_ = {};
  reach_lib::ArticulatedAccel c_oc;

  // If the queue is already full remove the oldest entry
  if (this->a_accel_.size() >= this->backlog_) {
    this->a_accel_.erase(this->a_accel_.begin());
  }
  std::pair<reach_lib::ArticulatedAccel, double> p{oc, delta_t};
  this->a_accel_.push_back(p);
  
  // Get the occupancy corresponding to the
  // interval delta_t
  int count = this->a_accel_.size() - 1;
  while (a_accel_[count].second > this->frame_duration_ * (this->a_accel_.size() - 1 - count) && count > 0) {
    count--;
  }
  this->val_counter_[0] = count;

  if (count > this->a_accel_.size()) {
    c_oc = this->a_accel_[this->a_accel_.size() - 1].first;
  } else {
    c_oc = this->a_accel_[count].first;
  }

  // If the previous occupancies are correct,
  // all joints should intersect with the occupancy
  for (int i = 0; i < joints.size(); i++) {
    for (auto& it : c_oc.get_occupancy()) {
      if (it.get_occupancy().intersection({joints[i]})) {
        this->intersect_.push_back(true);
        break;
      }
    }
    if (this->intersect_.size() < i+1) {
      this->intersect_.push_back(false);
    }
  }

  for (int i = 0; i < this->intersect_.size(); i++) {
    if (this->intersect_[i] == false) {
      std::cout << "Attention joint " << i <<
      " not contained in occupancy at t-" << count << "!\n";
    }
  }
  return this->intersect_;
}

std::vector<bool> Validation::validation(reach_lib::ArticulatedVel oc, double delta_t,
                                         std::vector<reach_lib::Point> joints) {
  this->intersect_ = {};
  reach_lib::ArticulatedVel c_oc;

  // If the queue is already full remove the oldest entry
  if (this->a_vel_.size() >= this->backlog_) {
    this->a_vel_.erase(this->a_vel_.begin());
  }
  std::pair<reach_lib::ArticulatedVel, double> p{oc, delta_t};
  this->a_vel_.push_back(p);
  

  // Get the occupancy corresponding to the
  // interval delta_t
  int count = this->a_vel_.size() - 1;
  while (a_vel_[count].second > this->frame_duration_ * (this->a_vel_.size() - 1 - count) && count > 0) {
    count--;
  }
  this->val_counter_[1] = count;

  if (count > this->a_vel_.size()) {
    c_oc = this->a_vel_[this->a_vel_.size() - 1].first;
  } else {
    c_oc = this->a_vel_[count].first;
  }

  // If the previous occupancies are correct,
  // all joints should intersect with the occupancy
  for (int i = 0; i < joints.size(); i++) {
    for (auto& it : c_oc.get_occupancy()) {
      if (it.get_occupancy().intersection({joints[i]})) {
        this->intersect_.push_back(true);
        break;
      }
    }
    if (this->intersect_.size() < i+1) {
      this->intersect_.push_back(false);
    }
  }

  for (int i = 0; i < this->intersect_.size(); i++) {
    if (this->intersect_[i] == false) {
      std::cout << "Attention joint " << i <<
      " not contained in occupancy at t-" << count << "!\n";
    }
  }
  return this->intersect_;
}

std::vector<bool> Validation::validation(reach_lib::ArticulatedPos oc, double delta_t,
                                         std::vector<reach_lib::Point> joints) {
  this->intersect_ = {};
  reach_lib::ArticulatedPos c_oc;

  // If the queue is already full remove the oldest entry
  if (this->a_pos_.size() >= this->backlog_) {
    this->a_pos_.erase(this->a_pos_.begin());
  }
  std::pair<reach_lib::ArticulatedPos, double> p{oc, delta_t};
  this->a_pos_.push_back(p);

  // Get the occupancy corresponding to the
  // interval delta_t
  int count = this->a_pos_.size() - 1;
  while (a_pos_[count].second > this->frame_duration_ * (this->a_pos_.size() - 1 - count) && count > 0) {
    count--;
  }
  this->val_counter_[2] = count;

  if (count > this->a_pos_.size()) {
    c_oc = this->a_pos_[this->a_pos_.size() - 1].first;
  } else {
    c_oc = this->a_pos_[count].first;
  }

  // If the previous occupancies are correct,
  // all joints should intersect with the occupancy
  for (int i = 0; i < joints.size(); i++) {
    for (auto& it : c_oc.get_occupancy()) {
      if (it.get_occupancy().intersection({joints[i]})) {
        this->intersect_.push_back(true);
        break;
      }
    }
    if (this->intersect_.size() < i+1) {
      this->intersect_.push_back(false);
    }
  }

  for (int i = 0; i < this->intersect_.size(); i++) {
    if (this->intersect_[i] == false) {
      std::cout << "Attention joint " << i <<
      " not contained in occupancy at t-" << count << "!\n";
    }
  }
  return this->intersect_;
}

std::vector<bool> Validation::validation(reach_lib::PedestrianAccel oc, double delta_t,
                                         std::vector<reach_lib::Point> joints) {
  this->intersect_ = {};
  reach_lib::PedestrianAccel c_oc;

  // If the queue is already full remove the oldest entry
  if (this->p_accel_.size() >= this->backlog_) {
    this->p_accel_.erase(this->p_accel_.begin());
  }
  std::pair<reach_lib::PedestrianAccel, double> p{oc, delta_t};
  this->p_accel_.push_back(p);

  // Get the occupancy corresponding to the
  // interval delta_t
  int count = this->p_accel_.size() - 1;
  while (p_accel_[count].second > this->frame_duration_ * (this->p_accel_.size() - 1 - count) && count > 0) {
    count--;
  }
  this->val_counter_[3] = count;

  if (count > this->p_accel_.size()) {
    c_oc = this->p_accel_[this->p_accel_.size() - 1].first;
  } else {
    c_oc = this->p_accel_[count].first;
  }

  // If the previous occupancies are correct,
  // all joints should intersect with the occupancy
  for (int i = 0; i < joints.size(); i++) {
    for (auto& it : c_oc.get_occupancy()) {
      if (it.get_occupancy().intersection({joints[i]})) {
        this->intersect_.push_back(true);
        break;
      }
    }
    if (this->intersect_.size() < i+1) {
      this->intersect_.push_back(false);
    }
  }

  for (int i = 0; i < this->intersect_.size(); i++) {
    if (this->intersect_[i] == false) {
      std::cout << "Attention joint " << i <<
      " not contained in occupancy at t-" << count << "!\n";
    }
  }
  return this->intersect_;
}

std::vector<bool> Validation::validation(reach_lib::PedestrianVel oc, double delta_t,
                                         std::vector<reach_lib::Point> joints) {
  this->intersect_ = {};
  reach_lib::PedestrianVel c_oc;

  // If the queue is already full remove the oldest entry
  if (this->p_vel_.size() >= this->backlog_) {
    this->p_vel_.erase(this->p_vel_.begin());
  }
  std::pair<reach_lib::PedestrianVel, double> p{oc, delta_t};
  this->p_vel_.push_back(p);
  

  // Get the occupancy corresponding to the
  // interval delta_t
  int count = this->p_vel_.size() - 1;
  while (p_vel_[count].second > this->frame_duration_ * (this->p_vel_.size() - 1 - count) && count > 0) {
    count--;
  }
  this->val_counter_[4] = count;

  if (count > this->p_vel_.size()) {
    c_oc = this->p_vel_[this->p_vel_.size() - 1].first;
  } else {
    c_oc = this->p_vel_[count].first;
  }

  // If the previous occupancies are correct,
  // all joints should intersect with the occupancy
  for (int i = 0; i < joints.size(); i++) {
    for (auto& it : c_oc.get_occupancy()) {
      if (it.get_occupancy().intersection({joints[i]})) {
        this->intersect_.push_back(true);
        break;
      }
    }
    if (this->intersect_.size() < i+1) {
      this->intersect_.push_back(false);
    }
  }

  for (int i = 0; i < this->intersect_.size(); i++) {
    if (this->intersect_[i] == false) {
      std::cout << "Attention joint " << i <<
      " not contained in occupancy at t-" << count << "!\n";
    }
  }
  return this->intersect_;
}

std::vector<bool> Validation::capsule_set_intersection(std::vector<reach_lib::Capsule> set1,
                                                       std::vector<reach_lib::Capsule> set2) {
  std::vector<bool> intersections = {};
  for (int i = 0; i < set1.size(); i++) {
    intersections.push_back(false);
    for (int j = 0; j < set2.size(); j++) {
      if (reach_lib::intersections::capsule_capsule_intersection(set1[i], set2[j]) == true) {
        intersections[i] = true;
        break;
      }
    }
  }
  return intersections;
}

std::vector<bool> Validation::cylinder_capsule_set_intersction(std::vector<reach_lib::Cylinder> sety,
                                                               std::vector<reach_lib::Capsule> seta) {
  std::vector<bool> intersections = {};
  for (int i = 0; i < sety.size(); i++) {
    intersections.push_back(false);
    for (int j = 0; j < seta.size(); j++) {
      if (reach_lib::intersections::cylinder_capsule_intersection(sety[i], seta[j]) == true) {
        intersections[i] = true;
        break;
      }
    }
  }
  return intersections;
}

}  // namespace validation
