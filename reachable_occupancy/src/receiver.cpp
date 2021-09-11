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
#include <vector>

#include "reachable_occupancy/time.h"
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"

#include "receiver.hpp"
#include "point.hpp"

namespace receiver {

void Receiver::callback(sensor_msgs::JointState joints) {
  // Reset joint data
  this->names = joints.name;
  this->joints = {};
  this->velocities = {};

  // Initialize containers
  int x = joints.position.size();
  int i = 0;
  int j = 0;

  // The first three entries represent the joint position
  while (i <= x-3) {
    Point p;
    p.x = joints.position[i];
    p.y = joints.position[i+1];
    p.z = joints.position[i+2];
    this->joints.push_back(p);
    i += 3;
  }

  x = joints.velocity.size();
  // The second triple represents the joint velocities
  while (j <= x-3) {
    Point p;
    p.x = joints.velocity[j];
    p.y = joints.velocity[j+1];
    p.z = joints.velocity[j+2];
    this->velocities.push_back(p);
    j += 3;
  }
}

void Receiver::callback_time(reachable_occupancy::time time) {
  // Check whether the transmission has stopped
  if (time.t_a == -1 || time.t_b == -1) {
    this->stop = true;
  // Translate the joint position
  } else {
    this->t_a = time.t_a;
    this->t_b = time.t_b;
  }
}
}  // namespace receiver
