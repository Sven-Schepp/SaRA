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

#include "reachable_occupancy/time.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include "point.hpp"



#ifndef REACHABLE_OCCUPANCY_RECEIVER_HPP_
#define REACHABLE_OCCUPANCY_RECEIVER_HPP_

namespace receiver {

//! \typedef A shortcut to the Point type
typedef point::Point Point;

//! \brief This class models callback functions for time and
//! joint information [Cartesian position and velocity (x y z)].
//! All data is received over ros-topics and converted to types
//! used by Reach-RI.
class Receiver{
 public:
  //! \brief The names of all received joints in the published order.
  std::vector<std::string> names = {};

  //! \brief The joint positions in Cartesian coordinates (x y z).
  std::vector<Point> joints = {};

  //! \brief The joint velocities in Cartesian coordinates (x y z).
  std::vector<Point> velocities = {};

  //! \brief The starting point of the reachability analysis interval.
  double t_a = 0.0;

  //! \brief The end point of the reachability analysis interval.
  double t_b = 0.0;

  //! \brief Indicates the end of data transmission.
  //! Triggered by receiving t_start = -1 or t_end = -1.
  bool stop = false;

  //! \brief Empty constructor
  Receiver() {}

  //! \brief Empty destructor
  ~Receiver() {}

  void callback(sensor_msgs::JointState joints);

  void callback_time(reachable_occupancy::time time);
};
}  // namespace receiver
#endif  //  REACHABLE_OCCUPANCY_RECEIVER_HPP_
