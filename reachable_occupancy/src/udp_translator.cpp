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
#include <ros/ros.h>
#include <vector>

#include "geometry_msgs/Point.h"
#include "point.hpp"
#include "reach_lib.hpp"
#include "reachable_occupancy/JointLocations.h"
#include "reachable_occupancy/ReachableOccupancy.h"
#include "reachable_occupancy/Capsule.h"
#include "reachable_occupancy/CapsuleArray.h"

#include "udp_com/UdpPacket.h"
#include "udp_com/UdpSend.h"
#include "udp_com/UdpSocket.h"

#include "udp_transiever.cpp"

typedef udp_transiever::UDPTransiever UDPTransiever;

//! \brief Translate capsule type to capsule msg type
reachable_occupancy::CapsuleArray cap_to_msg(std::vector<reach_lib::Capsule> ca) {
  reachable_occupancy::CapsuleArray cma;
  for (int i = 0; i < ca.size(); i++) {
    reachable_occupancy::Capsule c;

    geometry_msgs::Point p1;
    p1.x = ca[i].p1_.x;
    p1.y = ca[i].p1_.y;
    p1.z = ca[i].p1_.z;

    geometry_msgs::Point p2;
    p2.x = ca[i].p2_.x;
    p2.y = ca[i].p2_.y;
    p2.z = ca[i].p2_.z;

    c.p1 = p1;
    c.p2 = p2;
    c.r = ca[i].r_;

    cma.capsules.push_back(c);
  }
  return cma;
}

//! \brief Translate reach_lib::Point to geometry_msgs::Point
std::vector<geometry_msgs::Point> pos_to_msg (std::vector<reach_lib::Point> pa) {
  std::vector<geometry_msgs::Point> pma;
  for (int i = 0; i < pa.size(); i++) {
    geometry_msgs::Point p;
    p.x = pa[i].x;
    p.y = pa[i].y;
    p.z = pa[i].z;
    pma.push_back(p);
  }
  return pma;
}

// This file is sued to build an executable ROS node that
// translate data recieved using the udp_co package to messages types
// defined in reachable_occupancy
int main(int argc, char** argv) {
  // ros system
  ros::init(argc, argv, "udp_translator");
  ros::NodeHandle nh("eno2");
  ros::Rate loop_rate(60);

  UDPTransiever tsv = UDPTransiever(8, 1.0/1000.0, "192.168.7.13", "192.168.7.1", 8080);
  UDPTransiever tsv_rob = UDPTransiever(11, 1.0/1.0, "192.168.7.13", "192.168.7.1", 8079);
  ros::Subscriber frame_data_subscriber = nh.subscribe("udp/p8080", 1000,
                                          &udp_transiever::UDPTransiever::udp_callback, &tsv);

  ros::Subscriber robot_subscriber = nh.subscribe("udp/p8079", 1000,
                                     &udp_transiever::UDPTransiever::robot_callback, &tsv_rob);

  ros::Publisher pub_rob = nh.advertise<reachable_occupancy::ReachableOccupancy>("/SaRA_demo_robot_capsules", 1000);
  ros::Publisher pub_human = nh.advertise<reachable_occupancy::JointLocations>("/SaRA_demo_human_capsules", 1000);

  while (ros::ok()) {
    ros::spinOnce();

    reachable_occupancy::ReachableOccupancy rob_capsules;
    reachable_occupancy::JointLocations human_pos;

    rob_capsules.capsules = cap_to_msg(tsv_rob.get_robot_capsules());
    
    human_pos.positions = pos_to_msg(tsv.get_joint_pos());
    human_pos.velocities = pos_to_msg(tsv.get_joint_vel());
    human_pos.delta_t = tsv_rob.get_t_brake();

    pub_rob.publish(rob_capsules);
    pub_human.publish(human_pos);

    loop_rate.sleep();
  }
  return 0;
}