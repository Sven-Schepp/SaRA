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

#include "geometry_msgs/Point.h"
#include "point.hpp"
#include "reachable_occupancy/JointLocations.h"

#include "udp_com/UdpPacket.h"
#include "udp_com/UdpSend.h"
#include "udp_com/UdpSocket.h"

#include "udp_transiever.hpp"

namespace udp_transiever {

UDPTransiever::UDPTransiever(int joint_num, double scale,
                             std::string servIP,
                             std::string clientIP,
                             int port,
                             std::string connection_interface,
                             std::string pubTopic,
                             int robot_joint_num) :
                             servIP_(servIP), clientIP_(clientIP), scale_(scale),
                             port_(port), joint_num_(joint_num), robot_joint_num_(robot_joint_num),
                             pubTopic_(pubTopic) {
  for (int i = 0; i < joint_num; i++) {
    this->lag_counter.push_back(0);
  }
}

void UDPTransiever::udp_callback(udp_com::UdpPacket data) {
  // Receive current frame duration
  double delta_t = 0.0;
  long double now = data.header.stamp.sec + 1e-9*data.header.stamp.nsec;
  if (this->time_ == 0.0) {
    delta_t = 1.0/60.0;
    this->time_ = now;
  } else {
    delta_t = now - this->time_;
    this->time_  = now;
  }
  
  
  // LS LE LW RS RE RW H CL CL_v ST_v

  // Removing out of date joint information
  std::vector<geometry_msgs::Point> joint_positions = {};
  std::vector<geometry_msgs::Point> joint_velocities = {};
  std::vector<Point> vel_prev;
  if (this->joint_vel_.size() != 0) {
    vel_prev = this->joint_vel_;
  }
  this->joint_vel_ = {};
  std::vector<Point> pos_prev = this->joint_pos_;
  this->joint_pos_ = {};
  
  for (int i = 0; i < this->joint_num_; i++) {
    geometry_msgs::Point p;
    p.x = this->scale_ * (*reinterpret_cast<float*>(&data.data[i * 12]));
    p.y = this->scale_ * (*reinterpret_cast<float*>(&data.data[4 + i * 12]));
    p.z = this->scale_ * (*reinterpret_cast<float*>(&data.data[8 + i * 12]));
    joint_positions.push_back(p);

    this->joint_pos_.push_back(Point(p.x, p.y, p.z));
  }

  if (this->joint_pos_.size() > 0) {
    // Get virtual clav position of desired; SET HERE
    bool get_virtual_joints = true;
    if (get_virtual_joints) {
      geometry_msgs::Point clv;
      clv.x = joint_positions[3].x + 0.5*(joint_positions[0].x - joint_positions[3].x);
      clv.y = joint_positions[3].y + 0.5*(joint_positions[0].y - joint_positions[3].y);
      clv.z = joint_positions[3].z + 0.5*(joint_positions[0].z - joint_positions[3].z);
      joint_positions.push_back(clv);

      Point clv_r;
      clv_r.x = clv.x;
      clv_r.y = clv.y;
      clv_r.z = clv.z;
      this->joint_pos_.push_back(clv_r);

      // Get virtual strn position
      geometry_msgs::Point strn;
      strn.x = joint_positions[8].x + 0.7*(joint_positions[8].x - joint_positions[6].x);
      strn.y = joint_positions[8].y + 0.7*(joint_positions[8].y - joint_positions[6].y);
      strn.z = joint_positions[8].z + 1.8*(joint_positions[8].z - joint_positions[6].z);
      joint_positions.push_back(strn);

      Point strn_r;
      strn_r.x = strn.x;
      strn_r.y = strn.y;
      strn_r.z = strn.z;
      this->joint_pos_.push_back(strn_r);
    }
  }

  // There are no previous positions for velocity calculation at t = 0
  if (pos_prev.size() == 0) {
    pos_prev = this->joint_pos_;
  }

  // Update last correct position
  if (this->last_correct_.size() == 0) {
    for (int i = 0; i < this->joint_pos_.size(); i++) {
      std::pair<Point, bool> p{pos_prev[i], 0};
      this->last_correct_.push_back(p);
    }
  } else {
    for (int i = 0; i < this->joint_pos_.size(); i++) {
      if (std::get<1>(this->last_correct_[i]) == 0) {
        std::pair<Point, bool> p{pos_prev[i], 0};
        this->last_correct_[i] = p;
      }
    }
  }


  // Check wether the joint has returned to the desired region
  if (this->last_correct_.size() > 0) {
    double region_radius = 0.2;
    for (int i = 0; i < this->last_correct_.size(); i++) {
      if (this->last_correct_[i].second == 1 && Point::norm(this->joint_pos_[i], this->last_correct_[i].first) < region_radius) {
        this->last_correct_[i].first = this->joint_pos_[i];
        this->last_correct_[i].second = 0;
      }
    }
  }

  // Velocity estimation using forward difference
  for (int i = 0; i < this->joint_pos_.size(); i++) {
    this->joint_vel_.push_back((joint_pos_[i] - pos_prev[i]));
    this->joint_vel_[i].x /= delta_t;
    this->joint_vel_[i].y /= delta_t;
    this->joint_vel_[i].z /= delta_t;

    geometry_msgs::Point v;
    v.x = this->joint_vel_[i].x;
    v.y = this->joint_vel_[i].y;
    v.z = this->joint_vel_[i].z;
    joint_velocities.push_back(v);

    // Smoothing
    bool smoothing = false;
    double nv = Point::norm(this->joint_vel_[i]);
    if (nv > 3.0 && this->lag_counter[i] < 30 && vel_prev.size() != 0 && smoothing) {
      // std::cout << "Too fast: "<< i << ": " << nv << " " << this->lag_counter[i] << "\n";
      // naive
      // this->joint_pos_[i] = pos_prev[i];
      this->joint_pos_[i] = this->last_correct_[i].first;

      // prediction
      if (this->last_correct_[i].second == 0) { 
        this->last_correct_[i].second = 1;
      }

      geometry_msgs::Point temp;
      temp.x = pos_prev[i].x;
      temp.y = pos_prev[i].y;
      temp.z = pos_prev[i].z;
      joint_positions[i] = temp;
      this->lag_counter[i] += 1;
    } else {
      this->lag_counter[i] = 0;
      this->last_correct_[i].first = this->joint_pos_[i];
      this->last_correct_[i].second = 0;
    }
  }
  

  pos_prev = this->joint_pos_;

  // Check maximum joint velocities
  bool maximum_vel = true;
  for (int i = 0; i < this->joint_vel_.size(); i++) {
    double temp = Point::norm(this->joint_vel_[i]);
    if (temp > 2.0) {
      // std::cout << "Joint" << i << " too fast: " << temp << "\n";
    }
  }

  this->jlMSG_.header = data.header;
  this->jlMSG_.positions = joint_positions;
  this->jlMSG_.velocities = joint_velocities;
  this->jlMSG_.delta_t = delta_t;
}


void UDPTransiever::robot_callback(udp_com::UdpPacket data) {
  // Reset outdate capsule information
  this->robot_capsules_ = {};
  std::vector<Point> rob_pos_prev = this->rob_pos_;
  this->rob_pos_ = {};

  for (int i = 0; i < this->joint_num_; i++) {
    Point p1 = Point();
    Point p2 = Point();

    
    // Every float consist of 4 bytes; We have 7 floats per capsule 2*(x, y, z) + 1*r
    // -> A capsule has 28 bytes
    p1.x = this->scale_ * (*reinterpret_cast<float*>(&data.data[i * 28]));
    p1.y = this->scale_ * (*reinterpret_cast<float*>(&data.data[4 + i * 28]));
    p1.z = this->scale_ * (*reinterpret_cast<float*>(&data.data[8 + i * 28]));

    p2.x = this->scale_ * (*reinterpret_cast<float*>(&data.data[12 + i * 28]));
    p2.y = this->scale_ * (*reinterpret_cast<float*>(&data.data[16 + i * 28]));
    p2.z = this->scale_ * (*reinterpret_cast<float*>(&data.data[20 + i * 28]));

    double r = this->scale_ * (*reinterpret_cast<float*>(&data.data[24 + i * 28]));
    rob_pos_.push_back(p1);
    rob_pos_.push_back(p2);





    // Staggered approach
    /*
    p1.x = this->scale_ * (*reinterpret_cast<float*>(&data.data[i * 4 + 0]));
    p1.y = this->scale_ * (*reinterpret_cast<float*>(&data.data[i * 4 + 28]));
    p1.z = this->scale_ * (*reinterpret_cast<float*>(&data.data[i * 4 + 2 * 28]));

    p2.x = this->scale_ * (*reinterpret_cast<float*>(&data.data[i * 4 + 3 * 28]));
    p2.y = this->scale_ * (*reinterpret_cast<float*>(&data.data[i * 4 + 4 * 28]));
    p2.z = this->scale_ * (*reinterpret_cast<float*>(&data.data[i * 4 + 5 * 28]));

    double r = this->scale_ * (*reinterpret_cast<float*>(&data.data[i * 4 + 6 * 28]));*/

    Capsule c = Capsule(p1, p2, r);
    this->robot_capsules_.push_back(c);
  }
  this->fs_ = true;
  for (int i = 0; i < rob_pos_prev.size(); i++) {
    if (Point::norm(this->rob_pos_[i], rob_pos_prev[i]) > 0.0002) {
      this->fs_ = false;
      break;
    }
  }
  // std::cout << "Stop: " << this->full_stop_ << "\n";
  this->t_brake_ = (*reinterpret_cast<float*>(&data.data[(this->robot_capsules_.size()) * 28]));
}

bool UDPTransiever::create_socket(ros::NodeHandle nh, std::string srcAddr,
                                  std::string destAddr, uint16_t port,
                                  bool isMulicast) {
  // create socket request
  udp_com::UdpSocket socket_request;
  // Populate request service message
  socket_request.request.srcAddress = srcAddr;
  socket_request.request.destAddress = destAddr;
  socket_request.request.port = port;
  socket_request.request.isMulticast = isMulicast;
  // Create ROS based socket creation client
  ros::ServiceClient udp_socket_creation_service_client_ = nh.serviceClient<udp_com::UdpSocket>("udp/create_socket");

  // Return service call response
  if (ros::service::waitForService(udp_socket_creation_service_client_.getService(), -1) && udp_socket_creation_service_client_.call(socket_request)) {
    return true;
  }
  return false;
}

}  // namespace udp_transiever
