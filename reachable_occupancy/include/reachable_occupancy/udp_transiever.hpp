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

#include "reachable_occupancy/JointLocations.h"

#include "capsule.hpp"
#include "point.hpp"

#include "udp_com/UdpPacket.h"
#include "udp_com/UdpSend.h"
#include "udp_com/UdpSocket.h"

#ifndef REACHABLE_OCCUPANCY_UDP_TRANSIEVER_HPP_
#define REACHABLE_OCCUPANCY_UDP_TRANSIEVER_HPP_

namespace udp_transiever {

//! \brief A shortcut to the Point type
typedef point::Point Point;


//! \brief A shortcut to the Capsule type
typedef occupancy_containers::capsule::Capsule Capsule;

//! \brief This class handles udp reception of joint locations
//! by the means of the udp_com ros packet. The received locations
//! can subsequently be published in form of a custom JointLocations message.
class UDPTransiever {
 public:
  //! \brief Empty constructor
  UDPTransiever() {}

  //! \brief Instatiates an Object of type UDPTransiever.
  //! \param[in] servIP IP-Address of this device on the network used to receive udp packets
  //! (the address of the device that the udp packets are sent to)
  //! \param[in] port The udp port on which this device should be listening
  //! \param[in] pubTopic The topic onto which joint positions should be pubished
  UDPTransiever(int joint_num, double scale = 1.0/1000.0, std::string servIP = "192.168.13.1",
                std::string clientIP_ = "192.168.1.1", int port = 8080,
                std::string connection_interface = "eno2",
                std::string pubTopic = "reachable_occupancy/joint_locations",
                int robot_joint_num = 0);

  //! \brief Empty destructor
  ~UDPTransiever() {}

  //! \brief Returns the address of the device that the udp packets are sent to
  inline std::string get_servIP() {
    return this->servIP_;
  }

  //! \brief Sets the address of the device that the udp packets are sent to
  inline void set_serv_ip(std::string serv_ip) {
    this->servIP_ = serv_ip;
  }

  //! \brief Returns the address of the device that the udp packets are sent to
  inline std::string get_client_ip() {
    return this->clientIP_;
  }

  //! \brief Sets the address of the device that the udp packets are sent to
  inline void set_client_ip(std::string client_ip) {
    this->clientIP_ = client_ip;
  }

  //! \brief Returns the udp port on which this device is listening
  inline int get_port() {
    return this->port_;
  }

  //! \brief Sets the udp port on which this device is listening
  inline void set_port(int port) {
    this->port_ = port;
  }

  //! \brief Returns the topic onto which joint positions should be pubished
  inline std::string get_pub_topic() {
    return this->pubTopic_;
  }

  //! \brief Sets the topic onto which joint positions should be pubished
  inline std::string set_pub_topic(std::string pub_topic) {
    this->pubTopic_ = pub_topic;
  }

  //! \brief Returns the factor with which joint locations are scaled
  inline double get_scale() {
    return this->scale_;
  }

  //! \brief Sets the factor with which joint locations are scaled
  inline void set_scale(double scale) {
    this->scale_ = scale;
  }

  //! \brief Returns the current time until stand-still of the robot
  inline double get_t_brake() {
    return this->t_brake_;
  }

  //! \brief Returns the number of joints received within a UDP package
  inline double get_joint_num() {
    return this->joint_num_;
  }

  //! \brief Sets the number of joints received within a UDP package
  inline void set_joint_num(double joint_num) {
    this->joint_num_ = joint_num;
  }

  //! \brief Returns the joint location message containing positions and velocities
  inline reachable_occupancy::JointLocations get_jlMSG() {
    return this->jlMSG_;
  }

  //! \brief Returns the current joint psotions of type Point
  inline std::vector<Point> get_joint_pos() {
    return this->joint_pos_;
  }

  //! \brief Returns the the current joint velocities of type Point
  inline std::vector<Point> get_joint_vel() {
    return this->joint_vel_;
  }

  //! \brief Returns the current crobot capsules
  inline std::vector<Capsule> get_robot_capsules() {
    return this->robot_capsules_;
  }

  //! \brief Returns the full_stop_ parameter
  inline bool get_full_stop() {
    return this->fs_;
  }

  //! \brief The ROS callback that receives the unpacked udp_data
  //! and converts the byte array into joint positions.
  //! \param[in] data The received UDP message converted from UDP packets
  void udp_callback(udp_com::UdpPacket data);

  //! \brief The ROS callback that receives unpacked udp_data
  //! and converts the byte array into robot capsules and t_brake.
  //! \param[in] data The received UDP message converted from UDP packets
  void robot_callback(udp_com::UdpPacket data);

  //! \brief Creates a socket that can listen and send UDP packets
  //! \param[in] nh The nodehandle used to call the socket creation service
  //! \param[in] srcAddr The address of this machine
  //! \param[in] destAddr The address of the recipient (not required)
  //! \param[in] port The UDP port used for communication
  //! \param[in] isMulticast Defines if the socket is multicast (not required)
  static bool create_socket(ros::NodeHandle nh, std::string srcAddr = "192.168.13.1",
                   std::string destAddr = "192.168.1.1", uint16_t port = 8080,
                  bool isMulicast = false);
  
  

 private:
  //! \brief IP-Address of this device on the network used to receive udp packets
  std::string servIP_ = "192.168.1.1";

  //! \brief IP-Address of the device on the network used to transmit udp packets
  std::string clientIP_ = "192.168.1.2";

  //! \brief The udp port on which this device should be listening
  int port_ = 8080;

  //! \brief The number of joints beeing tracked and transmitted
  int joint_num_ = 0;

  //! \brief The number of joints of the robot
  int robot_joint_num_ = 0;

  //! \brief The topic onto which joint positions should be pubished
  std::string pubTopic_ = "reachable_occupancy/jointLocations";

  //! \brief The time at which the previous udp package arrived
  double t_prev_ = 0.0;

  //! \brief The brakeing time of the robot dependent on its velocity
  double t_brake_ = 0.0;

  //! \brief The factor with which positions are scaled (used for unit transcription)
  //! Default value used to transform measured mm into m units
  double scale_ = 1.0/1000.0;

  //! \brief The current joint positions to be published
  reachable_occupancy::JointLocations jlMSG_;

  //! \brief Contains a list of joint positions (Cartesian) of type Point
  //! for immediate reuse within the calling script
  std::vector<Point> joint_pos_ = {};

  //! \brief Contains the current Cartesian positions of all robot joints
  std::vector<Point> rob_pos_ = {};

  //! \brief Indicates that the robot has performed a ful stop
  bool fs_ = false;

  //! \brief Contains a list of joint velocities (Cartesian) of type Point
  //! for immediate reuse within the calling script
  std::vector<Point> joint_vel_ = {};

  //! \brief Contains the last correct location of any joint before an irregular jump
  std::vector<std::pair<Point, bool>> last_correct_ = {};

  //! \brief Contains the current robot capsules as received from the robot via UDP
  std::vector<Capsule> robot_capsules_ = {};

  //! \brief The curent time used to determine the duration between received positions
  long double time_ = 0.0;

  //! \brief The number of frames a joint was frozen for during smoothing
  std::vector<int> lag_counter {};

  //! \brief The ROS node handle used for udp related tasks
  ros::NodeHandle udp_h;

  //! \brief The publisher used to publish converted joint positions
  ros::Publisher loc_pub;

  //! \brief The subscriber used to receive unpacked udp data
  ros::Subscriber udp_sub;
};
}  // namespace udp_transiever
#endif  // 
