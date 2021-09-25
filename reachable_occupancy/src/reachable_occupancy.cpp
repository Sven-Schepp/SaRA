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

#include <chrono>
#include <map>

#include "reachable_occupancy/ReachableOccupancy.h"
#include "reach_lib.hpp"
#include "validation.cpp"
#include "visualizer.cpp"
#include "volume.cpp"
#include "udp_transiever.cpp"

#include "visualization_msgs/MarkerArray.h"

// Type definitions for shortcuts

typedef visualizer::Visualizer Visualizer;

typedef validation::Validation Validation;

typedef volume::Volume Volume;

typedef udp_transiever::UDPTransiever UDPTransiever;


//! \brief Receives robot and human data
class Receiver {
 public:
  void robot_callback(reachable_occupancy::ReachableOccupancy data){
    this->robot_capsules_ = {};
    this->t_a = data.t.t_a;
    this->t_b = data.t.t_b;
    std::vector<reach_lib::Point> rob_pos_prev = this->rob_pos_;
    this->rob_pos_ = {};
    for (int i = 0; i < data.capsules.capsules.size(); i++) {
      reach_lib::Capsule c;
      reach_lib::Point p1;
      p1.x = data.capsules.capsules[i].p1.x;
      p1.y = data.capsules.capsules[i].p1.y;
      p1.z = data.capsules.capsules[i].p1.z;
      
      reach_lib::Point p2;
      p2.x = data.capsules.capsules[i].p2.x;
      p2.y = data.capsules.capsules[i].p2.y;
      p2.z = data.capsules.capsules[i].p2.z;

      rob_pos_.push_back(p1);
      rob_pos_.push_back(p2);
      c = reach_lib::Capsule(p1,p2,data.capsules.capsules[i].r);
      this->robot_capsules_.push_back(c);
    }
    this->fs_ = true;
    for (int i = 0; i < rob_pos_prev.size(); i++) {
      if (reach_lib::Point::norm(this->rob_pos_[i], rob_pos_prev[i]) > 0.002) {
        this->fs_ = false;
        break;
      }
    }
  }

  void human_callback(reachable_occupancy::JointLocations data){
    this->joint_pos_ = {};
    this->joint_vel_ = {};
    this->t_brake = data.delta_t;
    for (int i = 0; i < data.positions.size(); i++) {
      reach_lib::Point p;
      p.x = data.positions[i].x;
      p.y = data.positions[i].y;
      p.z = data.positions[i].z;

      reach_lib::Point v;
      v.x = data.positions[i].x;
      v.y = data.positions[i].y;
      v.z = data.positions[i].z;

      this->joint_pos_.push_back(p);
      this->joint_vel_.push_back(v);
    }
  }

  std::vector<reach_lib::Point> get_joint_pos() {
    return this->joint_pos_;
  }

  std::vector<reach_lib::Point> get_joint_vel() {
    return this->joint_vel_;
  }

  double get_t_brake() {
    return this->t_brake;
  }

  std::vector<reach_lib::Capsule> get_robot_capsules() {
    return this->robot_capsules_;
  }

  double get_fs() {
    return this->fs_;
  }

  double t_a = 0.0;

  double t_b = 0.0;

 private:
  //! \brief The humans current joint position
  std::vector<reach_lib::Point> joint_pos_ = {};

  //! \brief The humans current joint velocities
  std::vector<reach_lib::Point> joint_vel_ = {};

  //! \brief The robots current braking time
  double t_brake = 0.0;

  //! \brief The robots current capsules
  std::vector<reach_lib::Capsule> robot_capsules_ = {};

  //! \brief Contains the robots current position
  std::vector<reach_lib::Point> rob_pos_ = {};

  bool fs_ = false;
};

//////////////////////////////////////// MAIN FUNCTION //////////////////////////////////////////////

int main(int argc, char** argv) {
  //// ROS system
  ros::init(argc, argv, "reachable_occupancy_main");
  ros::NodeHandle nh("ReachRI");
  ros::Rate loop_rate(60);

  //// Execution parameters
  bool use_ros_params = true;

    // UDP based parameters
  bool use_udp_transmission = false;
  bool use_robot_capsules = true;
  std::string ip_receive = "192.168.7.13";
  std::string ip_send = "192.168.7.1";
  int port_human_pos = 8080;
  int port_robot_capsules = 8079;
  int joint_num = 8;
  int rob_capsule_num = 11;
  double scale_human_pos = 1.0/1000.0;
  double scale_robot_capsules = 1.0/1.0;
    // ROS topic reception and publishing parameters
  std::string robot_topic = "/SaRA_demo_robot_capsules";  // "udp/p8079"
  std::string joint_topic = "/SaRA_demo_human_capsules";  // "udp/p8080"
  std::string vis_pub_topic = "/visualization_marker_array";
  std::string occupancy_topic = "/occupancies";
    // Timing based parameters
  bool timing_on = false;
    // Visualization parameters
  bool scenery = true;
    // Occupancy calculation parameters
  bool calc_articulated_acc = false;
  bool calc_articulated_vel = false;
  bool calc_articulated_pos = false;
  bool calc_pedestrian_acc = false;
  bool calc_pedestrian_vel = true;
    // System parameters
  double measurement_uncertainty_pos = 0.0;
  double measurement_uncertainty_vel = 0.0;
    // Articulated ACC and VEL
  std::vector<std::string> body_parts_names = {"RightUpperArm", "RightForeArm", "RightHand",
                                              "LeftUpperArm", "LeftForeArm", "LeftHand",
                                              "Torso", "Head"};
  std::vector<int> body_parts_joint_1 = {3, 4, 5, 0, 1, 2, 8, 6};
  std::vector<int> body_parts_joint_2 = {4, 5, 5, 1, 2, 2, 9, 6};
  std::vector<double> articulated_max_a = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0,
                                           30.0, 2.0, 20.0, 20.0};
  std::vector<double> articulated_max_v = {2.0, 2.0, 2.0, 2.0, 2.0,
                                           2.0, 2.0, 2.0, 2.0, 2.0};
  std::vector<double> body_part_thickness = {0.1, 0.1, 0.205, 0.1,
                                             0.1, 0.205, 0.4, 0.4};
    // Articulated POS
  std::vector<std::string> extremities_names = {"RightArm", "LeftArm"};
  std::vector<int> extremity_joints = {3, 0};
  std::vector<double> extremity_max_v = {2.6, 0.0, 0.0, 2.46};
  std::vector<double> extremity_length = {0.635, 0.0, 0.0, 0.635};
  std::vector<double> extremity_thickness = {0.205, 0.0, 0.0, 0.205};
    // Pedestrian ACC and VEL
  double pedestrian_arm_span = 1.8;
  double pedestrian_height = 3.0;
  double pedestrian_max_a = 10.0;
  double pedestrian_max_v = 2.0;
  double coordinate_offset = -1.1;
  int pedestrian_center_joint = 8;

  // Visualization parameters
  bool use_visualization = true;
  bool vis_joints = true;
  // r,g,b,a
  std::vector<double> color_articulated_acc = {0.0, 0.0, 1.0, 0.6};
  std::vector<double> color_articulated_vel = {0.0, 0.0, 1.0, 0.6};
  std::vector<double> color_articulated_pos = {0.0, 0.0, 1.0, 0.6};
  std::vector<double> color_pedestrian_acc = {0.0, 0.0, 1.0, 0.6};
  std::vector<double> color_pedestrian_vel = {0.0, 0.0, 1.0, 0.6};
  std::vector<double> color_robot_capsule = {1.0, 0.0, 1.0, 0.6};
  std::vector<double> color_joints = {1.0, 0.0, 0.0, 1.0};
  std::vector<double> color_points_in = {1.0, 0.0, 0.0, 1.0};
  std::vector<double> color_points_out = {0.0, 1.0, 1.0, 1.0};
  // Should the current capsules or those being validated be displayed
  bool vis_validated = false;

  // Validation parameters
  bool use_validation = false;
  std::vector<bool> display_grid = {false, false, false, false, false};
  double frame_duration = 1.0/60.0;
  reach_lib::Point origin_grid = reach_lib::Point(0.0, 0.0, 0.0);
  std::vector<double> grid_size = {3.0, 3.0, 3.0};
  double grid_resolution = 0.1;

  // Volume parameters
  std::vector<bool> use_volume = {false, false, false, false, false};
  int smart_origin = 8; // Index of the joint used to center the grid at every frame

  //// Reading parameters from ROS parameter server
  if(ros::param::has("/use_ros_params")){
    bool temp = false;
    ros::param::get("/use_ros_params",temp);
    use_ros_params = temp;
  }
  if (use_ros_params) {
    // UDP related
    if(ros::param::has("/use_udp_transmission")){
    bool temp = false;
    ros::param::get("/use_udp_transmission",temp);
    use_udp_transmission = temp;
    }
    if(ros::param::has("/use_robot_capsules")){
    bool temp = false;
    ros::param::get("/use_robot_capsules",temp);
    use_robot_capsules = temp;
    }
    if(ros::param::has("/ip_receive")){
    std::string temp = "";
    ros::param::get("/ip_receive",temp);
    ip_receive = temp;
    }
    if(ros::param::has("/ip_send")){
    std::string temp = "";
    ros::param::get("/ip_send",temp);
    ip_send = temp;
    }
    if(ros::param::has("/port_human_pos")){
    int temp = 0;
    ros::param::get("/port_human_pos",temp);
    port_human_pos = temp;
    }
    if(ros::param::has("/port_robot_capsules")){
    int temp = 0;
    ros::param::get("/port_robot_capsules",temp);
    port_robot_capsules = temp;
    }
    if(ros::param::has("/joint_num")){
    int temp = 0;
    ros::param::get("/joint_num",temp);
    joint_num = temp;
    }
    if(ros::param::has("/capsule_num")){
    int temp = 0;
    ros::param::get("/capsule_num",temp);
    rob_capsule_num = temp;
    }
    if(ros::param::has("/scale_human_pos")){
    int temp = 0;
    ros::param::get("/scale_human_pos",temp);
    scale_human_pos = temp;
    }
    if(ros::param::has("/scale_robot_capsules")){
    int temp = 0;
    ros::param::get("/scale_robot_capsules",temp);
    scale_robot_capsules = temp;
    }
    // ROS topic related
    if(ros::param::has("/joint_topic")){
    std::string temp = "";
    ros::param::get("/joint_topic",temp);
    joint_topic = temp;
    }
    if(ros::param::has("/robot_topic")){
    std::string temp = "";
    ros::param::get("/robot_topic",temp);
    robot_topic = temp;
    }
    // Timing related
    if(ros::param::has("/timing_on")){
    bool temp = false;
    ros::param::get("/timing_on",temp);
    timing_on = temp;
    }
    // Occupancy calculation related
    if(ros::param::has("/measurement_uncertainty_pos")){
    double temp = {};
    ros::param::get("/measurement_uncertainty_pos",temp);
    measurement_uncertainty_pos = temp;
    }
    if(ros::param::has("/measurement_uncertainty_vel")){
    double temp = {};
    ros::param::get("/measurement_uncertainty_vel",temp);
    measurement_uncertainty_vel = temp;
    }
    if(ros::param::has("/calc_articulated_acc")){
    bool temp = false;
    ros::param::get("/calc_articulated_acc",temp);
    calc_articulated_acc = temp;
    }
    if(ros::param::has("/calc_articulated_vel")){
    bool temp = false;
    ros::param::get("/calc_articulated_vel",temp);
    calc_articulated_vel = temp;
    }
    if(ros::param::has("/calc_articulated_pos")){
    bool temp = false;
    ros::param::get("/calc_articulated_pos",temp);
    calc_articulated_pos = temp;
    }
    if(ros::param::has("/calc_pedestrian_acc")){
    bool temp = false;
    ros::param::get("/calc_pedestrian_acc",temp);
    calc_pedestrian_acc = temp;
    }
    if(ros::param::has("/calc_pedestrian_vel")){
    bool temp = false;
    ros::param::get("/calc_pedestrian_vel",temp);
    calc_pedestrian_vel = temp;
    }
    if(ros::param::has("/body_parts_names")){
    std::vector<std::string> temp = {};
    ros::param::get("/body_parts_names",temp);
    body_parts_names = temp;
    }
    if(ros::param::has("/body_parts_joint_1")){
    std::vector<int> temp = {};
    ros::param::get("/body_parts_joint_1",temp);
    body_parts_joint_1 = temp;
    }
    if(ros::param::has("/body_parts_joint_2")){
    std::vector<int> temp = {};
    ros::param::get("/body_parts_joint_2",temp);
    body_parts_joint_2 = temp;
    }
    if(ros::param::has("/articulated_max_a")){
    std::vector<double> temp = {};
    ros::param::get("/articulated_max_a",temp);
    articulated_max_a = temp;
    }
    if(ros::param::has("/articulated_max_v")){
    std::vector<double> temp = {};
    ros::param::get("/articulated_max_v",temp);
    articulated_max_v = temp;
    }
    if(ros::param::has("/body_part_thickness")){
    std::vector<double> temp = {};
    ros::param::get("/body_part_thickness",temp);
    body_part_thickness = temp;
    }
    if(ros::param::has("/extremities_names")){
    std::vector<std::string> temp = {};
    ros::param::get("/extremities_names",temp);
    extremities_names = temp;
    }
    if(ros::param::has("/extremity_thickness")){
    std::vector<double> temp = {};
    ros::param::get("/extremity_thickness",temp);
    extremity_thickness = temp;
    }
    if(ros::param::has("/extremity_joints")){
    std::vector<int> temp = {};
    ros::param::get("/extremity_joints",temp);
    extremity_joints = temp;
    }
    if(ros::param::has("/extremity_length")){
    std::vector<double> temp = {};
    ros::param::get("/extremity_length",temp);
    extremity_length = temp;
    }
    if(ros::param::has("/extremity_max_v")){
    std::vector<double> temp = {};
    ros::param::get("/extremity_max_v",temp);
    extremity_max_v = temp;
    }
    if(ros::param::has("/pedestrian_arm_span")){
    double temp = {};
    ros::param::get("/pedestrian_arm_span",temp);
    pedestrian_arm_span = temp;
    }
    if(ros::param::has("/pedestrian_height")){
    double temp = {};
    ros::param::get("/pedestrian_height",temp);
    pedestrian_height = temp;
    }
    if(ros::param::has("/pedestrian_max_a")){
    double temp = {};
    ros::param::get("/pedestrian_max_a",temp);
    pedestrian_max_a = temp;
    }
    if(ros::param::has("/pedestrian_max_v")){
    double temp = {};
    ros::param::get("/pedestrian_max_v",temp);
    pedestrian_max_v = temp;
    }
    if(ros::param::has("/coordinate_offset")){
    double temp = {};
    ros::param::get("/coordinate_offset",temp);
    coordinate_offset = temp;
    }
    if(ros::param::has("/pedestrian_cener_joint")){
    int temp = {};
    ros::param::get("/pedestrian_center_joint",temp);
    pedestrian_center_joint = temp;
    }
    // Visualization related
    if(ros::param::has("/use_visualization")){
    bool temp = false;
    ros::param::get("/use_visualization",temp);
    use_visualization = temp;
    }
    if(ros::param::has("/vis_joints")){
    bool temp = false;
    ros::param::get("/vis_joints",temp);
    vis_joints = temp;
    }
    if(ros::param::has("/color_articulated_acc")){
    std::vector<double> temp = {};
    ros::param::get("/color_articulated_acc",temp);
    color_articulated_acc = temp;
    }
    if(ros::param::has("/color_articulated_vel")){
    std::vector<double> temp = {};
    ros::param::get("/color_articulated_vel",temp);
    color_articulated_vel = temp;
    }
    if(ros::param::has("/color_articulated_pos")){
    std::vector<double> temp = {};
    ros::param::get("/color_articulated_pos",temp);
    color_articulated_pos = temp;
    }
    if(ros::param::has("/color_pedestrian_acc")){
    std::vector<double> temp = {};
    ros::param::get("/color_pedestrian_acc",temp);
    color_pedestrian_acc = temp;
    }
    if(ros::param::has("/color_pedestrian_vel")){
    std::vector<double> temp = {};
    ros::param::get("/color_pedestrian_vel",temp);
    color_pedestrian_vel = temp;
    }
    if(ros::param::has("/color_joints")){
    std::vector<double> temp = {};
    ros::param::get("/color_joints",temp);
    color_joints = temp;
    }
    if(ros::param::has("/color_robot_capsule")){
    std::vector<double> temp = {};
    ros::param::get("/color_robot_capsule",temp);
    color_robot_capsule = temp;
    }
    if(ros::param::has("/color_point_in")){
    std::vector<double> temp = {};
    ros::param::get("/color_points_in",temp);
    color_points_in = temp;
    }
    if(ros::param::has("/color_point_out")){
    std::vector<double> temp = {};
    ros::param::get("/color_points_out",temp);
    color_points_out = temp;
    }
    if(ros::param::has("/vis_validated")){
    bool temp = false;
    ros::param::get("/vis_validated",temp);
    vis_validated = temp;
    }
    // Validation related
    if(ros::param::has("/use_validation")){
    bool temp = false;
    ros::param::get("/use_validation",temp);
    use_validation = temp;
    }
    if(ros::param::has("/frame_duration")){
    double temp = 0;
    ros::param::get("/frame_duration",temp);
    frame_duration = temp;
    }
    // Volume related
    if(ros::param::has("/use_volume")){
    std::vector<bool> temp = {};
    ros::param::get("/use_volume",temp);
    use_volume = temp;
    }
    if(ros::param::has("/display_grid")){
    std::vector<bool> temp = {};
    ros::param::get("/display_grid",temp);
    display_grid = temp;
    }
    if(ros::param::has("/origin_grid")){
    std::vector<double> temp = {};
    ros::param::get("/origin_grid",temp);
    origin_grid = reach_lib::Point(temp[0], temp[1], temp[2]);
    }
    if(ros::param::has("/smart_origin")){
    int temp = 0;
    ros::param::get("/smart_origin",temp);
    smart_origin = temp;
    } else {
      smart_origin = -1;
    }
    if(ros::param::has("/grid_size")){
    std::vector<double> temp = {};
    ros::param::get("/grid_size",temp);
    grid_size = temp;
    }
    if(ros::param::has("/grid_resolution")){
    double temp = {};
    ros::param::get("/grid_resolution",temp);
    grid_resolution = temp;
    }
  }

  //// UDP architecture
  UDPTransiever tsv = UDPTransiever();
  UDPTransiever tsv_rob = UDPTransiever();
  ros::Subscriber frame_data_subscriber;
  ros::Subscriber robot_subscriber;
  ros::Subscriber frame_data_subscriber_udp_recorded;
  ros::Subscriber robot_subscriber_udp_recorded;

  // Non UDP receivers
  Receiver rv;
  if (use_udp_transmission) {
    // Open port for joint positions
    tsv.set_joint_num(joint_num);
    tsv.set_scale(scale_human_pos);
    tsv.set_client_ip(ip_receive);
    tsv.set_serv_ip(ip_send);
    tsv.set_port(port_human_pos);
    if (!UDPTransiever::create_socket(nh, ip_receive, ip_send, port_human_pos, false)) {
      ROS_WARN("Socket Not Connected %d!",port_human_pos);
    }

    // Subscribe to port topic opened by ROS_UDP
    frame_data_subscriber = nh.subscribe("udp/p" + std::to_string(port_human_pos), 1000,
                                         &udp_transiever::UDPTransiever::udp_callback, &tsv);

    // Open port for robot capsules
    if (use_robot_capsules) {
      tsv_rob.set_joint_num(rob_capsule_num);
      tsv_rob.set_scale(scale_robot_capsules);
      tsv_rob.set_client_ip(ip_receive);
      tsv_rob.set_serv_ip(ip_send);
      tsv_rob.set_port(port_robot_capsules);
      if (!UDPTransiever::create_socket(nh, ip_receive, ip_send, port_robot_capsules, false)) {
        ROS_WARN("Socket Not Connected %d!", port_robot_capsules);
      }

      // Subscribe to port topic opened by ROS_UDP
      robot_subscriber = nh.subscribe("udp/p" + std::to_string(port_robot_capsules), 1000,
                                      &udp_transiever::UDPTransiever::robot_callback, &tsv_rob);
    }
  } else {
    frame_data_subscriber = nh.subscribe(joint_topic, 1000,
                                         &Receiver::human_callback, &rv);
    /*
    frame_data_subscriber_udp_recorded = nh.subscribe(joint_topic, 1000,
                                         &udp_transiever::UDPTransiever::udp_callback, &tsv);*/
    if (use_robot_capsules) {
      robot_subscriber = nh.subscribe(robot_topic, 1000,
                                       &Receiver::robot_callback, &rv);
      /*
      robot_subscriber_udp_recorded = nh.subscribe(robot_topic, 1000,
                                       &udp_transiever::UDPTransiever::robot_callback, &tsv_rob);*/              
    }
  }

  //// Publishing data
  ros::Publisher pub_vis = nh.advertise<visualization_msgs::MarkerArray>
                           ("/visualization_marker_array", 1000);
  ros::Publisher pub_a_acc = nh.advertise<reachable_occupancy::ReachableOccupancy>
                             (occupancy_topic + "/articulated_acc", 1000);
  ros::Publisher pub_a_vel = nh.advertise<reachable_occupancy::ReachableOccupancy>
                             (occupancy_topic + "/articulated_vel", 1000);
  ros::Publisher pub_a_pos = nh.advertise<reachable_occupancy::ReachableOccupancy>
                             (occupancy_topic + "/articulated_pos", 1000);
  ros::Publisher pub_p_acc = nh.advertise<reachable_occupancy::ReachableOccupancy>
                             (occupancy_topic + "/pedestrian_acc", 1000);
  ros::Publisher pub_p_vel = nh.advertise<reachable_occupancy::ReachableOccupancy>
                             (occupancy_topic + "/pedestrian_vel", 1000);


  //// Timer setup
  long double running_sum = 0.0;
  double loop_counter = 1.0;
  double average = 0.0;
  double longest = 0.0;
  int shortest = 1E9;


  //// Set up human models
  reach_lib::System system = reach_lib::System(measurement_uncertainty_pos,
                                               measurement_uncertainty_vel);
  std::map<std::string, std::pair<int, int>> index_articulated_acc;

  // ACC and VEL
  for (int i = 0; i < body_parts_names.size(); i++) {
    index_articulated_acc[body_parts_names[i]] =
    std::pair<int, int> (body_parts_joint_1[i], body_parts_joint_2[i]);
  }

  std::map<std::string, std::pair<int, int>> index_articulated_vel = index_articulated_acc;
  std::map<std::string, double> articulated_thickness;
  for (int i = 0; i < body_parts_names.size(); i++) {
    articulated_thickness[body_parts_names[i]] = body_part_thickness[i];
  }

  // POS
  std::map<std::string, std::pair<int, int>> index_articulated_pos;
  for (int i = 0; i < extremities_names.size(); i++) {
    index_articulated_pos[extremities_names[i]] =
    std::pair<int, int> (extremity_joints[i], extremity_joints[i]);
  }

  reach_lib::ArticulatedAccel articulated_acc =
  reach_lib::ArticulatedAccel(system, index_articulated_acc, articulated_thickness, articulated_max_a);
  reach_lib::ArticulatedVel articulated_vel =
  reach_lib::ArticulatedVel(system, index_articulated_vel, articulated_thickness, articulated_max_v);
  reach_lib::ArticulatedPos articulated_pos =
  reach_lib::ArticulatedPos(system, index_articulated_pos, extremity_thickness,
                            extremity_max_v, extremity_length);

  reach_lib::PedestrianAccel pedestrian_acc =
  reach_lib::PedestrianAccel(system, pedestrian_height,pedestrian_arm_span,
                             pedestrian_max_a, coordinate_offset);
  reach_lib::PedestrianVel pedestrian_vel =
  reach_lib::PedestrianVel(system, pedestrian_height,pedestrian_arm_span,
                             pedestrian_max_a, coordinate_offset);

  reach_lib::Articulated* articulated_acc_p = &articulated_acc;
  reach_lib::Articulated* articulated_vel_p = &articulated_vel;
  reach_lib::Articulated* articulated_pos_p = &articulated_pos;

  reach_lib::Pedestrian* pedestrian_acc_p = &pedestrian_acc;
  reach_lib::Pedestrian* pedestrian_vel_p = &pedestrian_vel;

  // Set up visualization
  Visualizer articulated_acc_vis = Visualizer();
  articulated_acc_vis.set_id_map(articulated_acc_p, 1);
  articulated_acc_vis.color_.r = color_articulated_acc[0];
  articulated_acc_vis.color_.g = color_articulated_acc[1];
  articulated_acc_vis.color_.b = color_articulated_acc[2];
  articulated_acc_vis.color_.a = color_articulated_acc[3];

  Visualizer articulated_vel_vis = Visualizer();
  articulated_vel_vis.set_id_map(articulated_vel_p, 2);
  articulated_vel_vis.color_.r = color_articulated_vel[0];
  articulated_vel_vis.color_.g = color_articulated_vel[1];
  articulated_vel_vis.color_.b = color_articulated_vel[2];
  articulated_vel_vis.color_.a = color_articulated_vel[3];

  Visualizer articulated_pos_vis = Visualizer();
  articulated_pos_vis.set_id_map(articulated_pos_p, 3);
  articulated_pos_vis.mode_ = "ARTICULATED-POS";
  articulated_pos_vis.color_.r = color_articulated_pos[0];
  articulated_pos_vis.color_.g = color_articulated_pos[1];
  articulated_pos_vis.color_.b = color_articulated_pos[2];
  articulated_pos_vis.color_.a = color_articulated_pos[3];

  Visualizer pedestrian_acc_vis = Visualizer();
  pedestrian_acc_vis.set_id_map(pedestrian_acc_p, 4);
  pedestrian_acc_vis.color_.r = color_pedestrian_acc[0];
  pedestrian_acc_vis.color_.g = color_pedestrian_acc[1];
  pedestrian_acc_vis.color_.b = color_pedestrian_acc[2];
  pedestrian_acc_vis.color_.a = color_pedestrian_acc[3];

  Visualizer pedestrian_vel_vis = Visualizer();
  pedestrian_vel_vis.set_id_map(pedestrian_vel_p, 5);
  pedestrian_vel_vis.color_.r = color_pedestrian_vel[0];
  pedestrian_vel_vis.color_.g = color_pedestrian_vel[1];
  pedestrian_vel_vis.color_.b = color_pedestrian_vel[2];
  pedestrian_vel_vis.color_.a = color_pedestrian_vel[3];

  Visualizer robot_capsule_vis = Visualizer();
  robot_capsule_vis.set_id_map(7, 6);
  robot_capsule_vis.color_.r = color_robot_capsule[0];
  robot_capsule_vis.color_.g = color_robot_capsule[1];
  robot_capsule_vis.color_.b = color_robot_capsule[2];
  robot_capsule_vis.color_.a = color_robot_capsule[3];

  Visualizer joint_vis = Visualizer();
  joint_vis.color_.r = color_joints[0];
  joint_vis.color_.g = color_joints[1];
  joint_vis.color_.b = color_joints[2];
  joint_vis.color_.a = color_joints[3];

  Visualizer points_in_vis = Visualizer();
  points_in_vis.color_.r = color_points_in[0];
  points_in_vis.color_.g = color_points_in[1];
  points_in_vis.color_.b = color_points_in[2];
  points_in_vis.color_.a = color_points_in[3];

  Visualizer points_out_vis = Visualizer();
  points_out_vis.color_.r = color_points_out[0];
  points_out_vis.color_.g = color_points_out[1];
  points_out_vis.color_.b = color_points_out[2];
  points_out_vis.color_.a = color_points_out[3];

  // Set up validation
  Validation articulated_acc_val = Validation(0.2, frame_duration);
  Validation articulated_vel_val = Validation(0.2, frame_duration);
  Validation articulated_pos_val = Validation(0.2, frame_duration);
  Validation pedestrian_acc_val = Validation(0.2, frame_duration);
  Validation pedestrian_vel_val = Validation(0.2, frame_duration);

  // Set up volume
  reach_lib::Point origin_volume = origin_grid;
  Volume articulated_acc_vol = Volume(grid_size[0], grid_size[1], grid_size[2], grid_resolution, origin_volume);
  Volume articulated_vel_vol = Volume(grid_size[0], grid_size[1], grid_size[2], grid_resolution, origin_volume);
  Volume articulated_pos_vol = Volume(grid_size[0], grid_size[1], grid_size[2], grid_resolution, origin_volume);
  Volume pedestrian_acc_vol = Volume(grid_size[0], grid_size[1], grid_size[2], grid_resolution, origin_volume);
  Volume pedestrian_vel_vol = Volume(grid_size[0], grid_size[1], grid_size[2], grid_resolution, origin_volume);

  // Define delta_t loop variables
  double t_a = 0.0;
  double t_b = 0.0;

  while(ros::ok()) {
    // Start cycle clock
    auto t1 = std::chrono::high_resolution_clock::now();
    // Empty the visualization array
    visualization_msgs::MarkerArray marr = {};

    // Get current breaking time of the robot
    if (tsv_rob.get_t_brake() > 0.0 && use_udp_transmission) {
      t_b = tsv_rob.get_t_brake();
    }
    if (!use_udp_transmission) {
      t_a = rv.t_a;
      t_b = rv.get_t_brake();
    }

    // Update chosen models
    if (use_udp_transmission) {
      if (tsv.get_joint_pos().size() > 0) {
        if (calc_articulated_acc) {
          articulated_acc.update(t_a, t_b, tsv.get_joint_pos(), tsv.get_joint_vel());
        }
        if (calc_articulated_vel) {
          articulated_vel.update(t_a, t_b, tsv.get_joint_pos());
        }
        if (calc_articulated_pos) {
          std::vector<reach_lib::Point> temp = {};
          for (const auto& it : extremity_joints) {
            temp.push_back(tsv.get_joint_pos()[it]);
          }
          articulated_pos.update(t_a, t_b, temp);
        }
        if (calc_pedestrian_acc) {
          pedestrian_acc.update(t_a, t_b, {tsv.get_joint_pos()[pedestrian_center_joint]},
                                {tsv.get_joint_vel()[pedestrian_center_joint]});
        }
        if (calc_pedestrian_vel) {
          pedestrian_vel.update(t_a, t_b, {tsv.get_joint_pos()[pedestrian_center_joint]});
        }
      }
    } else {
      if (rv.get_joint_pos().size() > 0) {
        if (calc_articulated_acc) {
          articulated_acc.update(t_a, t_b, rv.get_joint_pos(), rv.get_joint_vel());
        }
        if (calc_articulated_vel) {
          articulated_vel.update(t_a, t_b, rv.get_joint_pos());
        }
        if (calc_articulated_pos) {
          std::vector<reach_lib::Point> temp = {};
          for (const auto& it : extremity_joints) {
            temp.push_back(rv.get_joint_pos()[it]);
          }
          articulated_pos.update(t_a, t_b, rv.get_joint_pos());
        }
        if (calc_pedestrian_acc) {
          pedestrian_acc.update(t_a, t_b, {rv.get_joint_pos()[pedestrian_center_joint]},
                                {rv.get_joint_vel()[pedestrian_center_joint]});
        }
        if (calc_pedestrian_vel) {
          pedestrian_vel.update(t_a, t_b, {rv.get_joint_pos()[pedestrian_center_joint]});
        }
      }
    }

    // Validate occupancies / determine intersections with robot capsules and visualize them
    if (use_validation && tsv.get_joint_pos().size() > 0) {
      // Check for intersections with robot capsules
      std::vector<bool> articulated_acc_hr_intersect = articulated_acc_val.capsule_set_intersection(
      reach_lib::get_capsules(articulated_acc), tsv_rob.get_robot_capsules());
      std::vector<bool> articulated_vel_hr_intersect = articulated_vel_val.capsule_set_intersection(
      reach_lib::get_capsules(articulated_vel), tsv_rob.get_robot_capsules());
      std::vector<bool> articulated_pos_hr_intersect = articulated_pos_val.capsule_set_intersection(
      reach_lib::get_capsules(articulated_pos), tsv_rob.get_robot_capsules());
      std::vector<bool> pedestrian_acc_hr_intersect = pedestrian_acc_val.cylinder_capsule_set_intersction(
      reach_lib::get_cylinders(pedestrian_acc), tsv_rob.get_robot_capsules());
      std::vector<bool> pedestrian_vel_hr_intersect = pedestrian_vel_val.cylinder_capsule_set_intersction(
      reach_lib::get_cylinders(pedestrian_vel), tsv_rob.get_robot_capsules());

      articulated_acc_val.validation(articulated_acc, t_b, tsv.get_joint_pos());
      articulated_vel_val.validation(articulated_vel, t_b, tsv.get_joint_pos());
      articulated_pos_val.validation(articulated_pos, t_b, tsv.get_joint_pos());
      pedestrian_acc_val.validation(pedestrian_acc, t_b, tsv.get_joint_pos());
      pedestrian_vel_val.validation(pedestrian_vel, t_b, tsv.get_joint_pos());

      if (articulated_acc_val.get_a_accel().size() >= articulated_acc_val.get_backlog()) {
        std::vector<reach_lib::Capsule> caps = {};
        for (auto& it : articulated_acc_val.get_a_pos()
            [articulated_acc_val.get_val_counter()[2]].first.get_occupancy()) {
          caps.push_back(it.get_occupancy());
        }
        if (vis_validated) {
          articulated_acc_vis.vis_capsules(caps, articulated_acc_hr_intersect);
        }
      }
      if (articulated_vel_val.get_a_vel().size() >= articulated_vel_val.get_backlog()) {
        std::vector<reach_lib::Capsule> caps = {};
        for (auto& it : articulated_vel_val.get_a_pos()
            [articulated_vel_val.get_val_counter()[2]].first.get_occupancy()) {
          caps.push_back(it.get_occupancy());
        }
        if (vis_validated) {
          articulated_vel_vis.vis_capsules(caps, articulated_vel_hr_intersect);
        }
      }
      if (articulated_pos_val.get_a_pos().size() >= articulated_pos_val.get_backlog()) {
        std::vector<reach_lib::Capsule> caps = {};
        for (auto& it : articulated_pos_val.get_a_pos()
            [articulated_pos_val.get_val_counter()[2]].first.get_occupancy()) {
          caps.push_back(it.get_occupancy());
        }
        if (vis_validated) {
          articulated_pos_vis.vis_capsules(caps, articulated_pos_hr_intersect);
        }
      }
      if (pedestrian_acc_val.get_p_accel().size() >= pedestrian_acc_val.get_backlog()) {
        reach_lib::PedestrianAccel* p_a = &pedestrian_acc_val.get_p_accel()
        [pedestrian_acc_val.get_val_counter()[3]].first;
        if (vis_validated) {
          articulated_acc_vis.vis_pedestrian(p_a, pedestrian_acc_hr_intersect);
        }
      }
      if (pedestrian_vel_val.get_p_vel().size() >= pedestrian_vel_val.get_backlog()) {
        reach_lib::PedestrianVel* p_v = &pedestrian_vel_val.get_p_vel()
        [pedestrian_vel_val.get_val_counter()[3]].first;
        if (vis_validated) {
          articulated_vel_vis.vis_pedestrian(p_v, pedestrian_vel_hr_intersect);
        }
      }
    }
    if (use_validation && rv.get_joint_pos().size() > 0 && !use_udp_transmission) {
      // Check for intersections with robot capsules
      std::vector<bool> articulated_acc_hr_intersect = articulated_acc_val.capsule_set_intersection(
      reach_lib::get_capsules(articulated_acc), rv.get_robot_capsules());
      std::vector<bool> articulated_vel_hr_intersect = articulated_vel_val.capsule_set_intersection(
      reach_lib::get_capsules(articulated_vel), rv.get_robot_capsules());
      std::vector<bool> articulated_pos_hr_intersect = articulated_pos_val.capsule_set_intersection(
      reach_lib::get_capsules(articulated_pos), rv.get_robot_capsules());
      std::vector<bool> pedestrian_acc_hr_intersect = pedestrian_acc_val.cylinder_capsule_set_intersction(
      reach_lib::get_cylinders(pedestrian_acc), rv.get_robot_capsules());
      std::vector<bool> pedestrian_vel_hr_intersect = pedestrian_vel_val.cylinder_capsule_set_intersction(
      reach_lib::get_cylinders(pedestrian_vel), rv.get_robot_capsules());

      articulated_acc_val.validation(articulated_acc, t_b, rv.get_joint_pos());
      articulated_vel_val.validation(articulated_vel, t_b, rv.get_joint_pos());
      articulated_pos_val.validation(articulated_pos, t_b, rv.get_joint_pos());
      pedestrian_acc_val.validation(pedestrian_acc, t_b, rv.get_joint_pos());
      pedestrian_vel_val.validation(pedestrian_vel, t_b, rv.get_joint_pos());

      if (articulated_acc_val.get_a_accel().size() >= articulated_acc_val.get_backlog()) {
        std::vector<reach_lib::Capsule> caps = {};
        for (auto& it : articulated_acc_val.get_a_pos()
            [articulated_acc_val.get_val_counter()[2]].first.get_occupancy()) {
          caps.push_back(it.get_occupancy());
        }
        if (vis_validated) {
          articulated_acc_vis.vis_capsules(caps, articulated_acc_hr_intersect);
        }
      }
      if (articulated_vel_val.get_a_vel().size() >= articulated_vel_val.get_backlog()) {
        std::vector<reach_lib::Capsule> caps = {};
        for (auto& it : articulated_vel_val.get_a_pos()
            [articulated_vel_val.get_val_counter()[2]].first.get_occupancy()) {
          caps.push_back(it.get_occupancy());
        }
        if (vis_validated) {
          articulated_vel_vis.vis_capsules(caps, articulated_vel_hr_intersect);
        }
      }
      if (articulated_pos_val.get_a_pos().size() >= articulated_pos_val.get_backlog()) {
        std::vector<reach_lib::Capsule> caps = {};
        for (auto& it : articulated_pos_val.get_a_pos()
            [articulated_pos_val.get_val_counter()[2]].first.get_occupancy()) {
          caps.push_back(it.get_occupancy());
        }
        if (vis_validated) {
          articulated_pos_vis.vis_capsules(caps, articulated_pos_hr_intersect);
        }
      }
      if (pedestrian_acc_val.get_p_accel().size() >= pedestrian_acc_val.get_backlog()) {
        reach_lib::PedestrianAccel* p_a = &pedestrian_acc_val.get_p_accel()
        [pedestrian_acc_val.get_val_counter()[3]].first;
        if (vis_validated) {
          articulated_acc_vis.vis_pedestrian(p_a, pedestrian_acc_hr_intersect);
        }
      }
      if (pedestrian_vel_val.get_p_vel().size() >= pedestrian_vel_val.get_backlog()) {
        reach_lib::PedestrianVel* p_v = &pedestrian_vel_val.get_p_vel()
        [pedestrian_vel_val.get_val_counter()[3]].first;
        if (vis_validated) {
          articulated_vel_vis.vis_pedestrian(p_v, pedestrian_vel_hr_intersect);
        }
      }
    }

    // Calculate volume
    if (rv.get_joint_pos().size() > 0 && !use_udp_transmission) {
      if (use_volume[0]) {
        if (smart_origin != -1 && smart_origin < tsv.get_joint_pos().size()) {
          articulated_acc_vol.set_origin(tsv.get_joint_pos()[smart_origin]);
        }
        articulated_acc_vol.volume_routine(articulated_acc);
      }
      if (use_volume[1]) {
        if (smart_origin != -1 && smart_origin < tsv.get_joint_pos().size()) {
          articulated_vel_vol.set_origin(tsv.get_joint_pos()[smart_origin]);
        }
        articulated_vel_vol.volume_routine(articulated_vel);
      }
      if (use_volume[2]) {
        if (smart_origin != -1 && smart_origin < tsv.get_joint_pos().size()) {
          articulated_pos_vol.set_origin(tsv.get_joint_pos()[smart_origin]);
        }
        articulated_pos_vol.volume_routine(articulated_pos);
      }
      if (use_volume[3]) {
        if (smart_origin != -1 && smart_origin < tsv.get_joint_pos().size()) {
          pedestrian_acc_vol.set_origin(tsv.get_joint_pos()[smart_origin]);
        }
        pedestrian_acc_vol.volume_routine(pedestrian_acc);
      }
      if (use_volume[4]) {
        if (smart_origin != -1 && smart_origin < tsv.get_joint_pos().size()) {
          pedestrian_vel_vol.set_origin(tsv.get_joint_pos()[smart_origin]);
        }
        pedestrian_vel_vol.volume_routine(pedestrian_vel);
      }
    } else {
      if (use_volume[0]) {
        if (smart_origin != -1 && smart_origin < rv.get_joint_pos().size()) {
          articulated_acc_vol.set_origin(rv.get_joint_pos()[smart_origin]);
        }
        articulated_acc_vol.volume_routine(articulated_acc);
      }
      if (use_volume[1]) {
        if (smart_origin != -1 && smart_origin < rv.get_joint_pos().size()) {
          articulated_vel_vol.set_origin(rv.get_joint_pos()[smart_origin]);
        }
        articulated_vel_vol.volume_routine(articulated_vel);
      }
      if (use_volume[2]) {
        if (smart_origin != -1 && smart_origin < rv.get_joint_pos().size()) {
          articulated_pos_vol.set_origin(rv.get_joint_pos()[smart_origin]);
        }
        articulated_pos_vol.volume_routine(articulated_pos);
      }
      if (use_volume[3]) {
        if (smart_origin != -1 && smart_origin < rv.get_joint_pos().size()) {
          pedestrian_acc_vol.set_origin(rv.get_joint_pos()[smart_origin]);
        }
        pedestrian_acc_vol.volume_routine(pedestrian_acc);
      }
      if (use_volume[4]) {
        if (smart_origin != -1 && smart_origin < rv.get_joint_pos().size()) {
          pedestrian_vel_vol.set_origin(rv.get_joint_pos()[smart_origin]);
        }
        pedestrian_vel_vol.volume_routine(pedestrian_vel);
      }
    }

    // Visualizing joints and in/out points clouds
    if (use_visualization) {
      if (vis_joints && tsv.get_joint_pos().size() > 0 && use_udp_transmission) {
        joint_vis.vis_point_cloud(tsv.get_joint_pos(), "joint_positions");
      }
      if (vis_joints && rv.get_joint_pos().size() > 0 && !use_udp_transmission) {
        joint_vis.vis_point_cloud(rv.get_joint_pos(), "joint_positions");
      }
      if (display_grid[0]) {
        points_in_vis.vis_point_cloud(articulated_acc_vol.points_in, "points_in_art_acc");
        points_in_vis.vis_point_cloud(articulated_acc_vol.points_out, "points_out_art_acc");
      }
      if (display_grid[1]) {
        points_in_vis.vis_point_cloud(articulated_vel_vol.points_in, "points_in_art_vel");
        points_in_vis.vis_point_cloud(articulated_vel_vol.points_out, "points_out_art_vel");
      }
      if (display_grid[2]) {
        points_in_vis.vis_point_cloud(articulated_pos_vol.points_in, "points_in_art_pos");
        points_in_vis.vis_point_cloud(articulated_pos_vol.points_out, "points_out_art_pos");
      }
      if (display_grid[3]) {
        points_in_vis.vis_point_cloud(pedestrian_acc_vol.points_in, "points_in_ped_acc");
        points_in_vis.vis_point_cloud(pedestrian_acc_vol.points_out, "points_out_ped_acc");
      }
      if (display_grid[4]) {
        points_in_vis.vis_point_cloud(pedestrian_vel_vol.points_in, "points_in_ped_vel");
        points_in_vis.vis_point_cloud(pedestrian_vel_vol.points_out, "points_out_ped_vel");
      }
    }

    
    // Visualize occupancies
    if (!vis_validated && use_visualization) {
      if (calc_articulated_acc) {
        articulated_acc_vis.vis_articulated(articulated_acc_p);
      }
      if (calc_articulated_vel) {
        articulated_vel_vis.vis_articulated(articulated_vel_p);
      }
      if (calc_articulated_pos) {
        articulated_pos_vis.vis_articulated(articulated_pos_p);
      }
      if (calc_pedestrian_acc) {
        pedestrian_acc_vis.vis_pedestrian(pedestrian_acc_p);
      }
      if (calc_pedestrian_vel) {
        pedestrian_vel_vis.vis_pedestrian(pedestrian_vel_p);
      }
      // Robot occupancies
      if (tsv_rob.get_robot_capsules().size() > 0 && use_robot_capsules && use_udp_transmission) {
        robot_capsule_vis.vis_capsules(tsv_rob.get_robot_capsules());
      }
      if (rv.get_robot_capsules().size() > 0 && use_robot_capsules && !use_udp_transmission) {
        robot_capsule_vis.vis_capsules(rv.get_robot_capsules());
      }
    }

    // Build a visualized scenery to embed the occupancies in
    if (scenery) {
      visualization_msgs::Marker room = visualization_msgs::Marker();

      room.header.frame_id = "/map";
      room.header.stamp = ros::Time::now();
      room.ns = "room";
      room.action = visualization_msgs::Marker::ADD;
      room.type = visualization_msgs::Marker::MESH_RESOURCE;
      room.lifetime = ros::Duration(2);
      room.id = 10000000;

      room.mesh_resource = "package://reachable_occupancy/rviz/meshes/room.dae";
      room.color.a = 1.0;
      room.color.r = 220.0/255.0;
      room.color.g = 220.0/255.0;
      room.color.b = 220.0/255.0;

      room.scale.x = 0.4;
      room.scale.y = 0.4;
      room.scale.z = 0.2;

      room.pose.position.x = 5.0;
      room.pose.position.y = 0.0;
      room.pose.position.z = -1.1;


      visualization_msgs::Marker door = visualization_msgs::Marker();

      door.header.frame_id = "/map";
      door.header.stamp = ros::Time::now();
      door.ns = "door";
      door.action = visualization_msgs::Marker::ADD;
      door.type = visualization_msgs::Marker::MESH_RESOURCE;
      door.lifetime = ros::Duration(2);
      door.id = 10000001;

      door.mesh_resource = "package://reachable_occupancy/rviz/meshes/door.stl";
      door.color.a = 1.0;
      door.color.r = 1.0;
      door.color.g = 1.0;
      door.color.b = 1.0;

      door.scale.x = 0.01;
      door.scale.y = 0.01;
      door.scale.z = 0.01;

      door.pose.position.x = -2.6;
      door.pose.position.y = 0.63;
      door.pose.position.z = -1.1;

      door.pose.orientation.x = -0.5;
      door.pose.orientation.y = 0.5;
      door.pose.orientation.z = -0.5;
      door.pose.orientation.w = 0.5;


      visualization_msgs::Marker box = visualization_msgs::Marker();

      box.header.frame_id = "/map";
      box.header.stamp = ros::Time::now();
      box.ns = "box";
      box.action = visualization_msgs::Marker::ADD;
      box.type = visualization_msgs::Marker::CUBE;
      box.lifetime = ros::Duration(2);
      box.id = 10000002;

      box.color.a = 1.0;
      box.color.r = 0.0;
      box.color.g = 0.0;
      box.color.b = 0.0;

      box.scale.x = 0.2;
      box.scale.y = 0.2;
      box.scale.z = 0.16;

      box.pose.position.x = 0.7;
      box.pose.position.y = -1.3;
      box.pose.position.z = -0.4;


      visualization_msgs::Marker rob_table = visualization_msgs::Marker();

      rob_table.header.frame_id = "/map";
      rob_table.header.stamp = ros::Time::now();
      rob_table.ns = "robot table";
      rob_table.action = visualization_msgs::Marker::ADD;
      rob_table.type = visualization_msgs::Marker::MESH_RESOURCE;
      rob_table.lifetime = ros::Duration(2);
      rob_table.id = 10000003;

      rob_table.mesh_resource = "package://reachable_occupancy/rviz/meshes/cofee_table.stl";
      rob_table.color.a = 1.0;
      rob_table.color.r = 169.0/255.0;
      rob_table.color.g = 169.0/255.0;
      rob_table.color.b = 169.0/255.0;

      rob_table.scale.x = 2.4;
      rob_table.scale.y = 1.8;
      rob_table.scale.z = 2.1;

      rob_table.pose.position.x = -0.45;
      rob_table.pose.position.y = 0.63;
      rob_table.pose.position.z = -1.1;

      rob_table.pose.orientation.z = -0.7071081;
      rob_table.pose.orientation.w = -0.7071055;


      visualization_msgs::Marker table = visualization_msgs::Marker();

      table.header.frame_id = "/map";
      table.header.stamp = ros::Time::now();
      table.ns = "table";
      table.action = visualization_msgs::Marker::ADD;
      table.type = visualization_msgs::Marker::MESH_RESOURCE;
      table.lifetime = ros::Duration(2);
      table.id = 10000004;

      table.mesh_resource = "package://reachable_occupancy/rviz/meshes/table.dae";
      table.color.a = 1.0;
      table.color.r = 169.0/255.0;
      table.color.g = 169.0/255.0;
      table.color.b = 169.0/255.0;

      table.scale.x = 0.3;
      table.scale.y = 0.22;
      table.scale.z = 0.27;

      table.pose.position.x = 1.0;
      table.pose.position.y = -0.94;
      table.pose.position.z = -1.1;

      table.pose.orientation.z = -0.7071081;
      table.pose.orientation.w = -0.7071055;

      // Add all assets to the marker array
      marr.markers.push_back(room);
      marr.markers.push_back(door);
      marr.markers.push_back(box);
      marr.markers.push_back(rob_table);
      marr.markers.push_back(table);
    }

    // loop timer
    auto t2 = std::chrono::high_resolution_clock::now();
    auto d = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    running_sum = running_sum + d;
    if (d < shortest && d > 0) {
      shortest = d;
    }
    if (d > longest) {
      longest = d;
    }
    loop_counter++;
    if (timing_on) {
      average = running_sum/loop_counter;
      std::cout << average << " " << longest << " " << shortest << "\n";
    }

    // Publishing
    if (use_visualization) {
      pub_vis.publish(marr); // Visuals
      pub_vis.publish(articulated_acc_vis.markers_);
      pub_vis.publish(articulated_vel_vis.markers_);
      pub_vis.publish(articulated_pos_vis.markers_);
      pub_vis.publish(pedestrian_acc_vis.markers_);
      pub_vis.publish(pedestrian_vel_vis.markers_);
      if (use_robot_capsules && tsv.get_robot_capsules().size() > 0 && use_udp_transmission) {
        pub_vis.publish(robot_capsule_vis.markers_);
      }
      if (use_robot_capsules && rv.get_robot_capsules().size() > 0 && !use_udp_transmission) {
        pub_vis.publish(robot_capsule_vis.markers_);
      }
    }

    // ROS loop mechanism
    ros::spinOnce();
    loop_rate.sleep();
  }
  if (timing_on) {
    std::cout << "Average loop time: " << average;
  }
  return 0;
}
