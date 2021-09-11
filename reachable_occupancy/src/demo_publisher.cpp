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
#include <math.h>

#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

#include "reach_lib.hpp"
#include "validation.cpp"
#include "visualizer.cpp"
#include "volume.cpp"
#include "udp_transiever.cpp"

#include "visualization_msgs/MarkerArray.h"

// read from csv data
std::vector<std::pair<std::string, std::vector<double>>> read_csv(std::string filepath) {
  std::vector<std::pair<std::string, std::vector<double>>> result;

  // input
  std::ifstream pos_data(filepath);

  // check whether file is open or not
  if (!pos_data.is_open()) {
    throw std::runtime_error("Could not open file: " + filepath);
  } else {
    std::cout << "Successfully read: " << filepath << "\n";
  }

  std::string line, colname;
  double val;

  // Read colnames
  if (pos_data.good()) {
    // Extract the first line in the file
    std::getline(pos_data, line);

    // Create a stringstream from line
    std::stringstream ss(line);

    // Extract each column name
    while (std::getline(ss, colname, ',')) {
      // Initialize and add <colname, int vector> pairs to result
      result.push_back({colname, std::vector<double> {}});
    }
  }

  // Read data, line by line
  while (std::getline(pos_data, line)) {
    // Create a stringstream of the current line
    std::stringstream ss(line);

    // Keep track of the current column index
    int colIdx = 0;

    // Extract each integer
    while (ss >> val) {
      // Add the current integer to the 'colIdx' column's values vector
      result.at(colIdx).second.push_back(val);

      // If the next token is a comma, ignore it and move on
      if (ss.peek() == ',') ss.ignore();

      // Increment the column index
      colIdx++;
    }
  }

  // Close file
  pos_data.close();

  return result;
}


std::vector<reach_lib::Point> extract_frame(std::vector<std::pair<std::string,
                                            std::vector<double>>> pos_data,
                                            int frame, double scale) {
  std::vector<reach_lib::Point> pos;
  // std::vector<std::string> joints = {"RShoulder", "RForearm", "RHand", "LShoulder", "LForearm",
  // "LHand", "Neck", "Hip", "RThigh", "RShin", "RFoot", "LThigh", "LShin", "LFoot", "Head"};
  std::vector<int> index = {16, 19, 22, 43, 46, 49, 70, 1, 82, 85, 88, 100, 103, 106, 73};
  for (auto it : index) {
    reach_lib::Point p = reach_lib::Point();
    // std::cout<<"Extracting: "<<std::get<0>(pos_data[it])<<"\n";
    p.x = scale * std::get<1>(pos_data[it])[frame];
    p.y = scale * std::get<1>(pos_data[it+2])[frame];
    p.z = scale * std::get<1>(pos_data[it+1])[frame];
    pos.push_back(p);
  }
  return pos;
}







////////////////////////////////////////////////////////////////////////////////////////////
// Start of the MAIN TEST FUNCTION

typedef occupancy_containers::capsule::Capsule Capsule;

typedef visualizer::Visualizer Visualizer;

typedef volume::Volume Volume;

typedef udp_transiever::UDPTransiever UDPTransiever;

int main(int argc, char** argv) {
  // ros system
  ros::init(argc, argv, "reachable_occupancy_main");
  ros::NodeHandle nh("eno2");
  ros::Rate loop_rate(60);



  // UDP architecture
  UDPTransiever tsv = UDPTransiever(8, 1.0/1000.0, "192.168.7.13", "192.168.7.1", 8080);
  UDPTransiever tsv_rob = UDPTransiever(11, 1.0/1.0, "192.168.7.13", "192.168.7.1", 8079);
  bool rosbag = true;
  bool robot_capsules = false;
  if (!rosbag) {
    if (!UDPTransiever::create_socket(nh, "192.168.7.13", "192.168.7.1", 8080, false)) {
      ROS_WARN("Socket Not Connected 8080!");
    }
  }

  // Open port for robot capsules
  if (!rosbag && robot_capsules) {
    if (!UDPTransiever::create_socket(nh,"192.168.7.13", "192.168.7.1", 8079, false)) {
      ROS_WARN("Socket Not Connected 8079!");
    }
  }

  // Demo data setup
  std::map<std::string, std::pair<int, int>> index_demo;

  // Right arm init
  index_demo["RightUpperArm"] = std::pair<int, int> (3, 4);
  index_demo["RightForeArm"] = std::pair<int, int> (4, 5);
  index_demo["RightHand"] = std::pair<int, int> (5, 5);

  // Left arm init
  index_demo["LeftUpperArm"] = std::pair<int, int> (0, 1);
  index_demo["LeftForeArm"] = std::pair<int, int> (1, 2);
  index_demo["LeftHand"] = std::pair<int, int> (2, 2);

  // Center
  index_demo["Torso"] = std::pair<int, int> (8, 9);
  index_demo["Head"] = std::pair<int, int> (6, 6);


  std::vector<double> max_a_demo = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 30.0, 2.0,
                                20.0, 20.0};

  // Maximum velocity (the same for all joints)
  std::vector<double> max_v_demo;
  for (int i = 0; i < 10; i++) {
      max_v_demo.push_back(2.0);
  }

  // BodyPart thickness
  std::map<std::string, double> thickness_demo;
  thickness_demo["RightUpperArm"] = 0.1;
  thickness_demo["RightForeArm"] = 0.1;
  thickness_demo["RightHand"] = 0.205;
  thickness_demo["LeftUpperArm"] = 0.1;
  thickness_demo["LeftForeArm"] = 0.1;
  thickness_demo["LeftHand"] = 0.205;
  thickness_demo["Torso"] = 0.4;
  thickness_demo["Head"] = 0.4;

  reach_lib::System s_demo = reach_lib::System();

  // Position model data
  std::vector<double> max_v_p_demo = {2.46, 2.46};
  std::vector<double> length_demo = {0.635, 0.635};
  std::vector<double> thickness_p_demo = {0.205, 0.205};

  std::map<std::string, std::pair<int, int>> index_p_demo;

  // Initialize extremities
  index_p_demo["RightArm"] = std::pair<int, int> (1, 1);  // 3
  index_p_demo["LeftArm"] = std::pair<int, int> (0, 0);   // 0

  // Pedestrian model data
  double arm_span_demo = 1.8;
  double height_demo = 3.0;
  double max_a_ped_demo = 10.0;
  double max_v_ped_demo = 2.0;

  // Build Articulated human objects for UDP based demo purposes
  // reach_lib::ArticulatedAccel human_demo = reach_lib::ArticulatedAccel(s_demo, index_demo, thickness_demo, max_a_demo);
  reach_lib::ArticulatedVel human_demo = reach_lib::ArticulatedVel(s_demo, index_demo, thickness_demo, max_v_demo);
  // reach_lib::ArticulatedPos human_demo = reach_lib::ArticulatedPos(s_demo, index_p_demo, thickness_p_demo, max_v_p_demo, length_demo);
  reach_lib::Articulated* human_demo_p = &human_demo;

  // Build Pedestrian human objects for UDP based demo purposes
  // reach_lib::PedestrianAccel human_demo_ped = reach_lib::PedestrianAccel(s_demo, height_demo, arm_span_demo, max_a_ped_demo, -1.1);
  reach_lib::PedestrianVel human_demo_ped = reach_lib::PedestrianVel(s_demo, height_demo, arm_span_demo, max_v_ped_demo, -1.1);
  reach_lib::Pedestrian* human_demo_ped_p = &human_demo_ped;

  // Define validation
  validation::Validation val = validation::Validation(0.02, 1.0/60.0);

  // Set up volume
  // reach_lib::Point origin = reach_lib::Point(0.0, -2.0, 0.0);
  // volume::Volume vol = volume::Volume(2.0, 2.0, 2.0, 0.1, origin);

  visualization_msgs::Marker points_i = visualization_msgs::Marker();
  points_i.header.frame_id = "/map";
  points_i.header.stamp = ros::Time::now();
  points_i.ns = "points_i";
  points_i.action = visualization_msgs::Marker::ADD;
  points_i.type = visualization_msgs::Marker::POINTS;
  points_i.lifetime = ros::Duration(1);
  points_i.color.r = 1.0;
  points_i.color.a = 1.0;
  points_i.id = 2222;
  points_i.scale.x = 0.02;
  points_i.scale.y = 0.02;

  visualization_msgs::Marker points_o = visualization_msgs::Marker();
  points_o.header.frame_id = "/map";
  points_o.header.stamp = ros::Time::now();
  points_o.ns = "points_o";
  points_o.action = visualization_msgs::Marker::ADD;
  points_o.type = visualization_msgs::Marker::POINTS;
  points_o.lifetime = ros::Duration(1);
  points_o.color.g = 1.0;
  points_o.color.a = 1.0;
  points_o.id = 2222;
  points_o.scale.x = 0.02;
  points_o.scale.y = 0.02;

  // Visualization
  Visualizer v_demo = Visualizer();
  v_demo.mode_ = "VIS";
  v_demo.set_id_map(human_demo_p, 300);
  v_demo.color_.a = 0.4;

  Visualizer r_demo = Visualizer();
  r_demo.set_id_map(7, 3);
  r_demo.color_.a = 0.6;
  r_demo.color_.r = 1.0;

  
  // Colors
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.a = 1.0;

  std_msgs::ColorRGBA green;
  green.g = 1.0;
  green.a = 1.0;

  std_msgs::ColorRGBA blue;
  blue.b = 1.0;
  blue.a = 1.0;

  std_msgs::ColorRGBA black;
  black.r = 0.0;
  black.b = 0.0;
  black.g = 0.0;
  black.a = 1.0;

  std_msgs::ColorRGBA pink;
  pink.r = 1.0;
  pink.b = 1.0;
  pink.a = 1.0;

  std_msgs::ColorRGBA cyan;
  cyan.g = 1.0;
  cyan.b = 1.0;
  cyan.a = 1.0;

  std_msgs::ColorRGBA orange;
  orange.g = 165.0/256.0;
  orange.r = 1.0;
  orange.a = 1.0;

  std_msgs::ColorRGBA purple;
  purple.r = 138;
  purple.g = 43;
  purple.b = 226;
  purple.a = 1.0;

  // Subscribers for experimental data
  ros::Subscriber frame_data_subscriber = nh.subscribe("udp/p8080", 1000,
                                          &udp_transiever::UDPTransiever::udp_callback, &tsv);

  ros::Subscriber robot_subscriber = nh.subscribe("udp/p8079", 1000,
                                     &udp_transiever::UDPTransiever::robot_callback, &tsv_rob);











  



  // timer
  long double running_sum = 0.0;
  double loop_counter = 1.0;
  double average = 0.0;
  double longest = 0.0;
  int shortest = 1E9;
  bool timing = false;

  // get data
  std::string filepath = "/home/sven/catkin_ws/src/reachable_occupancy/src/tests/simple_skeleton_mvmt.csv";
  if (ros::param::has("/csv_path")) {
      ros::param::get("/csv_path", filepath);
  } else {
      throw "Could not get filepath from ros-parameter server!";
  }

  // was: "./tests/simple_skeleton_mvmt.csv"
  std::vector<std::pair<std::string, std::vector<double>>> pos_data = read_csv(filepath);


  // Test parameters
  double t_a = 0.0;
  double t_b = 0.2;
  double frame_time = std::get<1>(pos_data[0])[1];
  double scale = 0.07;
  std::vector<std::pair<int, std::vector<reach_lib::Point>>> frame_pos;
  std::vector<std::pair<int, std::vector<reach_lib::Point>>> frame_vel;
  for (int i = 0; i < pos_data.size(); i++) {
      frame_pos.push_back({i, extract_frame(pos_data, i, scale)});
  }


  // Calculating velocity frame by frame from frame positions
  for (int i = 0; i < frame_pos.size(); i++) {
      std::vector<reach_lib::Point> pv;
      if (i == 0) {
          for (int j = 0; j < std::get<1>(pos_data[0]).size(); j++) {
              pv.push_back(reach_lib::Point());
          }
          frame_vel.push_back({i, pv});
      } else {
          std::vector<reach_lib::Point> pos_previous = std::get<1>(frame_pos[i-1]);
          std::vector<reach_lib::Point> pos_current = std::get<1>(frame_pos[i]);
          for (int j = 0; j < pos_previous.size(); j++) {
              reach_lib::Point d = pos_current[i] - pos_previous[i];
              double fact = reach_lib::Point::norm(d)/frame_time;
              d.x *= fact;
              d.y *= fact;
              d.z *= fact;
              pv.push_back(d);
          }
          frame_vel.push_back({i, pv});
      }
  }
  assert(frame_vel.size() == frame_pos.size());

  // Dummy values ACCEL and VEL ordered as defned in the joint-bodyPart map
  reach_lib::System s = reach_lib::System();

  std::map<std::string, std::pair<int, int>> index;

  // Right arm init
  index["RightUpperArm"] = std::pair<int, int> (0, 1);
  index["RightForeArm"] = std::pair<int, int> (1, 2);
  index["RightHand"] = std::pair<int, int> (2, 2);

  // Left arm init
  index["LeftUpperArm"] = std::pair<int, int> (3, 4);
  index["LeftForeArm"] = std::pair<int, int> (4, 5);
  index["LeftHand"] = std::pair<int, int> (5, 5);

  // Torso init
  index["Torso"] = std::pair<int, int> (6, 7);

  // Right leg init
  index["RightThigh"] = std::pair<int, int> (8, 9);
  index["RightShin"] = std::pair<int, int> (9, 10);
  index["RightFoot"] = std::pair<int, int> (10, 10);

  // Left leg init
  index["LeftThigh"] = std::pair<int, int> (11, 12);
  index["LeftShin"] = std::pair<int, int> (12, 13);
  index["LeftFoot"] = std::pair<int, int> (13, 13);

  // Head init
  index["Head"] = std::pair<int, int> (14, 14);

  // Maximum acceleration
  std::vector<double> max_a = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 20.0, 20.0,
                                30.0, 30.0, 50.0, 30.0, 30.0, 50.0, 25.0};

  // Maximum velocity (the same for all joints)
  std::vector<double> max_v;
  for (int i = 0; i < 15; i++) {
      max_v.push_back(14.0);
  }

  // BodyPart thickness
  std::map<std::string, double> thickness;
  thickness["RightUpperArm"] = 0.1;
  thickness["RightForeArm"] = 0.1;
  thickness["RightHand"] = 0.205;
  thickness["LeftUpperArm"] = 0.1;
  thickness["LeftForeArm"] = 0.1;
  thickness["LeftHand"] = 0.205;
  thickness["Torso"] = 0.3;
  thickness["RightThigh"] = 0.2;
  thickness["RightShin"] = 0.2;
  thickness["RightFoot"] = 0.3;
  thickness["LeftThigh"] = 0.2;
  thickness["LeftShin"] = 0.2;
  thickness["LeftFoot"] = 0.3;
  thickness["Head"] = 0.3;


  // dummy values POS
  std::vector<double> max_v_p = {2.46, 2.46, 2.46, 2.46};
  std::vector<double> length = {0.635, 0.635, 0.81, 0.81};
  std::vector<double> thickness_p = {0.205, 0.205, 0.3, 0.3};

  std::map<std::string, std::pair<int, int>> index_p;

  // Initialize extremities
  index_p["RightArm"] = std::pair<int, int> (0, 0);  // 0
  index_p["LeftArm"] = std::pair<int, int> (1, 1);   // 3
  index_p["RightLeg"] = std::pair<int, int> (2, 2);  // 8
  index_p["LeftLeg"] = std::pair<int, int> (3, 3);   // 11

  // dummy values Pedestrian
  double arm_span = 2.0;
  double height = 2.0;
  double max_a_ped = 10.0;
  double max_v_ped = 14.0;

  // Build ACCEL and VEL human objects
  reach_lib::ArticulatedAccel human_a = reach_lib::ArticulatedAccel(s, index, thickness, max_a);
  reach_lib::ArticulatedVel human_v = reach_lib::ArticulatedVel(s, index, thickness, max_v);

  // Build POS human object
  reach_lib::ArticulatedPos human_p = reach_lib::ArticulatedPos(s, index_p, thickness_p, max_v_p, length);

  // Build Pedestrian objects
  reach_lib::PedestrianAccel pedestrian_a = reach_lib::PedestrianAccel(s, height, arm_span, max_a_ped);
  reach_lib::PedestrianVel pedestrian_v = reach_lib::PedestrianVel(s, height, arm_span, max_v_ped);

  // Generate pointers to all models
  reach_lib::Articulated* human_a_p = &human_a;
  reach_lib::Articulated* human_v_p = &human_v;
  reach_lib::Articulated* human_p_p = &human_p;

  reach_lib::Pedestrian* pedestrian_a_p = &pedestrian_a;
  reach_lib::Pedestrian* pedestrian_v_p = &pedestrian_v;

  // Initalize the visualizers
  Visualizer v_a = Visualizer();
  v_a.mode_ = "ARTICULATED-VEL";
  v_a.set_id_map(human_v_p, 1);
  v_a.color_.a = 0.4;

  Visualizer v_p = Visualizer();
  v_p.mode_ = "PEDESTRIAN-ACCEL";
  v_p.set_id_map(pedestrian_a_p, 2);
  v_p.color_.r = 1.0;
  v_p.color_.b = 0.0;
  v_p.color_.g = 0.0;
  

  // Publisher initialization
  ros::Publisher pub_vis = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1000);








  // t_break measures
  long double max_t_break = 0.0;
  long double avg_t_break = 0.0;
  long double sum_t_break = 0.0;
  long double dev_t_break = 0.0;

  // Container for avg volume
  long double sum_vol = 0.0;
  long double avg_vol = 0.0;


  // Array for average vel
  long int counter = 0;
  std::vector<long double> avg_vel = {};
  std::vector<long double> dev_vel = {};
  std::vector<long double> sum_vel = {};
  std::vector<long double> fastest_vel = {};
  std::vector<long double> slowest_vel = {};

  while (ros::ok() && frame_pos.size() > 0) {
    auto t1 = std::chrono::high_resolution_clock::now();


    // current t_break
    if (tsv_rob.get_t_break() > 0.0) {
      t_b = tsv_rob.get_t_break(); 
      if (t_b > max_t_break) {
        max_t_break = t_b;
        sum_t_break += t_b;
        if (counter > 0) {
          long double old_avg_t_break = avg_t_break;
          avg_t_break = sum_t_break/static_cast<long double>(counter);
          dev_t_break = (dev_t_break * dev_t_break + (1.0/static_cast<long double>(counter))
                      * ((t_b - avg_t_break) * (t_b - old_avg_t_break) - dev_t_break * dev_t_break));
        }
      }
    }
    // std::cout<< "New t_break: " << t_b << "\n";

    // Visualize robot capsules
    if (tsv_rob.get_robot_capsules().size() > 0) {
      r_demo.vis_capsules(tsv_rob.get_robot_capsules());
    }

    // current positions and velocities
    std::vector<reach_lib::Point> pos = std::get<1>(frame_pos[0]);
    std::vector<reach_lib::Point> vel = std::get<1>(frame_vel[0]);

    // Extract only the required positions from pos for ArticulatedPos
    std::vector<reach_lib::Point> pos_p;
    pos_p.push_back(pos[0]);
    pos_p.push_back(pos[3]);
    pos_p.push_back(pos[8]);
    pos_p.push_back(pos[11]);

    // Extract only the required positions and velocities from pos for Pedestrian
    std::vector<reach_lib::Point> pos_ped;
    reach_lib::Point hip_center = pos[11] - pos[8];
    hip_center.x = pos[8].x + 0.5*hip_center.x;
    hip_center.y = pos[8].y + 0.5*hip_center.y;
    hip_center.z = pos[8].z + 0.5*hip_center.z;
    pos_ped.push_back(hip_center);

    std::vector<reach_lib::Point> vel_ped;
    reach_lib::Point hip_vel = vel[11] - vel[8];
    hip_vel.x = vel[8].x + 0.5*hip_vel.x;
    hip_vel.y = vel[8].y + 0.5*hip_vel.y;
    hip_vel.z = vel[8].z + 0.5*hip_vel.z;
    vel_ped.push_back(hip_vel);

    // Update UDP based demo
    std::vector<bool> human_robot_intersections = {};
    if (tsv.get_joint_pos().size() > 0) {
      std::vector<reach_lib::Point> te = {tsv.get_joint_pos()[0], tsv.get_joint_pos()[3]};
      std::vector<reach_lib::Point> te_ped = {tsv.get_joint_pos()[8]};
      
      // Articulated
      // human_demo.update(t_a, t_b, tsv.get_joint_pos(), tsv.get_joint_vel());
      human_demo.update(t_a, t_b, tsv.get_joint_pos());
      // human_demo.update(t_a, t_b, te);

      // Pedestrian
      // human_demo_ped.update(t_a, t_b, {tsv.get_joint_pos()[8]}, {tsv.get_joint_vel()[8]});
      human_demo_ped.update(t_a, t_b, {tsv.get_joint_pos()[8]});

      // Check for intersections between human and robot capsules
      if (tsv_rob.get_robot_capsules().size() > 0) {
        human_robot_intersections = val.capsule_set_intersection(reach_lib::get_capsules(human_demo), tsv_rob.get_robot_capsules());
        // human_robot_intersections = val.cylinder_capsule_set_intersction(reach_lib::get_cylinders(human_demo_ped), tsv_rob.get_robot_capsules());
      }

    }

    // Get velocities max avg
    bool clmp = true;
    bool b_vel = true;
    double m_vel = 2.0;
    if (tsv.get_joint_vel().size() > 0 && b_vel) {
      std::vector<long double> c_vel = {};
      for (const auto& it : tsv.get_joint_vel()) {
        c_vel.push_back(reach_lib::Point::norm(it));
      }
      if (sum_vel.size() == 0) {
        sum_vel = c_vel;
        slowest_vel = sum_vel;
        fastest_vel = sum_vel;
        counter++;
      } else {
        for (int i = 0; i < sum_vel.size(); i++) {
          if (clmp && c_vel[i] > m_vel) {
            sum_vel[i] = sum_vel[i] + m_vel;
          } else {
            sum_vel[i] = sum_vel[i] + reach_lib::Point::norm(tsv.get_joint_vel()[i]);
          }
        }
        counter++;
      }
      for (int i = 0; i < sum_vel.size(); i++) {
        if (c_vel[i] > fastest_vel[i]) {
          if (clmp && c_vel[i] > m_vel) {
          } else {
            fastest_vel[i] = c_vel[i];
          }
        } else if (c_vel[i] < slowest_vel[i] && c_vel[i] > 0.001) {
          slowest_vel[i] = c_vel[i];
        }
      }
      avg_vel = {};
      for (const auto& it : sum_vel) {
        avg_vel.push_back(it/static_cast<long double>(counter));
      }
    }



    // Update Articulated models
    // human_a.update(t_a, t_b, pos, vel);
    // human_v.update(t_a, t_b, pos);
    // human_p.update(t_a, t_b, pos_p);

    // Update Pedestrian models
    // pedestrian_a.update(t_a, t_b, pos_ped, vel_ped);
    // pedestrian_v.update(t_a, t_b, pos_ped);
    

    // Generate rviz markers
    // v_a.vis_articulated(human_v_p);
    // v_p.vis_pedestrian(pedestrian_a_p);


    // Intersection checks and visualization
    if (tsv.get_joint_pos().size() > 0) {
      val.validation(human_demo, t_b, tsv.get_joint_pos());
      // val.validation(human_demo_ped, t_b, tsv.get_joint_pos());
    }

    if (val.get_a_pos().size() >= val.get_backlog()) {
      std::vector<reach_lib::Capsule> caps = {};
      for (auto& it : val.get_a_pos()[val.get_val_counter()[2]].first.get_occupancy()) {
        caps.push_back(it.get_occupancy());
      }
      v_demo.vis_capsules(caps, human_robot_intersections);
    }

    if (val.get_a_vel().size() >= val.get_backlog()) {
      std::vector<reach_lib::Capsule> caps = {};
      for (auto& it : val.get_a_vel()[val.get_val_counter()[1]].first.get_occupancy()) {
        caps.push_back(it.get_occupancy());
      }
      v_demo.vis_capsules(caps, human_robot_intersections);
    }

    if (val.get_a_accel().size() >= val.get_backlog()) {
      std::vector<reach_lib::Capsule> caps = {};
      for (auto& it : val.get_a_accel()[val.get_val_counter()[0]].first.get_occupancy()) {
        caps.push_back(it.get_occupancy());
      }
      v_demo.vis_capsules(caps, human_robot_intersections);
    }

    if (val.get_p_accel().size() >= val.get_backlog()) {
      reach_lib::PedestrianAccel* p_a = &val.get_p_accel()[val.get_val_counter()[3]].first;
      v_demo.vis_pedestrian(p_a, human_robot_intersections);
    }

    if (val.get_p_vel().size() >= val.get_backlog()) {
      reach_lib::PedestrianVel* p_v = &val.get_p_vel()[val.get_val_counter()[4]].first;
      v_demo.vis_pedestrian(p_v, human_robot_intersections);
    }

    reach_lib::Point p = reach_lib::Point(0.0, -2.0, 1.0);
    // std::cout << "Bool check demo_a: " << human_demo.intersection({p}) << "\n";


    // Get volume
    Volume vol = Volume(4, 4, 3.5, 0.1, reach_lib::Point(0.0, 0.0, 0.0));
    vol.set_grid({});
    if (tsv.get_joint_pos().size()) {
      vol.set_origin(reach_lib::Point(tsv.get_joint_pos()[8].x - 2, tsv.get_joint_pos()[8].y - 2, tsv.get_joint_pos()[8].z - 1.5));
      vol.volume_routine(human_demo);
      std::cout << "Volume: " << vol.get_volume() << "\n";
      sum_vol += vol.get_volume();
      avg_vol = sum_vol/static_cast<long double>(counter);
    }

    points_i.points = {};
    for (const auto& it: vol.points_in) {
      geometry_msgs::Point gp;
      gp.x = it.x;
      gp.y = it.y;
      gp.z = it.z;
      points_i.points.push_back(gp);
    }

    points_o.points = {};
    for (const auto& it: vol.points_out) {
      geometry_msgs::Point gp;
      gp.x = it.x;
      gp.y = it.y;
      gp.z = it.z;
      points_o.points.push_back(gp);
    }


    /*
    reach_lib::Point p = reach_lib::Point();
    std::cout << "Bool check aa: " << human_a.intersection({p}) << "\n";
    std::cout << "Bool check av: " << human_v.intersection({p}) << "\n";
    std::cout << "Bool check ap: " << human_p.intersection({p}) << "\n";
    std::cout << "Bool check pa: " << pedestrian_a.intersection({p}) << "\n";
    std::cout << "Bool check pv: " << pedestrian_v.intersection({p}) << "\n";*/

    reach_lib::Capsule int_cap = reach_lib::Capsule(reach_lib::Point(0, -1, 1), reach_lib::Point(0, -1, 2), 1.0);
    reach_lib::Cylinder int_cyl = reach_lib::Cylinder(reach_lib::Point(0, -1, 1), reach_lib::Point(0, -1, 2), 1.0);
    auto tcap = human_a.get_occupancy()[0].get_occupancy();
    // std::cout << "Capdist: " << reach_lib::intersections::cylinder_capsule_dist(int_cyl, tcap) <<  "\n";



    // Erase the top vector element
    if (frame_pos.size() > 1) {
      frame_pos.erase(frame_pos.begin());
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
    if (timing) {
      average = running_sum/loop_counter;
      std::cout << average << " " << longest << " " << shortest << "\n";
    }

    // Publish visualization to rviz' marker topic
    // Visualize points
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


    visualization_msgs::Marker skel = visualization_msgs::Marker();

    skel.header.frame_id = "/map";
    skel.header.stamp = ros::Time::now();
    skel.ns = "skelt";
    skel.action = visualization_msgs::Marker::ADD;
    skel.type = visualization_msgs::Marker::POINTS;
    skel.lifetime = ros::Duration(1);

    skel.colors.push_back(red);
    skel.colors.push_back(green);
    skel.colors.push_back(black);
    skel.colors.push_back(blue);
    skel.colors.push_back(cyan);
    skel.colors.push_back(pink);
    skel.colors.push_back(orange);
    skel.colors.push_back(purple);
    skel.colors.push_back(orange);
    skel.colors.push_back(orange);

    skel.id = 1111;
    skel.scale.x = 0.1;
    skel.scale.y = 0.1;


    skel.points = {};
    for (int i = 0; i < tsv.get_jlMSG().positions.size(); i++)  {
      skel.points.push_back(tsv.get_jlMSG().positions[i]);
    }

    visualization_msgs::MarkerArray marr;
    marr.markers.push_back(skel);
    marr.markers.push_back(rob_table);
    marr.markers.push_back(table);
    marr.markers.push_back(room);
    marr.markers.push_back(box);
    marr.markers.push_back(door);

    // Volume grid
    marr.markers.push_back(points_i);
    marr.markers.push_back(points_o);

    // Models
    pub_vis.publish(v_demo.markers_);
    pub_vis.publish(marr);
    // pub_vis.publish(v_a.markers_);
    // pub_vis.publish(v_p.markers_);


    pub_vis.publish(r_demo.markers_);

    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout << "Average loop time: " << average << "\n";
  if (avg_vel.size() >= 10) {
    std::cout << "Avg vel: " << avg_vel[0] << " " << avg_vel[1] << " " << avg_vel[2] 
                      << " " << avg_vel[3] << " " << avg_vel[4] << " " << avg_vel[5]
                      << " " << avg_vel[6] << " " << avg_vel[8] << " " << avg_vel[9] << "\n";
  }
  if (fastest_vel.size() >= 10) {
    std::cout << "Fastest vel: " << fastest_vel[0] << " " << fastest_vel[1] << " " << fastest_vel[2] 
                          << " " << fastest_vel[4] << " " << fastest_vel[5] << " " << fastest_vel[6]
                          << " " << fastest_vel[6] << " " << fastest_vel[8] << " " << fastest_vel[9] << "\n";
  }
  if (slowest_vel.size() >= 10) {
    std::cout << "Slowest vel: " << slowest_vel[0] << " " << slowest_vel[1] << " " << slowest_vel[2] 
                          << " " << slowest_vel[4] << " " << slowest_vel[5] << " " << slowest_vel[6]
                          << " " << slowest_vel[6] << " " << slowest_vel[8] << " " << slowest_vel[9] << "\n";
  }
  std::cout << "Avg vol: " << avg_vol << "\n";
  std::cout << "Max t_break: " << max_t_break << "Avg t_break: " << avg_t_break << "Std dev t_break: " << dev_t_break << "\n";
  return 0;
}





// Computation time test:

/*
int main(int argc, char** argv) {
  ros::init(argc, argv, "reachable_occupancy_main");
  ros::NodeHandle nh("eno2");
  ros::Rate loop_rate(60);

  // timer
  long double running_sum = 0.0;
  double loop_counter = 1.0;
  double average = 0.0;
  double longest = 0.0;
  int shortest = 1E9;
  bool timing = true;

  // get data
  std::string filepath = "/home/sven/catkin_ws/src/reachable_occupancy/src/tests/simple_skeleton_mvmt.csv";
  if (ros::param::has("/csv_path")) {
      ros::param::get("/csv_path", filepath);
  } else {
      throw "Could not get filepath from ros-parameter server!";
  }

  // was: "./tests/simple_skeleton_mvmt.csv"
  std::vector<std::pair<std::string, std::vector<double>>> pos_data = read_csv(filepath);


  // Test parameters
  double t_a = 0.0;
  double t_b = 0.2;
  double frame_time = std::get<1>(pos_data[0])[1];
  double scale = 0.07;
  std::vector<std::pair<int, std::vector<reach_lib::Point>>> frame_pos;
  std::vector<std::pair<int, std::vector<reach_lib::Point>>> frame_vel;
  for (int i = 0; i < pos_data.size(); i++) {
      frame_pos.push_back({i, extract_frame(pos_data, i, scale)});
  }


  // Calculating velocity frame by frame from frame positions
  for (int i = 0; i < frame_pos.size(); i++) {
      std::vector<reach_lib::Point> pv;
      if (i == 0) {
          for (int j = 0; j < std::get<1>(pos_data[0]).size(); j++) {
              pv.push_back(reach_lib::Point());
          }
          frame_vel.push_back({i, pv});
      } else {
          std::vector<reach_lib::Point> pos_previous = std::get<1>(frame_pos[i-1]);
          std::vector<reach_lib::Point> pos_current = std::get<1>(frame_pos[i]);
          for (int j = 0; j < pos_previous.size(); j++) {
              reach_lib::Point d = pos_current[i] - pos_previous[i];
              double fact = reach_lib::Point::norm(d)/frame_time;
              d.x *= fact;
              d.y *= fact;
              d.z *= fact;
              pv.push_back(d);
          }
          frame_vel.push_back({i, pv});
      }
  }
  assert(frame_vel.size() == frame_pos.size());

  std::cout << frame_pos.size() << "\n";

  // Dummy values ACCEL and VEL ordered as defned in the joint-bodyPart map
  reach_lib::System s = reach_lib::System();

  std::map<std::string, std::pair<int, int>> index;

  // Right arm init
  index["RightUpperArm"] = std::pair<int, int> (0, 1);
  index["RightForeArm"] = std::pair<int, int> (1, 2);
  index["RightHand"] = std::pair<int, int> (2, 2);

  // Left arm init
  index["LeftUpperArm"] = std::pair<int, int> (3, 4);
  index["LeftForeArm"] = std::pair<int, int> (4, 5);
  index["LeftHand"] = std::pair<int, int> (5, 5);

  // Torso init
  index["Torso"] = std::pair<int, int> (6, 7);

  // Right leg init
  index["RightThigh"] = std::pair<int, int> (8, 9);
  index["RightShin"] = std::pair<int, int> (9, 10);
  index["RightFoot"] = std::pair<int, int> (10, 10);

  // Left leg init
  index["LeftThigh"] = std::pair<int, int> (11, 12);
  index["LeftShin"] = std::pair<int, int> (12, 13);
  index["LeftFoot"] = std::pair<int, int> (13, 13);

  // Head init
  index["Head"] = std::pair<int, int> (14, 14);

  // Maximum acceleration
  std::vector<double> max_a = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 20.0, 20.0,
                                30.0, 30.0, 50.0, 30.0, 30.0, 50.0, 25.0};

  // Maximum velocity (the same for all joints)
  std::vector<double> max_v;
  for (int i = 0; i < 15; i++) {
      max_v.push_back(14.0);
  }

  // BodyPart thickness
  std::map<std::string, double> thickness;
  thickness["RightUpperArm"] = 0.1;
  thickness["RightForeArm"] = 0.1;
  thickness["RightHand"] = 0.205;
  thickness["LeftUpperArm"] = 0.1;
  thickness["LeftForeArm"] = 0.1;
  thickness["LeftHand"] = 0.205;
  thickness["Torso"] = 0.3;
  thickness["RightThigh"] = 0.2;
  thickness["RightShin"] = 0.2;
  thickness["RightFoot"] = 0.3;
  thickness["LeftThigh"] = 0.2;
  thickness["LeftShin"] = 0.2;
  thickness["LeftFoot"] = 0.3;
  thickness["Head"] = 0.3;


  // dummy values POS
  std::vector<double> max_v_p = {2.46, 2.46, 2.46, 2.46};
  std::vector<double> length = {0.635, 0.635, 0.81, 0.81};
  std::vector<double> thickness_p = {0.205, 0.205, 0.3, 0.3};

  std::map<std::string, std::pair<int, int>> index_p;

  // Initialize extremities
  index_p["RightArm"] = std::pair<int, int> (0, 0);  // 0
  index_p["LeftArm"] = std::pair<int, int> (1, 1);   // 3
  index_p["RightLeg"] = std::pair<int, int> (2, 2);  // 8
  index_p["LeftLeg"] = std::pair<int, int> (3, 3);   // 11

  // dummy values Pedestrian
  double arm_span = 2.0;
  double height = 2.0;
  double max_a_ped = 10.0;
  double max_v_ped = 14.0;

  // Build ACCEL and VEL human objects
  reach_lib::ArticulatedAccel human_a = reach_lib::ArticulatedAccel(s, index, thickness, max_a);
  reach_lib::ArticulatedVel human_v = reach_lib::ArticulatedVel(s, index, thickness, max_v);

  // Build POS human object
  reach_lib::ArticulatedPos human_p = reach_lib::ArticulatedPos(s, index_p, thickness_p, max_v_p, length);

  // Build Pedestrian objects
  reach_lib::PedestrianAccel pedestrian_a = reach_lib::PedestrianAccel(s, height, arm_span, max_a_ped);
  reach_lib::PedestrianVel pedestrian_v = reach_lib::PedestrianVel(s, height, arm_span, max_v_ped);

  // Generate pointers to all models
  reach_lib::Articulated* human_a_p = &human_a;
  reach_lib::Articulated* human_v_p = &human_v;
  reach_lib::Articulated* human_p_p = &human_p;

  reach_lib::Pedestrian* pedestrian_a_p = &pedestrian_a;
  reach_lib::Pedestrian* pedestrian_v_p = &pedestrian_v;

  while (frame_pos.size() > 0) {

    std::cout << "Size: " << frame_pos.size() << "\n";

    auto t1 = std::chrono::high_resolution_clock::now();

    // current positions and velocities
    std::vector<reach_lib::Point> pos = std::get<1>(frame_pos[0]);
    std::vector<reach_lib::Point> vel = std::get<1>(frame_vel[0]);

    // Extract only the required positions from pos for ArticulatedPos
    std::vector<reach_lib::Point> pos_p;
    pos_p.push_back(pos[0]);
    pos_p.push_back(pos[3]);
    pos_p.push_back(pos[8]);
    pos_p.push_back(pos[11]);

    // Extract only the required positions and velocities from pos for Pedestrian
    std::vector<reach_lib::Point> pos_ped;
    reach_lib::Point hip_center = pos[11] - pos[8];
    hip_center.x = pos[8].x + 0.5*hip_center.x;
    hip_center.y = pos[8].y + 0.5*hip_center.y;
    hip_center.z = pos[8].z + 0.5*hip_center.z;
    pos_ped.push_back(hip_center);

    std::vector<reach_lib::Point> vel_ped;
    reach_lib::Point hip_vel = vel[11] - vel[8];
    hip_vel.x = vel[8].x + 0.5*hip_vel.x;
    hip_vel.y = vel[8].y + 0.5*hip_vel.y;
    hip_vel.z = vel[8].z + 0.5*hip_vel.z;
    vel_ped.push_back(hip_vel);

    // Update Articulated models
    human_a.update(t_a, t_b, pos, vel);
    human_v.update(t_a, t_b, pos);
    human_p.update(t_a, t_b, pos_p);

    // Update Pedestrian models
    pedestrian_a.update(t_a, t_b, pos_ped, vel_ped);
    pedestrian_v.update(t_a, t_b, pos_ped);

    // Erase the top vector element
    if (frame_pos.size() > 1) {
      frame_pos.erase(frame_pos.begin());
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
    if (timing) {
      average = running_sum/loop_counter;
      std::cout << average << " " << longest << " " << shortest << "\n";
    }
  }

  std::cout<< "Average loop time: " << average;
  return 0;
}

// Capsule intersection demo

    reach_lib::Capsule c1 = reach_lib::Capsule(reach_lib::Point(1.0, 1.0, 2.0), reach_lib::Point(1.5, 1.5, -2.0), 1.0);
    reach_lib::Capsule c2 = reach_lib::Capsule(reach_lib::Point(1.0, 1.0, -1.0), reach_lib::Point(-1.0, -1.0, -1.0), 1.0);



    visualization_msgs::MarkerArray capdem;

    visualization_msgs::Marker s11_dem = visualization_msgs::Marker();

    s11_dem.header.frame_id = "/map";
    s11_dem.header.stamp = ros::Time::now();
    s11_dem.ns = "s11";
    s11_dem.action = visualization_msgs::Marker::ADD;
    s11_dem.type = visualization_msgs::Marker::SPHERE;
    s11_dem.lifetime = ros::Duration(1);
    s11_dem.id = 100001;


    bool inter = false;
    inter = reach_lib::intersections::capsule_capsule_intersection(c1, c2);
    // reach_lib::Capsule(c1.p1_, c1.p1_, c1.r_)
    if (inter) {
      s11_dem.color.a = 0.6;
      s11_dem.color.r = 1.0;
      s11_dem.color.g = 0.0;
      s11_dem.color.b = 0.0;
    } else {
      s11_dem.color.a = 0.6;
      s11_dem.color.r = 0.0;
      s11_dem.color.g = 0.0;
      s11_dem.color.b = 1.0;
    }

    s11_dem.scale.x = 2*c1.r_;
    s11_dem.scale.y = 2*c1.r_;
    s11_dem.scale.z = 2*c1.r_;

    s11_dem.pose.position.x = c1.p1_.x;
    s11_dem.pose.position.y = c1.p1_.y;
    s11_dem.pose.position.z = c1.p1_.z;

    visualization_msgs::Marker s12_dem = visualization_msgs::Marker();

    s12_dem.header.frame_id = "/map";
    s12_dem.header.stamp = ros::Time::now();
    s12_dem.ns = "s12";
    s12_dem.action = visualization_msgs::Marker::ADD;
    s12_dem.type = visualization_msgs::Marker::SPHERE;
    s12_dem.lifetime = ros::Duration(1);
    s12_dem.id = 100002;

    s12_dem.color.a = 0.6;
    s12_dem.color.r = 0.0;
    s12_dem.color.g = 0.0;
    s12_dem.color.b = 1.0;

    s12_dem.scale.x = 2*c1.r_;
    s12_dem.scale.y = 2*c1.r_;
    s12_dem.scale.z = 2*c1.r_;

    s12_dem.pose.position.x = c1.p2_.x;
    s12_dem.pose.position.y = c1.p2_.y;
    s12_dem.pose.position.z = c1.p2_.z;

    visualization_msgs::Marker c1_dem = visualization_msgs::Marker();

    c1_dem.header.frame_id = "/map";
    c1_dem.header.stamp = ros::Time::now();
    c1_dem.ns = "c1";
    c1_dem.action = visualization_msgs::Marker::ADD;
    c1_dem.type = visualization_msgs::Marker::CYLINDER;
    c1_dem.lifetime = ros::Duration(1);
    c1_dem.id = 100003;

    c1_dem.color.a = 0.6;
    c1_dem.color.r = 0.0;
    c1_dem.color.g = 0.0;
    c1_dem.color.b = 1.0;

    c1_dem.scale.x = 2*c1.r_;
    c1_dem.scale.y = 2*c1.r_;
    c1_dem.scale.z = reach_lib::Point::norm(c1.p2_, c1.p1_);

    reach_lib::Point origin1 = reach_lib::Point::origin(c1.p2_, c1.p1_);
    geometry_msgs::Point pc1;
    pc1.x = origin1.x;
    pc1.y = origin1.y;
    pc1.z = origin1.z;
    c1_dem.pose.position = pc1;

    tf2::convert(Visualizer::orientation(c1.p1_, c1.p2_), c1_dem.pose.orientation);

    capdem.markers.push_back(s11_dem);
    capdem.markers.push_back(s12_dem);
    capdem.markers.push_back(c1_dem);




    visualization_msgs::Marker s21_dem = visualization_msgs::Marker();

    s21_dem.header.frame_id = "/map";
    s21_dem.header.stamp = ros::Time::now();
    s21_dem.ns = "s21";
    s21_dem.action = visualization_msgs::Marker::ADD;
    s21_dem.type = visualization_msgs::Marker::SPHERE;
    s21_dem.lifetime = ros::Duration(1);
    s21_dem.id = 100021;

    s21_dem.color.a = 0.6;
    s21_dem.color.r = 0.0;
    s21_dem.color.g = 0.0;
    s21_dem.color.b = 1.0;

    s21_dem.scale.x = 2*c2.r_;
    s21_dem.scale.y = 2*c2.r_;
    s21_dem.scale.z = 2*c2.r_;

    s21_dem.pose.position.x = c2.p1_.x;
    s21_dem.pose.position.y = c2.p1_.y;
    s21_dem.pose.position.z = c2.p1_.z;

    visualization_msgs::Marker s22_dem = visualization_msgs::Marker();

    s22_dem.header.frame_id = "/map";
    s22_dem.header.stamp = ros::Time::now();
    s22_dem.ns = "s22";
    s22_dem.action = visualization_msgs::Marker::ADD;
    s22_dem.type = visualization_msgs::Marker::SPHERE;
    s22_dem.lifetime = ros::Duration(1);
    s22_dem.id = 100022;

    s22_dem.color.a = 0.6;
    s22_dem.color.r = 0.0;
    s22_dem.color.g = 0.0;
    s22_dem.color.b = 1.0;

    s22_dem.scale.x = 2*c2.r_;
    s22_dem.scale.y = 2*c2.r_;
    s22_dem.scale.z = 2*c2.r_;

    s22_dem.pose.position.x = c2.p2_.x;
    s22_dem.pose.position.y = c2.p2_.y;
    s22_dem.pose.position.z = c2.p2_.z;

    visualization_msgs::Marker c2_dem = visualization_msgs::Marker();

    c2_dem.header.frame_id = "/map";
    c2_dem.header.stamp = ros::Time::now();
    c2_dem.ns = "c2";
    c2_dem.action = visualization_msgs::Marker::ADD;
    c2_dem.type = visualization_msgs::Marker::CYLINDER;
    c2_dem.lifetime = ros::Duration(1);
    c2_dem.id = 100023;

    c2_dem.color.a = 0.6;
    c2_dem.color.r = 0.0;
    c2_dem.color.g = 0.0;
    c2_dem.color.b = 1.0;

    c2_dem.scale.x = 2*c2.r_;
    c2_dem.scale.y = 2*c2.r_;
    c2_dem.scale.z = reach_lib::Point::norm(c2.p2_, c2.p1_);

    reach_lib::Point origin2 = reach_lib::Point::origin(c2.p2_, c2.p1_);
    geometry_msgs::Point pc2;
    pc2.x = origin2.x;
    pc2.y = origin2.y;
    pc2.z = origin2.z;
    c2_dem.pose.position = pc2;

    tf2::convert(Visualizer::orientation(c2.p1_, c2.p2_), c2_dem.pose.orientation);

    capdem.markers.push_back(s21_dem);
    capdem.markers.push_back(s22_dem);
    capdem.markers.push_back(c2_dem);

    // pub_vis.publish(capdem);

    reach_lib::Point X(1, 2, 3);
    reach_lib::Point Y(1, 6, 4);
    reach_lib::Point Z(5, 7, 3);

    double det = reach_lib::Point::determinant3x3(X, Y, Z);
    std::cout << "det: " << det << "\n";


*/
