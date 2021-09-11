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

#include <chrono>

#include "visualizer.cpp"
#include "receiver.cpp"
#include "volume.cpp"
#include "Articulated.hpp"
#include "Pedestrian.hpp"

typedef receiver::Receiver Receiver;
typedef visualizer::Visualizer Visualizer;
typedef volume::Volume Volume;

int main(int argc, char **argv) {
  ///< Standard node intiialization
  ros::init(argc, argv, "reachable_occupancy_main");
  ros::NodeHandle nh;
  ros::Rate loop_rate(60);

  // timer
  long double running_sum = 0.0;
  double loop_counter = 1.0;
  int longest = 0.0;
  int shortest = 1E9;
  bool timing = false;



  // dummy values ACCEL and VEL
  std::vector<double> max_a = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 20.0, 30.0, 30.0, 50.0,
                              30.0, 30.0, 50.0, 25.0};  // had one more 20.0 after 50.0
  std::vector<double> thickness = {0.1, 0.1, 0.205, 0.1, 0.1, 0.205, 0.3, 0.2, 0.2, 0.4, 0.2, 0.2, 0.4, 0.3};
  // !! map is sorted alphabetically -> thickness is not applied correctly
  std::vector<double> max_v;
  for (int i = 0; i < 15; i++) {
    max_v.push_back(2.46);  // was 14.0 alternatively 1.6
  }
  System s = System(0.002, 0.02);

  std::vector<std::tuple<std::string, int, int>> index;

  // Right arm init
  index.push_back(std::make_tuple("RightUpperArm", 0, 1));
  index.push_back(std::make_tuple("RightForearm", 1, 2));
  index.push_back(std::make_tuple("RightHand", 2 , 2));

  // Left arm init
  index.push_back(std::make_tuple("LeftUpperArm", 3, 4));
  index.push_back(std::make_tuple("LeftForearm", 4, 5));
  index.push_back(std::make_tuple("LeftHand", 5, 5));

  // Torso init
  index.push_back(std::make_tuple("Torso", 6, 7));

  // Right leg init
  index.push_back(std::make_tuple("RightThigh", 8, 9));
  index.push_back(std::make_tuple("RightShin", 9, 10));
  index.push_back(std::make_tuple("RightFoot", 10, 10));

  // Left leg init
  index.push_back(std::make_tuple("LefttThigh", 11, 12));
  index.push_back(std::make_tuple("LefttShin", 12, 13));
  index.push_back(std::make_tuple("LefttFoot", 13, 13));

  // Head init
  index.push_back(std::make_tuple("Head", 14, 14));



  // dummy values POS
  std::vector<double> max_v_p = {2.46, 2.46, 2.46, 2.46};
  std::vector<double> thickness_p = {0.205, 0.205, 0.3, 0.3};
  std::vector<double> lengths = {0.635, 0.635, 0.81, 0.81};

  std::vector<std::tuple<std::string, int>> index_p;
  std::vector<std::tuple<std::string, int, int, int>> index_p_inf;

  index_p.push_back(std::make_tuple("RightArm", 0));
  index_p.push_back(std::make_tuple("LeftArm", 3));
  index_p.push_back(std::make_tuple("RightLeg", 8));
  index_p.push_back(std::make_tuple("LeftLeg", 11));

  index_p_inf.push_back(std::make_tuple("RightArm", 0, 1, 2));
  index_p_inf.push_back(std::make_tuple("LeftArm", 3, 4, 5));
  index_p_inf.push_back(std::make_tuple("RightLeg", 8, 9, 10));
  index_p_inf.push_back(std::make_tuple("LeftLeg", 11, 12, 13));


  // Setting up test models
  // Articulated models
  Articulated human_a = Articulated(s, index, thickness, "ACCEL", max_v, max_a);
  Articulated human_v = Articulated(s, index, thickness, "VEL", max_v, max_a);
  // Articulated human_p = Articulated(s, index_p, thickness_p, max_v_p, lengths);
  Articulated human_p = Articulated(s, index_p_inf, thickness_p, max_v_p);

  // Pedestrian models
  Pedestrian ped = Pedestrian(s, 2.0, 2.0, 4.0, 20.0);

  ros::Publisher pub_vis = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1000);

  ros::Subscriber sub_j, sub_t;

  Receiver r = Receiver();

  sub_j = nh.subscribe("/sk_points", 1000, &Receiver::callback, &r);
  sub_t = nh.subscribe("/delta_t", 1000, &Receiver::callback_time, &r);

  // Visualizers
  Visualizer v_p = Visualizer();
  v_p.set_id_map(human_p, 1);
  v_p.color.g = 1.0;
  v_p.color.b = 0.0;
  v_p.color.a = 0.2;

  Visualizer v_a = Visualizer();
  v_a.set_id_map(human_a, 2);
  v_a.color.b = 1.0;
  v_a.color.r = 0.0;
  v_a.color.a = 0.4;

  Visualizer v_v = Visualizer();
  v_v.set_id_map(human_v, 3);
  v_v.color.r = 0.0;
  v_v.color.b = 1.0;
  v_v.color.a = 0.4;

  Point o = Point(-0.9, 0.0, 0.0);

  Volume vol = Volume(3.5, 4.0, 3.0, 0.1, o);

  for (auto it : v_a.id_map) {
    std::cout << std::get<0>(it) << " " << std::get<1>(it) << "\n";
  }
  for (auto it : human_a.get_index()) {
    std::cout << std::get<0>(it) << " " << std::get<1>(it) << "\n";
  }
  // std::cout<<human.get_body().size()<<"\n";

  while (ros::ok()) {
    // loop timer
    auto t1 = std::chrono::high_resolution_clock::now();


    visualization_msgs::MarkerArray M;

    visualization_msgs::Marker Joint_list = visualization_msgs::Marker();

    Joint_list.header.frame_id = "/map";
    Joint_list.header.stamp = ros::Time::now();
    Joint_list.ns = "skeleton";
    Joint_list.color.r = 1.0;
    Joint_list.color.a = 1.0;
    Joint_list.action = visualization_msgs::Marker::ADD;
    Joint_list.type = visualization_msgs::Marker::POINTS;
    Joint_list.lifetime = ros::Duration(0.1);

    Joint_list.id = 123458;
    Joint_list.scale.x = 0.08;
    Joint_list.scale.y = 0.08;

    visualization_msgs::Marker points_in = visualization_msgs::Marker();

    points_in.header.frame_id = "/map";
    points_in.header.stamp = ros::Time::now();
    points_in.ns = "points_int";
    points_in.color.r = 1.0;
    points_in.color.a = 1.0;
    points_in.action = visualization_msgs::Marker::ADD;
    points_in.type = visualization_msgs::Marker::POINTS;
    points_in.lifetime = ros::Duration(0.1);

    points_in.id = 1111;
    points_in.scale.x = 0.03;
    points_in.scale.y = 0.03;

    visualization_msgs::Marker points_out = visualization_msgs::Marker();

    points_out.header.frame_id = "/map";
    points_out.header.stamp = ros::Time::now();
    points_out.ns = "points_out";
    points_out.color.g = 1.0;
    points_out.color.a = 1.0;
    points_out.action = visualization_msgs::Marker::ADD;
    points_out.type = visualization_msgs::Marker::POINTS;
    points_out.lifetime = ros::Duration(0.1);

    points_out.id = 2222;
    points_out.scale.x = 0.015;
    points_out.scale.y = 0.015;


    if (r.stop) {
      // std::cout << "The Simulation has finished!" << std::endl;
      // return 0;

      vol.volume_routine(human_a);

      for (auto it : vol.points_in) {
        geometry_msgs::Point p;
        p.x = it.x;
        p.y = it.y;
        p.z = it.z;
        points_in.points.push_back(p);
      }
      for (auto it2 : vol.points_out) {
        geometry_msgs::Point p;
        p.x = it2.x;
        p.y = it2.y;
        p.z = it2.z;
        points_out.points.push_back(p);
      }
    }

    // call update on human with data from this cycle within Receiver
    // opt: get distance data from some topic and adjust the color gradient
    // call visualizer with data from human
    if (r.joints.size() != 0) {
      human_p.update(r.joints, r.velocities, r.t_start, r.t_end);
      human_a.update(r.joints, r.velocities, r.t_start, r.t_end);
      human_v.update(r.joints, r.velocities, r.t_start, r.t_end);
      v_p.vis_articulated(human_p);
      v_a.vis_articulated(human_a);
      v_v.vis_articulated(human_v);
    }

    for (auto const& it : r.joints) {
      geometry_msgs::Point p;
      p.x = it.x;
      p.y = it.y;
      p.z = it.z;
      Joint_list.points.push_back(p);
    }
    M.markers.push_back(Joint_list);
    M.markers.push_back(points_in);
    M.markers.push_back(points_out);


    // pub_vis.publish(v_p.markers);
    pub_vis.publish(v_a.markers);
    // pub_vis.publish(v_v.markers);
    pub_vis.publish(M);

    // loop timer
    auto t2 = std::chrono::high_resolution_clock::now();
    auto d = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    running_sum = running_sum + d;
    if (d < shortest && d > 150) {
      shortest = d;
    }
    if (d > longest) {
      longest = d;
    }
    loop_counter++;
    if (timing) {
      std::cout << running_sum/loop_counter << " " << longest << " " << shortest << "\n";
    }


    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
