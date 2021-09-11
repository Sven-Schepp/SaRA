#include <ros/ros.h>
#include <string>
#include <typeinfo>

#include "reach_lib.hpp"


int main(int argc, char** argv) {
  ros::init(argc, argv, "reachable_occupancy_main");
  ros::NodeHandle nh;
  ros::Rate loop_rate(60);


  while (ros::ok()) {
    reach_lib::ArticulatedAccel a;



    auto& b = typeid(a);
    std::cout << b.name();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

