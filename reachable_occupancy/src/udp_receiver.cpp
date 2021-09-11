#include <ros/ros.h>

#include "udp_transiever.cpp"

#include "udp_com/UdpPacket.h"
#include "udp_com/UdpSend.h"
#include "udp_com/UdpSocket.h"
#include "visualization_msgs/MarkerArray.h"

// Reception on port:
#define PORT	 8079
#define MAXLINE 1024


bool create_sock(ros::NodeHandle nh, std::string srcAddr = "192.168.7.13",
                   std::string destAddr = "192.168.7.1", uint16_t port = 8079,
                  bool isMulicast = false) {
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

void udp_callback(udp_com::UdpPacket data) {
  int num_joints = 7;  
  std::vector<float> positions;
  for (int i = 0; i < num_joints; i++) {
    positions.push_back(*reinterpret_cast<float*>(&data.data[i*4+12]));
  }
  std::cout << "Got sth:  " << positions[0] << " "
                            << positions[1] << " "
                            << positions[2] << " "
                            << positions[3] << " "
                            << positions[4] << " "
                            << positions[5] << "\n";
}

int main(int argc, char** argv) {
  // ROS system
  ros::init(argc, argv, "udp_receiver");
  ros::NodeHandle nh;
  ros::NodeHandle inet_h("eno2");
  ros::Rate loop_rate(60);

  // UDP receiver setup
  udp_transiever::UDPTransiever tsv = udp_transiever::UDPTransiever(6);


  bool rosbag = false;
  if (!rosbag) {
    if (!create_sock(inet_h)) {
      ROS_WARN("Socket Not Connected!");
    }
  }
  std::cout << "\n\nSocket created? " << "\n\n";

  // subscribe to the socket's topic
  // ros::Subscriber frame_data_subscriber = inet_h.subscribe("udp/p8080", 1000, udp_callback);
  ros::Subscriber frame_data_subscriber = inet_h.subscribe("udp/p8080", 1000,
                                          &udp_transiever::UDPTransiever::udp_callback, &tsv);
  
  ros::Publisher vis_publisher = inet_h.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array",1000);



  // ROS loop
  while (ros::ok()) {

    // Visualize points
    visualization_msgs::Marker points_in = visualization_msgs::Marker();

    points_in.header.frame_id = "/map";
    points_in.header.stamp = ros::Time::now();
    points_in.ns = "points_int";
    points_in.color.r = 1.0;
    points_in.color.a = 1.0;
    points_in.action = visualization_msgs::Marker::ADD;
    points_in.type = visualization_msgs::Marker::POINTS;
    points_in.lifetime = ros::Duration(1);

    points_in.id = 1111;
    points_in.scale.x = 0.1;
    points_in.scale.y = 0.1;

    for (int i = 0; i < tsv.get_jlMSG().positions.size(); i++)  {
      points_in.points.push_back(tsv.get_jlMSG().positions[i]);
    }
    visualization_msgs::MarkerArray marr;
    marr.markers.push_back(points_in);


    //std::cout << tsv.get_joint_pos().size() << "\n";

    vis_publisher.publish(marr);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}








/*
//
// client.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//


#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 2)
    {
      std::cerr << "Usage: client <host>" << std::endl;
      return 1;
    }

    boost::asio::io_service io_service;

    udp::resolver resolver(io_service);
    udp::resolver::query query(udp::v4(), argv[1], "daytime");
    udp::endpoint receiver_endpoint = *resolver.resolve(query);

    udp::socket socket(io_service);
    socket.open(udp::v4());

    boost::array<char, 1> send_buf  = {{ 0 }};
    socket.send_to(boost::asio::buffer(send_buf), receiver_endpoint);

    boost::array<char, 128> recv_buf;
    udp::endpoint sender_endpoint;
    size_t len = socket.receive_from(
        boost::asio::buffer(recv_buf), sender_endpoint);

    std::cout.write(recv_buf.data(), len);
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
*/
