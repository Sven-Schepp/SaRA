/*
This file is part of Reach-RI.

Reach-RI is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
TUM, either version 3 of the License, or
(at your option) any later version.

Reach-RI is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details: https://www.gnu.org/licenses/.
*/

#include <string>
#include <vector>

#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "articulated.hpp"
#include "articulated_accel.hpp"
#include "articulated_pos.hpp"
#include "articulated_vel.hpp"
#include "capsule.hpp"
#include "cylinder.hpp"
#include "pedestrian.hpp"
#include "pedestrian_accel.hpp"
#include "pedestrian_vel.hpp"
#include "obstacle.hpp"
#include "occupancy.hpp"
#include "visualizer.hpp"

namespace visualizer {

Visualizer::Visualizer(std::string mode) {
  this->frame_id_ = "/map";
  std_msgs::ColorRGBA color;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 1.0;
  color.a = 0.3;
  std_msgs::ColorRGBA start;
  start.r = 1.0;
  start.g = 0.0;
  start.b = 0.0;
  start.a = 0.6;
  std_msgs::ColorRGBA mid;
  mid.r = 1.0;
  mid.g = 1.0;
  mid.b = 0.0;
  mid.a = 0.6;
  std_msgs::ColorRGBA end;
  end.r = 34.0/255.0;
  end.g = 139.0/255.0;
  end.b = 34.0/255.0;
  end.a = 0.6;

  this->color_ = color;
  this->c_params_ = std::make_tuple(1.5, 10.0);
  this->start_ = color;
  this->end_ = end;
  this->mode_ = "ARTICULATED-ACCEL";
}

Visualizer::Visualizer(Articulated human, std_msgs::ColorRGBA color,
                       std::string mode, std::string frame_id, int id_domain) {
  this->color_ = color;
  this->frame_id_ = frame_id;
  this->mode_ = mode;
  this->id_domain_ = id_domain;

  for (auto const& it : human.get_body_segment_map()) {
      this->id_map_.push_back(std::make_tuple(it.first, 1000 * id_domain + 10*it.second.first));
  }
}

Visualizer::Visualizer(Pedestrian ped, std_msgs::ColorRGBA color,
                       std::string mode, std::string frame_id) {
  this->color_ = color;
  this->frame_id_ = frame_id;
  this->mode_ = mode;

  int domain = 100000;
  this->id_map_.push_back(std::make_tuple(ped.get_occupancy()[0].get_name(), domain));

  /* No suboccupancies allowed for now
  for (int i = 0; i <= ped.cylinder_list.size(); i++) {
      this->id_map_.push_back(std::make_tuple(std::to_string(i), domain+i));
  }*/
}

void Visualizer::set_id_map(Articulated* human_p, int id_domain) {
  for (auto const& it : (*human_p).get_body_segment_map()) {
      this->id_map_.push_back(std::make_tuple(it.first, 1000 * id_domain + 10*it.second.first));
  }
}

void Visualizer::set_id_map(Pedestrian* ped_p, int id_domain) {
  this->id_map_.push_back(std::make_tuple((*ped_p).get_occupancy()[0].get_name(), id_domain*1000));
}

void Visualizer::set_id_map(int num_caps, int id_domain) {
  for (int i = 0; i < num_caps; i++) {
    this->id_map_.push_back(std::make_tuple(std::to_string(i * 10 + id_domain * 1000), i * 10 + id_domain * 1000));
  }
  /*int count = 0;
  for (const auto it : caps) {
    if (it.p1_ == it.p2_) {
      this->id_map_.push_back(std::make_tuple(std::to_string(id_domain*1000 + 10*count), id_domain*1000 + 10*count));
    } else {
      this->id_map_.push_back(std::make_tuple(std::to_string(id_domain*1000 + 10*count), id_domain*1000 + 10*count));
    }
    count ++;
  }*/
}

void Visualizer::set_frame_id(std::string frame_id) {
    this->frame_id_ = frame_id;
}

tf2::Quaternion Visualizer::orientation(Point p1, Point p2) {
  if (p2.z < p1.z) {
      Point temp = p1;
      p1 = p2;
      p2 = temp;
  }

  double dis = Point::norm(p1, p2);
  double f = 0.0174533;
  double r = 0;
  double p = 0.0;
  double y = 0.0;

  if (dis == 0) {
      p = 0.0;
  } else {
      p = -asin((p2.y - p1.y)/dis);
  }

  if (cos(p) == 0 || dis == 0) {
  y = 0;
  } else {
  y = asin((p2.x-p1.x)/(cos(p)*dis));
  }
  tf2::Quaternion q;
  q.setRPY(p, y, r);  // formerly q.setRPY(r,p,Y);
  q.normalize();
  return q;
}

void Visualizer::vis_point_cloud(const std::vector<Point>& points, std::string name_space) {
  visualization_msgs::Marker marker = visualization_msgs::Marker();
  this->markers_.markers = {};
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = name_space;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.lifetime = ros::Duration(1);
  marker.color = this->color_;
  marker.id = this->id_domain_;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.points = {};
  for (int i = 0; i < points.size(); i++)  {
    geometry_msgs::Point p;
    p.x = points[i].x;
    p.y = points[i].y;
    p.z = points[i].z;
    marker.points.push_back(p);
  }
  this->markers_.markers.push_back(marker);
}

void Visualizer::vis_articulated(Articulated* human_p) {
  visualization_msgs::MarkerArray m;
  this->markers_ = m;
  if (this->mode_ == "ARTICULATED-POS") {
    // Dereferencing human pointer
    ArticulatedPos human = *dynamic_cast<ArticulatedPos*>(human_p);
    // Dereferencing occupancy pointers
    std::vector<occupancies::Occupancy*> ho_pointer = human.get_occupancy_p();
    std::vector<Extremity> bp;
    for (int i = 0; i < ho_pointer.size(); i++) {
      bp.push_back(*dynamic_cast<Extremity*>(ho_pointer[i]));
    }
    int count = 0;
    for (auto it : bp) {
      Capsule oc = it.get_occupancy();
      visualization_msgs::Marker S;
      S.header.frame_id = this->frame_id_;
      S.header.stamp = ros::Time::now();
      S.ns = it.get_name();
      S.color = this->color_;
      S.action = visualization_msgs::Marker::ADD;
      S.id = std::get<1>(this->id_map_[count]);
      S.type = visualization_msgs::Marker::SPHERE;
      S.lifetime = ros::Duration(1.0);
      S.scale.x = 2*oc.r_;
      S.scale.y = 2*oc.r_;
      S.scale.z = 2*oc.r_;

      geometry_msgs::Point p;
      p.x = oc.p1_.x;
      p.y = oc.p1_.y;
      p.z = oc.p1_.z;
      S.pose.position = p;
      S.pose.orientation = geometry_msgs::Quaternion();
      S.pose.orientation.w = 1.0;
      this->markers_.markers.push_back(S);
      count++;
    }
  } else {
    if (!(this->mode_ == "ARTICULATED-ACCEL" || this->mode_ == "ARTICULATED-VEL")) {
      throw "Visualization called on type different from initialization!";
    }
    // Dereferencing occupancy pointers
    Articulated human = *dynamic_cast<Articulated*>(human_p);
    // Dereference occupancy pointers
    std::vector<occupancies::Occupancy*> ho_pointer = human.get_occupancy_p();
    std::vector<BodyPart> bp;
    for (int i = 0; i < ho_pointer.size(); i++) {
      bp.push_back(*dynamic_cast<BodyPart*>(ho_pointer[i]));
    }

    int count = 0;
    for (auto it : bp) {
      Capsule oc = it.get_occupancy();

      // Ball detection
      if (oc.p1_ == oc.p2_) {
        visualization_msgs::Marker S;
        S.header.frame_id = this->frame_id_;
        S.header.stamp = ros::Time::now();
        S.ns = it.get_name();
        S.color = this->color_;
        S.action = visualization_msgs::Marker::ADD;
        S.id = std::get<1>(this->id_map_[count]);
        S.type = visualization_msgs::Marker::SPHERE;
        S.lifetime = ros::Duration(0.2);
        S.scale.x = 2*oc.r_;
        S.scale.y = 2*oc.r_;
        S.scale.z = 2*oc.r_;

        geometry_msgs::Point p;
        p.x = oc.p1_.x;
        p.y = oc.p1_.y;
        p.z = oc.p1_.z;
        S.pose.position = p;
        S.pose.orientation = geometry_msgs::Quaternion();
        S.pose.orientation.w = 1.0;
        this->markers_.markers.push_back(S);
      } else {
        visualization_msgs::Marker S1;
        S1.header.frame_id = this->frame_id_;
        S1.header.stamp = ros::Time::now();
        S1.ns = it.get_name();
        S1.color = this->color_;
        S1.action = visualization_msgs::Marker::ADD;
        S1.id = std::get<1>(this->id_map_[count]);
        S1.type = visualization_msgs::Marker::SPHERE;
        S1.lifetime = ros::Duration(1.0);
        S1.scale.x = 2*oc.r_;
        S1.scale.y = 2*oc.r_;
        S1.scale.z = 2*oc.r_;

        geometry_msgs::Point pa;
        pa.x = oc.p1_.x;
        pa.y = oc.p1_.y;
        pa.z = oc.p1_.z;
        S1.pose.position = pa;
        S1.pose.orientation = geometry_msgs::Quaternion();
        S1.pose.orientation.w = 1.0;
        this->markers_.markers.push_back(S1);

        visualization_msgs::Marker S2;
        S2.header.frame_id = this->frame_id_;
        S2.header.stamp = ros::Time::now();
        S2.ns = it.get_name();
        S2.color = this->color_;
        S2.action = visualization_msgs::Marker::ADD;
        S2.id = std::get<1>(this->id_map_[count]) + 1;
        S2.type = visualization_msgs::Marker::SPHERE;
        S2.lifetime = ros::Duration(1.0);
        S2.scale.x = 2*oc.r_;
        S2.scale.y = 2*oc.r_;
        S2.scale.z = 2*oc.r_;

        geometry_msgs::Point pb;
        pb.x = oc.p2_.x;
        pb.y = oc.p2_.y;
        pb.z = oc.p2_.z;
        S2.pose.position = pb;
        S2.pose.orientation = geometry_msgs::Quaternion();
        S2.pose.orientation.w = 1.0;
        this->markers_.markers.push_back(S2);

        visualization_msgs::Marker C;
        C.header.frame_id = this->frame_id_;
        C.header.stamp = ros::Time::now();
        C.ns = it.get_name();
        C.color = this->color_;
        C.action = visualization_msgs::Marker::ADD;
        C.id = std::get<1>(this->id_map_[count]) + 2;
        C.type = visualization_msgs::Marker::CYLINDER;
        C.lifetime = ros::Duration(1.0);
        C.scale.x = 2*oc.r_;
        C.scale.y = 2*oc.r_;
        C.scale.z = Point::norm(oc.p2_, oc.p1_);

        Point origin = Point::origin(oc.p1_, oc.p2_);
        geometry_msgs::Point pc;
        pc.x = origin.x;
        pc.y = origin.y;
        pc.z = origin.z;
        C.pose.position = pc;

        tf2::convert(Visualizer::orientation(oc.p1_, oc.p2_), C.pose.orientation);
        this->markers_.markers.push_back(C);
      }
      count++;
    }
  }
}

void Visualizer::vis_capsules(std::vector<Capsule> capsules, std::vector<bool> intersections) {
  this->markers_.markers = {};
  int count = 0;

  for (auto oc : capsules) {

    // Ball detection
    if (oc.p1_ == oc.p2_) {
      visualization_msgs::Marker S;
      S.header.frame_id = this->frame_id_;
      S.header.stamp = ros::Time::now();
      S.ns = "S: "+std::to_string(std::get<1>(this->id_map_[count]));
      
      if (intersections.size() > 0 && intersections[count] == true) {
        S.color.a = this->color_.a;
        S.color.r = 1.0;
        S.color.g = 0.0;
        S.color.b = 0.0;
      } else {
        S.color = this->color_;
      }
      S.action = visualization_msgs::Marker::ADD;
      S.id = std::get<1>(this->id_map_[count]);
      S.type = visualization_msgs::Marker::SPHERE;
      S.lifetime = ros::Duration(0.2);
      S.scale.x = 2*oc.r_;
      S.scale.y = 2*oc.r_;
      S.scale.z = 2*oc.r_;

      geometry_msgs::Point p;
      p.x = oc.p1_.x;
      p.y = oc.p1_.y;
      p.z = oc.p1_.z;
      S.pose.position = p;
      S.pose.orientation = geometry_msgs::Quaternion();
      S.pose.orientation.w = 1.0;
      this->markers_.markers.push_back(S);
    } else {
      visualization_msgs::Marker S1;
      S1.header.frame_id = this->frame_id_;
      S1.header.stamp = ros::Time::now();
      S1.ns = "S1: "+std::to_string(std::get<1>(this->id_map_[count]));
      if (intersections.size() > 0 && intersections[count] == true) {
        S1.color.a = this->color_.a;
        S1.color.r = 1.0;
        S1.color.g = 0.0;
        S1.color.b = 0.0;
      } else {
        S1.color = this->color_;
      }
      S1.action = visualization_msgs::Marker::ADD;
      S1.id = std::get<1>(this->id_map_[count]);
      S1.type = visualization_msgs::Marker::SPHERE;
      S1.lifetime = ros::Duration(1.0);
      S1.scale.x = 2*oc.r_;
      S1.scale.y = 2*oc.r_;
      S1.scale.z = 2*oc.r_;

      geometry_msgs::Point pa;
      pa.x = oc.p1_.x;
      pa.y = oc.p1_.y;
      pa.z = oc.p1_.z;
      S1.pose.position = pa;
      S1.pose.orientation = geometry_msgs::Quaternion();
      S1.pose.orientation.w = 1.0;
      this->markers_.markers.push_back(S1);

      visualization_msgs::Marker S2;
      S2.header.frame_id = this->frame_id_;
      S2.header.stamp = ros::Time::now();
      S2.ns = "S2: "+std::to_string(std::get<1>(this->id_map_[count]));
      if (intersections.size() > 0 && intersections[count] == true) {
        S2.color.a = this->color_.a;
        S2.color.r = 1.0;
        S2.color.g = 0.0;
        S2.color.b = 0.0;
      } else {
        S2.color = this->color_;
      }
      S2.action = visualization_msgs::Marker::ADD;
      S2.id = std::get<1>(this->id_map_[count]) + 1;
      S2.type = visualization_msgs::Marker::SPHERE;
      S2.lifetime = ros::Duration(1.0);
      S2.scale.x = 2*oc.r_;
      S2.scale.y = 2*oc.r_;
      S2.scale.z = 2*oc.r_;

      geometry_msgs::Point pb;
      pb.x = oc.p2_.x;
      pb.y = oc.p2_.y;
      pb.z = oc.p2_.z;
      S2.pose.position = pb;
      S2.pose.orientation = geometry_msgs::Quaternion();
      S2.pose.orientation.w = 1.0;
      this->markers_.markers.push_back(S2);

      visualization_msgs::Marker C;
      C.header.frame_id = this->frame_id_;
      C.header.stamp = ros::Time::now();
      C.ns = "C: "+std::to_string(std::get<1>(this->id_map_[count]));
      if (intersections.size() > 0 && intersections[count] == true) {
        C.color.a = this->color_.a;
        C.color.r = 1.0;
        C.color.g = 0.0;
        C.color.b = 0.0;
      } else {
        C.color = this->color_;
      }
      C.action = visualization_msgs::Marker::ADD;
      C.id = std::get<1>(this->id_map_[count]) + 2;
      C.type = visualization_msgs::Marker::CYLINDER;
      C.lifetime = ros::Duration(1.0);
      C.scale.x = 2*oc.r_;
      C.scale.y = 2*oc.r_;
      C.scale.z = Point::norm(oc.p2_, oc.p1_);

      Point origin = Point::origin(oc.p1_, oc.p2_);
      geometry_msgs::Point pc;
      pc.x = origin.x;
      pc.y = origin.y;
      pc.z = origin.z;
      C.pose.position = pc;

      tf2::convert(Visualizer::orientation(oc.p1_, oc.p2_), C.pose.orientation);
      this->markers_.markers.push_back(C);
    }
    count++;
  }
}

void Visualizer::vis_pedestrian(Pedestrian* ped_p, std::vector<bool> intersections) {
  // Get and cast the pointer to the current occupancy to cylinder
  // Dereference the pointer to get the current occupancy
  Pedestrian ped = *ped_p;
  Cylinder oc = ped.get_occupancy()[0].get_occupancy();

  visualization_msgs::MarkerArray m;
  this->markers_ = m;
  visualization_msgs::Marker S;
  S.header.frame_id = this->frame_id_;
  S.header.stamp = ros::Time::now();
  S.ns = "Maximum velocity based Cylinder";
  if (intersections.size() > 0 && intersections[0] == true) {
    S.color.a = this->color_.a;
    S.color.r = 1.0;
    S.color.g = 0.0;
    S.color.b = 0.0;
  } else {
    S.color = this->color_;
  }
  S.action = visualization_msgs::Marker::ADD;
  S.id = std::get<1>(this->id_map_[0]);
  S.type = visualization_msgs::Marker::CYLINDER;
  S.lifetime = ros::Duration(1.0);
  S.scale.x = 2*oc.r_;
  S.scale.y = 2*oc.r_;
  S.scale.z = ped.get_height();

  geometry_msgs::Point p;
  p.x = oc.p1_.x;
  p.y = oc.p1_.y;
  p.z = 0.5 * ped.get_height() + ped.get_offset();
  S.pose.position = p;
  S.pose.orientation = geometry_msgs::Quaternion();
  S.pose.orientation.w = 1.0;
  this->markers_.markers.push_back(S);
}

void Visualizer::alpha(double dist) {
  float a = 1/(pow(std::get<0>(this->c_params_), 3));
  this->color_.a = -a*pow(dist, 3) + 1;
}

void Visualizer::color_calc(double dist) {
  double interval = std::get<0>(this->c_params_)/std::get<1>(this->c_params_);

  for (int i = 0; i < std::get<1>(this->c_params_); i++) {  //  maybe use std::get<1>(this->c_params_)+1
    if (dist <= i*interval) {
      this->color_ = this->gradient[i];
      alpha(dist);
      return;
    }
  }
  this->color_ = this->gradient[this->gradient.size()-1];
  alpha(dist);
}

void Visualizer::color_gradient(std_msgs::ColorRGBA start, std_msgs::ColorRGBA end, int granularity) {
  for (int i = 0; i < this->gradient.size(); i++) {
      std_msgs::ColorRGBA c;  // maybe divide all color values by 255
      c.r = start.r + (static_cast<double>(i) / (granularity-1)) * (end.r - start.r);
      c.g = start.g + (static_cast<double>(i) / (granularity-1)) * (end.g - start.g);
      c.b = start.b + (static_cast<double>(i) / (granularity-1)) * (end.b - start.b);
  }
}
}  // namespace visualizer


// Demo python code for color gradients
/*
sh.coldis = (1.5,20)
sh.lin_gradient((255,0,0), (255,255,0), sh.coldis[1]/2)
sh.lin_gradient((255,255,0), (34,139,34), sh.coldis[1]/2)


global coldis(1.5,20) # max distance before 0 alpha and granularity
*/

/*
def alpha(dis):	
	# find alphafunction dependent on the max possible ditance use either x2 or x3
	global coldis
	a = 1/np.power(coldis[0], 3)
	return -a*np.power(dis, 3)+1

def color_calc(dis):
	global coldis
	global col_list
	global dist
	b = 0
	if not dist == dis:
		dist = dis
		b = 1
	interval = float(coldis[0])/coldis[1]
	
	for x in range(0, coldis[1]-1):
		if dis <= x*interval:
			return (col_list[x-1], alpha(dis))
	return (col_list[-1], alpha(dis))

def lin_gradient(start, end, n):
	global col_list
	s = start
	f = end
	
	if len(col_list) > 0:
		del col_list[-1]

	for x in range(0, n):
		curr_vec = []
		r = s[0] + (float(x)/(n-1))*(f[0]-s[0])
		g = s[1] + (float(x)/(n-1))*(f[1]-s[1])
		b = s[2] + (float(x)/(n-1))*(f[2]-s[2])
		curr_vec.append(r/255)
		curr_vec.append(g/255)
		curr_vec.append(b/255)
		
		col_list.append(curr_vec)
*/
