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
#include "point.hpp"
#include "obstacle.hpp"


#ifndef REACHABLE_OCCUPANCY_VISUALIZER_HPP_
#define REACHABLE_OCCUPANCY_VISUALIZER_HPP_


namespace visualizer {

typedef point::Point Point;

typedef occupancy_containers::capsule::Capsule Capsule;

typedef occupancy_containers::cylinder::Cylinder Cylinder;

typedef occupancies::body_parts::BodyPart BodyPart;

typedef occupancies::body_parts::accel::BodyPartAccel BodyPartAccel;

typedef occupancies::body_parts::vel::BodyPartVel BodyPartVel;

typedef occupancies::extremities::Extremity Extremity;

typedef obstacles::articulated::Articulated Articulated;

typedef obstacles::articulated::accel::ArticulatedAccel ArticulatedAccel;

typedef obstacles::articulated::vel::ArticulatedVel ArticulatedVel;

typedef obstacles::articulated::pos::ArticulatedPos ArticulatedPos;

typedef obstacles::pedestrian::Pedestrian Pedestrian;

typedef obstacles::pedestrian::accel::PedestrianAccel PedestrianAccel;

typedef obstacles::pedestrian::vel::PedestrianVel PedestrianVel;



class Visualizer{
 public:
  //! \brief The orientation of a capsule or cylinder in global coordinates.
  static tf2::Quaternion orientation(Point p1, Point p2);

  //! \brief A container for rviz markers of the current occupancies.
  visualization_msgs::MarkerArray markers_ = {};

  //! \brief The color of the current occupancy.
  std_msgs::ColorRGBA color_;

  //! \brief The frame id of the occupancy within rviz.
  std::string frame_id_ = "/map";

  //! \brief The decimal domain of the marker ids
  int id_domain_ = 0;

  //! \brief Defines the mode of visualization [ACCEL, VEL, POS]/[PEDESTRIAN, ARTICULATED]
  std::string mode_ = "UNKNOWN";

  //! \brief Staring point of the color gradient.
  std_msgs::ColorRGBA start_;

  //! \brief A midpoint for the gradient.
  std_msgs::ColorRGBA mid_;

  //! \brief Ending point of the color gradient.
  std_msgs::ColorRGBA end_;

  //! The size of the distance intervals using which the rgadient is calculated.
  int granularity_ = 0.02;

  //! \brief A map between capsule ids and capsule names for re-identification in rviz.
  std::vector<std::tuple<std::string, int>> id_map_ = {};

  //! \brief Empty constructor.
  explicit Visualizer(std::string mode_ = "ARTICULATED-ACCEL");

  //! \brief Instantiates a Visualizer object for the articulated models.
  //! \param[in] human The Articulated object whose occupancies are to be visualized.
  //! \param[in] color The color of the occupancy.
  //! \param[in] frame_id The frame_id used to assign a frame to the occupacy in rviz.
  Visualizer(Articulated human, std_msgs::ColorRGBA color,
             std::string mode, std::string frame_id = "/map",
             int id_domain = 0);

  //! \brief Instantiates a Visualizer object for the articulated models.
  //! \param[in] ped The Pedestrian object whose occupancies are to be visualized.
  //! \param[in] color The color of the occupancy.
  //! \param[in] frame_id The frame_id used to assign a frame to the occupacy in rviz.
  Visualizer(Pedestrian ped, std_msgs::ColorRGBA color,
             std::string mode, std::string frame_id = "/map");

  //! \brief Empty destructor
  ~Visualizer() {}

  //! \brief Sets the id_map_ parameter for articulated models.
  //! \param[in] human An object of any type of Articulated model
  //! \param[in] id_domain The rough decimal domain in which marker ids are placed
  void set_id_map(Articulated* human_p, int id_domain);

  //! \brief Sets the id_map_ parameter for pedestrian models.
  //! \param[in] human An object of any type of Articulated model
  //! \param[in] id_domain The rough decimal domain in which marker ids are placed
  void set_id_map(Pedestrian* ped_p, int id_domain);

  //! \brief Sets the id_map_ parameter for any array of capsules.
  //! \param[in] caps The length of the capsule vector to be visualized
  //! \param[in] id_domain The rough decimal domain in which marker ids are placed
  void set_id_map(int num_caps, int id_domain);

  //! \brief Sets the frame_id parameter.
  //! \param[in] frame_id The frame_id used to assign a frame to the occupacy in rviz.
  void set_frame_id(std::string frame_id);

  //! \brief Visualizes Articulated models
  //! \param[in] human The Articulated object whose occupancies are to be visualized.
  void vis_articulated(Articulated* human_p);

  //! \brief Visualizes any list of Capsules
  //! \param[in] capsules The list of Capsule objects that are to be visualized.
  void vis_capsules(std::vector<Capsule> capsules, std::vector<bool> intersections = {});

  //! \brief Visualizes Pedestrian models
  //! \param[in] ped The Pedestrian object whose occupancies are to be visualized.
  //! \param[in] only_desired_interval Does not visualize sub-occupancies if 'true'
  void vis_pedestrian(Pedestrian* ped_p, std::vector<bool> intersections = {});

  //! \brief Visualizes the joint positions (skeleton in Cartesian coordinates) as points
  //! \param[in] points A list of points to be visualized
  void vis_point_cloud(const std::vector<Point>& points, std::string name_space = "");

 private:
  //! \brief Contains the start and end interval of the color gradient
  std::tuple<double, double> c_params_;

  //! \brief Contains the color coding for the gradient
  std::vector<std_msgs::ColorRGBA> gradient;

  //! \brief Determines the occupancy's alpha value based on the distance to targets
  //! \param[in] dist Closest distance to any of the targets
  void alpha(double dist);

  //! \brief Calculates the occupancy's color
  //! based on the distance to any target and the color gradient
  //! \param[in] dist Closest distance to any of the targets
  void color_calc(double dist);

  //! \brief Calcualates a linear color gradient based on the start and end color
  //! \param[in] start The color that the gradient starts at
  //! \param[in] end The color that the gradient ends at
  void color_gradient(std_msgs::ColorRGBA start, std_msgs::ColorRGBA end, int granularity);
};
}  // namespace visualizer
#endif  // REACHABLE_OCCUPANCY_VISUALIZER_HPP_
