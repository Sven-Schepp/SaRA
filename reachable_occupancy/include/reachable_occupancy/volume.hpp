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

#include <vector>

#include "reach_lib.hpp"

#ifndef REACHABLE_OCCUPANCY_VOLUME_HPP_
#define REACHABLE_OCCUPANCY_VOLUME_HPP_

namespace volume {

//! \brief This class estimates the volume of reachable occupancies
//! by generating an even grid of points in 3D Cartesian space and
//! determining whether they lie within the occupancy.
class Volume{
 public:
  //! \brief The sum of all individual occupancy volumes.
  double total_volume;

  //! \brief The points inside the occupancy.
  std::vector<reach_lib::Point> points_in;

  //! \brief The points outside the occupancy.
  std::vector<reach_lib::Point> points_out;

  //! \brief Emtpy constructor.
  Volume();

  //! \brief Initializes a Volume object that estimates the volume of the current occupancy.
  //! \param[in] x_dim Dimension of the point grid in x direction.
  //! \param[in] y_dim Dimension of the point grid in y direction.
  //! \param[in] z_dim Dimension of the point grid in z direction.
  //! \param[in] res Resolution of the point grid in all individual directions.
  //! \param[in] origin Origin of the point grid.
  Volume(double x_dim, double y_dim, double z_dim, double res, reach_lib::Point origin = reach_lib::Point());

  //! \brief Empty destructor.
  ~Volume() {}

  //! \brief Estimates the volume of the current ArticulatedAccel occupancy.
  //! \param[in] human Current state of the ArticulatedAccel ocupancy.
  double volume_routine(reach_lib::ArticulatedAccel human);

  //! \brief Estimates the volume of the current ArticulatedVel occupancy.
  //! \param[in] human Current state of the ArticulatedVel ocupancy.
  double volume_routine(reach_lib::ArticulatedVel human);

  //! \brief Estimates the volume of the current ArticulatedPos occupancy.
  //! \param[in] human Current state of the ArticulatedPos ocupancy.
  double volume_routine(reach_lib::ArticulatedPos human);

  //! \brief Estimates the volume of the current PedestrianAccel occupancy.
  //! \param[in] human Current state of the PedestrianAccel ocupancy.
  double volume_routine(reach_lib::PedestrianAccel human);

  //! \brief Estimates the volume of the current PedestrianVel occupancy.
  //! \param[in] human Current state of the PedestrianVel ocupancy.
  double volume_routine(reach_lib::PedestrianVel human);

  //! \brief Returns the current grid
  inline std::vector<reach_lib::Point> get_grid() {
    return this->grid;
  }

  //! \brief Sets the current grid
  inline void set_grid(std::vector<reach_lib::Point> grd) {
    this->grid = grd;
  }

  //! \brief Returns th current origin of the grid
  inline reach_lib::Point get_origin() {
    return this->origin;
  }

  //! \brief Sets th current origin of the grid
  inline void set_origin(reach_lib::Point origin) {
    this->origin = origin;
  }

  //! \brief Returns the current volume
  inline double get_volume() {
    return this->volume;
  }

 private:
  //! \brief Used to check which points lie within the occupancy.
  std::vector<reach_lib::Point> grid;

  //! \brief Origin of the point grid.
  reach_lib::Point origin;

  //! \brief Current volume of the occupancy.
  double volume;

  //! \brief Total number of points with the grid.
  int num_points;
  //! \brief Dimension of the point grid in x direction.
  double x_dim;

  //! \brief Dimension of the point grid in y direction.
  double y_dim;

  //! \brief Dimension of the point grid in z direction.
  double z_dim;

  //! \brief Resolution of the point grid in all individual directions.
  double res;

  //! \brief Generates the grid used for volume estimation
  void generate_grid();

  //! \brief Calculates the volume of any Articulated model
  //! once a grid has beed generated.
  void calculate(std::vector<reach_lib::Capsule> capsules);

  //! \brief Calculates the volume of any Pedestrian model
  //! once a grid has beed generated.
  void calculate(std::vector<reach_lib::Cylinder> cylinders);
};
}  // namespace volume
#endif  //  REACHABLE_OCCUPANCY_VOLUME_HPP_
