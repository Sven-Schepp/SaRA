/*
This file is part of SaRA.

SaRA is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
TUM, either version 3 of the License, or
(at your option) any later version.

SaRA is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details: https://www.gnu.org/licenses/.
*/

#include <string>
#include <vector>

#include "cylinder.hpp"
#include "cylinder_perimeter.hpp"
#include "pedestrian.hpp"
#include "point.hpp"
#include "obstacle.hpp"
#include "system.hpp"

#ifndef REACH_LIB_INCLUDE_PEDESTRIAN_ACCEL_HPP_
#define REACH_LIB_INCLUDE_PEDESTRIAN_ACCEL_HPP_

namespace obstacles {
namespace pedestrian {
namespace accel {

//! \brief Defines a cylindrical safety perimeter based on dynamic extensions
//! to static aprroaches (ISO10218: https://www.iso.org/standard/51330.html)
//! described by http://mediatum.ub.tum.de/doc/1379662/905746.pdf.
//! Encorporates live velocity measurements and constant maximum acceleration
//! into occupancy calculations.
class PedestrianAccel : public Pedestrian {
 public:
  //! \brief Empty constructor
  PedestrianAccel() : Pedestrian() {}

  //! \brief Instantiates a cylindrical safety perimeter
  //!        based on constant maximum acceleration
  //!        and live velocity values.
  //! \param[in] max_a Maximum acceleration [m/s] (constant)
  PedestrianAccel(const System& sensor, double height,
                  double arm_span, double max_a, double offset = 0.0,
                  const Point& init_pos = Point(),
                  int steps = 0);

  //! \brief Calcualtes the current occupancy using this model
  //! \param[in] p Current location of points of interest in Cartesian coordinartes (x, y ,z)
  //! \param[in] v Current velocity of points of interest [static velocity is used here]
  //! \param[in] t_a Start of the interval of analysis
  //! \param[in] t_b End of the interval of analysis
  std::vector<CylinderPerimeter> update(double t_a, double t_b,
                                     std::vector<Point> p,
                                     std::vector<Point> v);

  //! \brief Determines if any point from the list 'targets' is located inside
  //!        the current occupancy.
  //! \param[in] targets A list of points of interest to be checked against
  //!                    the current occupancy.
  bool intersection(std::vector<Point> targets) const;

  //! \brief Returns the mode of reachability analysis
  //!        of this class as 'PEDESTRIAN-ACCEL'
  std::string get_mode() const {
      return "PEDESTRIAN-ACCEL";
  }

  //! \brief Returns the maximum acceleration parameter
  double get_max_a() const {
    return max_a_;
  }

  //! \brief Emtpy destructor
  ~PedestrianAccel() {}

 private:
  //! \brief Maximum acceleration of the human (constant)
  double max_a_ = 0.0;

  //! \brief The number of cylinders formed from intermediate intervals
  int steps_ = 0.0;

  //! \brief A list of intermediary occupacies
  std::vector<Cylinder> cylinder_list_ = {};
};
}  // namespace accel
}  // namespace pedestrian
}  // namespace obstacles
#endif  //  REACH_LIB_INCLUDE_PEDESTRIAN_ACCEL_HPP_
