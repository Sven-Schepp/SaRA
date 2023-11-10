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

#include <tuple>
#include <vector>

#include "cylinder.hpp"
#include "occupancy_container.hpp"
#include "point.hpp"
#include "sphere.hpp"
#include "capsule.hpp"

#ifndef REACH_LIB_INCLUDE_AABB_HPP_
#define REACH_LIB_INCLUDE_AABB_HPP_

namespace occupancy_containers {
namespace aabb {

//! This class defines the axis-aligned bounding box occupancy medium
//! It is given as a box defined by its limits in x, y, and z direction.
class AABB : public OccupancyContainer {
 private:

 public:
  //! \brief Minimum in x, y, and z direction
  std::vector<double> min_ = {0.0, 0.0, 0.0};

  //! \brief Maximum in x, y, and z direction
  std::vector<double> max_ = {0.0, 0.0, 0.0};
  
  //! \brief Empty constructor
  AABB() : OccupancyContainer() {}

  //! \brief Constructs an AABB from given limits
  //! \param[in] min Minimum in x, y, and z direction
  //! \param[in] max Maximum in x, y, and z direction
  AABB(const std::vector<double>& min, const std::vector<double>& max) {
    min_ = min;
    max_ = max;
  }

  //! \brief Static cast constructor from AABB to OccupancyModels
  explicit AABB(const OccupancyContainer& o) {
    //  TODO
  }

  //! \brief Empty destructor
  ~AABB() {}

  //! \brief Returns if a point is inside the AABB
  //! \param[in] target Point of interest in Cartesian (x, y, z).
  //! \return True if the point is inside the AABB, false otherwise.
  inline bool intersection(const Point& target) {
    return (min_[0] <= target.x && target.x <= max_[0] &&
            min_[1] <= target.y && target.y <= max_[1] &&
            min_[2] <= target.z && target.z <= max_[2]);
  }

  //! \brief Determines whether the AABB intersects with
  //!        any point in points.
  //! \param[in] targets List of points of interest in Cartesian (x, y, z).
  //! \return True if any point is inside the AABB, false otherwise.
  inline bool intersection(const std::vector<Point>& targets) {
    for (const auto& target : targets) {
      if (intersection(target)) {
        return true;
      }
    }
    return false;
  }

};
}  //  namespace aabb
}  //  namespace occupancy_containers
#endif  //  REACH_LIB_INCLUDE_AABB_HPP_
