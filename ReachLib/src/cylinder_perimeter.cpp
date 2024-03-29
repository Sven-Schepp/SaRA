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
#include "occupancy.hpp"
#include "point.hpp"

namespace occupancies {
namespace cylinder_perimeters {

CylinderPerimeter::CylinderPerimeter() : Occupancy() {
  this->occupancy_p = &this->occupancy_;
}

CylinderPerimeter::CylinderPerimeter(const Point& init_pos, double height, double radius, std::string name) :
                                     Occupancy(name) {
  Cylinder c = Cylinder(init_pos, Point(init_pos.x, init_pos.y, height), radius);
  this->occupancy_ = c;
  this->cylinder_list_.push_back(c);
}

bool CylinderPerimeter::intersection(std::vector<Point> targets) const {
  return this->occupancy_.intersection(targets);
}
}  //  namespace cylinder_perimeters
}  //  namespace occupancies
