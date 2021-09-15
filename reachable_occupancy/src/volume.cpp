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

#include <vector>

#include "reach_lib.hpp"
#include "volume.hpp"

namespace volume {

Volume::Volume() {
  this->volume = 0.0;
  this->num_points = 0;
  this->x_dim = 0.0;
  this->y_dim = 0.0;
  this->z_dim = 0.0;
  this->res = 0.0;
  this->origin = reach_lib::Point();
}


Volume::Volume(double x_dim, double y_dim, double z_dim, double res, reach_lib::Point origin) {
  this->volume = 0.0;
  this->num_points = 0;
  this->x_dim = x_dim;
  this->y_dim = y_dim;
  this->z_dim = z_dim;
  this->res = res;
  this->origin = origin;
  this->generate_grid();
}

void Volume::generate_grid() {
  int x_lim = floor(this->x_dim/this->res);
  int y_lim = floor(this->y_dim/this->res);
  int z_lim = floor(this->z_dim/this->res);
  this->num_points = x_lim * y_lim * z_lim;
  this->total_volume = this->x_dim * this->y_dim * this->z_dim;
  this->grid = {};

  for (int i = 0; i < x_lim; i++) {
    for (int j = 0; j < y_lim; j++) {
      for (int k = 0; k < z_lim; k++) {
          this->grid.push_back(reach_lib::Point(this->origin.x+i*this->res,
                               this->origin.y+j*this->res,
                               this->origin.z+k*this->res));
      }
    }
  }
}

void Volume::calculate(std::vector<reach_lib::Capsule> capsules) {
  int count = 0;
  this->points_in = {};
  this->points_out = {};
  for (auto const &p : this->grid) {
    bool in = false;
    for (auto it : capsules) {
      if (it.intersection({p})) {
        count++;
        this->points_in.push_back(p);
        in = true;
        break;
      }
    }
    if (in == false) {
      this->points_out.push_back(p);
    }
  }

  this->volume = (static_cast<double>(count)/
                  static_cast<double>(this->num_points)) * this->total_volume;
}

void Volume::calculate(std::vector<reach_lib::Cylinder> cylinders) {
  int count = 0;
  this->points_in = {};
  this->points_out = {};
  for (auto const &p : this->grid) {
    bool in = false;
    for (auto it : cylinders) {
      if (it.intersection({p})) {
        count++;
        this->points_in.push_back(p);
        in = true;
        break;
      }
    }
    if (in == false) {
      this->points_out.push_back(p);
    }
  }

  this->volume = (static_cast<double>(count)/
                  static_cast<double>(this->num_points)) * this->total_volume;
}

double Volume::volume_routine(reach_lib::ArticulatedAccel human) {
  if (this->grid.size() == 0) {
    Volume::generate_grid();
  }
  std::vector<reach_lib::BodyPartAccel> occupancy = human.get_occupancy();
  std::vector<reach_lib::Capsule> capsules = {};
  for (auto it : occupancy) {
    capsules.push_back(it.get_occupancy());
  }

  Volume::calculate(capsules);
  /*std::cout << this->x_dim/this->res << " " << this->y_dim/this->res << " " << this->z_dim/this->res << " "
            << this->points_in.size() << " " << this->volume << "\n";*/
  return this->volume;
}

double Volume::volume_routine(reach_lib::ArticulatedVel human) {
  if (this->grid.size() == 0) {
    Volume::generate_grid();
  }
  std::vector<reach_lib::BodyPartVel> occupancy = human.get_occupancy();
  std::vector<reach_lib::Capsule> capsules = {};
  for (auto it : occupancy) {
    capsules.push_back(it.get_occupancy());
  }

  Volume::calculate(capsules);
  /*std::cout << this->x_dim/this->res << " " << this->y_dim/this->res << " " << this->z_dim/this->res << " "
            << this->points_in.size() << " " << this->volume << "\n";*/
  return this->volume;
}

double Volume::volume_routine(reach_lib::ArticulatedPos human) {
  if (this->grid.size() == 0) {
    Volume::generate_grid();
  }
  std::vector<reach_lib::Extremity> occupancy = human.get_occupancy();
  std::vector<reach_lib::Capsule> capsules = {};
  for (auto it : occupancy) {
    capsules.push_back(it.get_occupancy());
  }

  Volume::calculate(capsules);
  /*std::cout << this->x_dim/this->res << " " << this->y_dim/this->res << " " << this->z_dim/this->res << " "
            << this->points_in.size() << " " << this->volume << "\n";*/
  return this->volume;
}

double Volume::volume_routine(reach_lib::PedestrianAccel human) {
  if (this->grid.size() == 0) {
    Volume::generate_grid();
  }
  std::vector<reach_lib::CylinderPerimeter> occupancy = human.get_occupancy();
  std::vector<reach_lib::Cylinder> cylinders = {};
  for (auto it : occupancy) {
    cylinders.push_back(it.get_occupancy());
  }

  Volume::calculate(cylinders);
  /*std::cout << this->x_dim/this->res << " " << this->y_dim/this->res << " " << this->z_dim/this->res << " "
            << this->points_in.size() << " " << this->volume << "\n";*/
  return this->volume;
}

double Volume::volume_routine(reach_lib::PedestrianVel human) {
  if (this->grid.size() == 0) {
    Volume::generate_grid();
  }
  std::vector<reach_lib::CylinderPerimeter> occupancy = human.get_occupancy();
  std::vector<reach_lib::Cylinder> cylinders = {};
  for (auto it : occupancy) {
    cylinders.push_back(it.get_occupancy());
  }

  Volume::calculate(cylinders);
  /*std::cout << this->x_dim/this->res << " " << this->y_dim/this->res << " " << this->z_dim/this->res << " "
            << this->points_in.size() << " " << this->volume << "\n";*/
  return this->volume;
}

}  // namespace volume
