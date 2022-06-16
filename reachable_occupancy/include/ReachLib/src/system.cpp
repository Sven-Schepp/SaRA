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

#include "system.hpp"

namespace systems {

System::System(double measurement_error_pos,
               double measurement_error_vel,
               double delay) :
               measurement_error_pos_(measurement_error_pos),
               measurement_error_vel_(measurement_error_vel),
               delay_(delay) {
  // NO TODO
}
}  // namespace system
