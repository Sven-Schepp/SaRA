// -*- lsst-c++ -*/
/**
 * @file BODY_PART_COMBINED_fixture.h
 * @brief Defines the test fixture for the human reach object
 * @version 0.1
 * @copyright This file is part of SaRA-Shield.
 * SaRA-Shield is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 * SaRA-Shield is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with SaRA-Shield.
 * If not, see <https://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "reach_lib.hpp"

#ifndef BODY_PART_COMBINED_FIXTURE_H
#define BODY_PART_COMBINED_FIXTURE_H

namespace occupancies{
namespace body_parts {
namespace combined {

/**
 * @brief Test fixture for the combined body part
 */
class BodyPartCombinedTest : public ::testing::Test {
 protected:
  /**
   * @brief The body part combined object
   */
  BodyPartCombined body_;

  double measurement_error_pos_;
  double measurement_error_vel_;
  double v_max_;
  double a_max_;
  double thickness_;

  /**
   * @brief Create the body part combined object
   */
  void SetUp() override {
    measurement_error_pos_ = 0.01;
    measurement_error_vel_ = 0.02;
    v_max_ = 2.0;
    a_max_ = 50.0;
    thickness_ = 0.1;

    body_ = BodyPartCombined("test", thickness_, v_max_, v_max_, a_max_, a_max_);
  }
};
}  // namespace combined
}  // namespace body_parts
}  // namespace occupancies

#endif  // BODY_PART_COMBINED_FIXTURE_H