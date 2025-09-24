#include <gtest/gtest.h>
#include <stdexcept>
#include <vector>
#include <map>
#include <string>

#include "reach_lib.hpp"

namespace obstacles {
namespace articulated {
namespace pos {

/**
 * @brief Test fixture for ArticulatedPos predictions functionality
 */
class ArticulatedPosPredictionsTest : public ::testing::Test {
 protected:
  ArticulatedPos articulated_model_;
  System system_;
  std::map<std::string, jointPair> body_segment_map_;
  std::vector<double> thickness_;
  std::vector<double> max_v_;
  std::vector<double> length_;

  void SetUp() override {
    // Set up system parameters
    system_.measurement_error_pos_ = 0.01;
    system_.measurement_error_vel_ = 0.02;
    system_.delay_ = 0.01;

    // Set up extremities: left arm, right arm (shoulders as base joints)
    body_segment_map_["left_arm"] = std::make_pair(0, 0);   // Left shoulder (base joint only)
    body_segment_map_["right_arm"] = std::make_pair(1, 1);  // Right shoulder (base joint only)

    // Extremity properties (thickness is hand thickness, length is shoulder to hand)
    thickness_ = {0.05, 0.05};   // Hand thickness for both arms
    max_v_ = {2.0, 2.5};        // Maximum velocity for shoulders
    length_ = {0.6, 0.65};      // Arm length (shoulder to hand)

    // Create the articulated model
    articulated_model_ = ArticulatedPos(system_, body_segment_map_, thickness_, max_v_, length_);
  }

  /**
   * @brief Helper function to create a prediction with spheres at given positions
   */
  reach_lib::Prediction createPrediction(double time,
                                         const std::vector<reach_lib::Point>& positions,
                                         double error_radius = 0.05) {
    std::vector<reach_lib::Sphere> spheres;
    for (const auto& pos : positions) {
      spheres.emplace_back(pos, error_radius);
    }
    return std::make_pair(time, spheres);
  }

  /**
   * @brief Helper function to create a prediction with different error radii per joint
   */
  reach_lib::Prediction createPredictionWithVariableErrors(double time,
                                                          const std::vector<reach_lib::Point>& positions,
                                                          const std::vector<double>& error_radii) {
    std::vector<reach_lib::Sphere> spheres;
    for (size_t i = 0; i < positions.size(); ++i) {
      double radius = (i < error_radii.size()) ? error_radii[i] : 0.05;
      spheres.emplace_back(positions[i], radius);
    }
    return std::make_pair(time, spheres);
  }
};

TEST_F(ArticulatedPosPredictionsTest, UpdateWithPredictions_BasicFunctionality) {
  // Create test predictions with variable errors for shoulder joints
  std::vector<reach_lib::Point> positions_t0 = {
    reach_lib::Point(0.0, 0.5, 1.4),   // Left shoulder
    reach_lib::Point(0.0, -0.5, 1.4)   // Right shoulder
  };

  std::vector<reach_lib::Point> positions_t1 = {
    reach_lib::Point(0.1, 0.5, 1.4),   // Left shoulder - slight movement
    reach_lib::Point(0.0, -0.5, 1.4)   // Right shoulder - stationary
  };

  // Variable errors: left shoulder more uncertain than right
  std::vector<double> errors_t0 = {0.02, 0.01};
  std::vector<double> errors_t1 = {0.03, 0.01};

  std::vector<reach_lib::Prediction> predictions = {
    createPredictionWithVariableErrors(0.0, positions_t0, errors_t0),
    createPredictionWithVariableErrors(1.0, positions_t1, errors_t1)
  };

  // t_a = 0.1 -> uses predictions[0]
  double t_a = 0.1;
  double t_b = 0.2;

  // Call update_with_predictions
  std::vector<Extremity> result = articulated_model_.update_with_predictions(t_a, t_b, predictions);

  // Verify that we got the expected number of extremities
  EXPECT_EQ(result.size(), 2);  // left_arm and right_arm

  // Verify that each extremity has valid occupancy
  for (const auto& extremity : result) {
    auto capsule = extremity.get_occupancy();
    EXPECT_GT(capsule.r_, 0.0);  // Radius should be positive
  }

  // Check left arm capsule (should use errors_t0[0] = 0.02)
  auto left_arm_capsule = result[0].get_occupancy();
  EXPECT_NEAR(left_arm_capsule.p1_.x, 0.0, 1e-7);
  EXPECT_NEAR(left_arm_capsule.p1_.y, 0.5, 1e-7);
  EXPECT_NEAR(left_arm_capsule.p1_.z, 1.4, 1e-7);
  EXPECT_NEAR(left_arm_capsule.p2_.x, 0.0, 1e-7);
  EXPECT_NEAR(left_arm_capsule.p2_.y, 0.5, 1e-7);
  EXPECT_NEAR(left_arm_capsule.p2_.z, 1.4, 1e-7);

  double pos_error = errors_t0[0];  // 0.02
  double t = t_b - predictions[0].first + system_.delay_;  // 0.2 - 0.0 + 0.01 = 0.21
  double vel = max_v_[0];  // 2.0
  double expected_radius = pos_error + t * vel + length_[0] + thickness_[0]/2.0;
  EXPECT_NEAR(left_arm_capsule.r_, expected_radius, 1e-7);

  // Check right arm capsule (should use errors_t0[1] = 0.01)
  auto right_arm_capsule = result[1].get_occupancy();
  EXPECT_NEAR(right_arm_capsule.p1_.x, 0.0, 1e-7);
  EXPECT_NEAR(right_arm_capsule.p1_.y, -0.5, 1e-7);
  EXPECT_NEAR(right_arm_capsule.p1_.z, 1.4, 1e-7);

  pos_error = errors_t0[1];  // 0.01
  vel = max_v_[1];  // 2.5
  expected_radius = pos_error + t * vel + length_[1] + thickness_[1]/2.0;
  EXPECT_NEAR(right_arm_capsule.r_, expected_radius, 1e-7);

  // Test with second prediction (t_a = 1.1 -> uses predictions[1])
  t_a = 1.1;
  t_b = 2.1;

  result = articulated_model_.update_with_predictions(t_a, t_b, predictions);
  EXPECT_EQ(result.size(), 2);

  // Check updated positions from predictions[1]
  left_arm_capsule = result[0].get_occupancy();
  EXPECT_NEAR(left_arm_capsule.p1_.x, 0.1, 1e-7);  // Updated position
  EXPECT_NEAR(left_arm_capsule.p1_.y, 0.5, 1e-7);
  EXPECT_NEAR(left_arm_capsule.p1_.z, 1.4, 1e-7);

  pos_error = errors_t1[0];  // 0.03 (increased error)
  t = t_b - predictions[1].first + system_.delay_;  // 2.1 - 1.0 + 0.01 = 1.11
  vel = max_v_[0];  // 2.0
  expected_radius = pos_error + t * vel + length_[0] + thickness_[0]/2.0;
  EXPECT_NEAR(left_arm_capsule.r_, expected_radius, 1e-7);
}

TEST_F(ArticulatedPosPredictionsTest, UpdateWithPredictions_EmptyPredictionsThrows) {
  std::vector<reach_lib::Prediction> empty_predictions;

  EXPECT_THROW(
    articulated_model_.update_with_predictions(1.0, 2.0, empty_predictions),
    std::invalid_argument
  );
}

TEST_F(ArticulatedPosPredictionsTest, UpdateWithPredictions_FirstPredictionTimeInvalid) {
  // Create prediction with time > t_a
  std::vector<reach_lib::Point> positions = {
    reach_lib::Point(0.0, 0.5, 1.4),
    reach_lib::Point(0.0, -0.5, 1.4)
  };

  std::vector<reach_lib::Prediction> predictions = {
    createPrediction(2.0, positions)  // time=2.0 > t_a=1.0
  };

  EXPECT_THROW(
    articulated_model_.update_with_predictions(1.0, 2.0, predictions),
    std::invalid_argument
  );
}

}  // namespace pos
}  // namespace articulated
}  // namespace obstacles