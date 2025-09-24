#include <gtest/gtest.h>
#include <stdexcept>
#include <vector>
#include <map>
#include <string>

#include "reach_lib.hpp"

namespace obstacles {
namespace articulated {
namespace vel {

/**
 * @brief Test fixture for ArticulatedVel predictions functionality
 */
class ArticulatedVelPredictionsTest : public ::testing::Test {
 protected:
  ArticulatedVel articulated_model_;
  System system_;
  std::map<std::string, jointPair> body_segment_map_;
  std::map<std::string, double> thickness_;
  std::vector<double> max_v_;

  void SetUp() override {
    // Set up system parameters
    system_.measurement_error_pos_ = 0.01;
    system_.measurement_error_vel_ = 0.02;
    system_.delay_ = 0.01;

    // Set up a simple 2-joint articulated model
    body_segment_map_["upper_arm"] = std::make_pair(0, 1);
    body_segment_map_["lower_arm"] = std::make_pair(1, 2);

    thickness_["upper_arm"] = 0.08;
    thickness_["lower_arm"] = 0.06;

    // Maximum velocities for joints 0, 1, 2
    max_v_ = {2.0, 2.5, 2.0};

    // Create the articulated model
    articulated_model_ = ArticulatedVel(system_, body_segment_map_, thickness_, max_v_);
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

TEST_F(ArticulatedVelPredictionsTest, UpdateWithPredictions_BasicFunctionality) {
  // Create test predictions with realistic variable errors
  std::vector<reach_lib::Point> positions_t0 = {
    reach_lib::Point(0.0, 0.0, 0.0),  // Joint 0 (shoulder)
    reach_lib::Point(0.3, 0.0, 0.0),  // Joint 1 (elbow)
    reach_lib::Point(0.6, 0.0, 0.0)   // Joint 2 (wrist)
  };

  std::vector<reach_lib::Point> positions_t1 = {
    reach_lib::Point(0.0, 0.0, 0.0),  // Joint 0 (shoulder) - stationary
    reach_lib::Point(0.35, 0.0, 0.0), // Joint 1 (elbow) - small movement
    reach_lib::Point(0.7, 0.0, 0.0)   // Joint 2 (wrist) - larger movement
  };

  // Realistic error progression: shoulder joint most accurate, wrist least accurate
  std::vector<double> errors_t0 = {0.01, 0.03, 0.05};
  std::vector<double> errors_t1 = {0.01, 0.04, 0.07};

  std::vector<reach_lib::Prediction> predictions = {
    createPredictionWithVariableErrors(0.0, positions_t0, errors_t0),
    createPredictionWithVariableErrors(1.0, positions_t1, errors_t1)
  };

  // t_a = 0.1 > predictions[0] but < predictions[1]
  // -> Only predictions[0] relevant. 
  double t_a = 0.1;
  double t_b = 0.2;

  // Call update_with_predictions
  std::vector<BodyPartVel> result = articulated_model_.update_with_predictions(t_a, t_b, predictions);

  // Verify that we got the expected number of body parts
  EXPECT_EQ(result.size(), 2);  // upper_arm and lower_arm

  // Verify that each body part has valid occupancy
  for (const auto& body_part : result) {
    auto capsule = body_part.get_occupancy();
    EXPECT_GT(capsule.r_, 0.0);  // Radius should be positive
  }

  // Check upper arm capsule
  auto upper_arm_capsule = result[1].get_occupancy();
  EXPECT_NEAR(upper_arm_capsule.p1_.x, 0.0, 1e-7);
  EXPECT_NEAR(upper_arm_capsule.p1_.y, 0.0, 1e-7);
  EXPECT_NEAR(upper_arm_capsule.p1_.z, 0.0, 1e-7);
  EXPECT_NEAR(upper_arm_capsule.p2_.x, 0.3, 1e-7);
  EXPECT_NEAR(upper_arm_capsule.p2_.y, 0.0, 1e-7);
  EXPECT_NEAR(upper_arm_capsule.p2_.z, 0.0, 1e-7);
  // Should use larger error: 0.03 > 0.01 -> 0.03
  double pos_error = std::max(errors_t0[0], errors_t0[1]);
  double t = t_b - predictions[0].first + system_.delay_;
  double vel = std::max(max_v_[0], max_v_[1]);
  EXPECT_NEAR(upper_arm_capsule.r_, pos_error + t * vel + thickness_["upper_arm"]/2.0, 1e-7);
  auto lower_arm_capsule = result[0].get_occupancy();
  EXPECT_NEAR(lower_arm_capsule.p1_.x, 0.3, 1e-7);
  EXPECT_NEAR(lower_arm_capsule.p1_.y, 0.0, 1e-7);
  EXPECT_NEAR(lower_arm_capsule.p1_.z, 0.0, 1e-7);
  EXPECT_NEAR(lower_arm_capsule.p2_.x, 0.6, 1e-7);
  EXPECT_NEAR(lower_arm_capsule.p2_.y, 0.0, 1e-7);
  EXPECT_NEAR(lower_arm_capsule.p2_.z, 0.0, 1e-7);
  pos_error = std::max(errors_t0[1], errors_t0[2]);
  t = t_b - predictions[0].first + system_.delay_;
  vel = std::max(max_v_[1], max_v_[2]);
  EXPECT_NEAR(lower_arm_capsule.r_, pos_error + t * vel + thickness_["lower_arm"]/2.0, 1e-7);

  // t_a = 1.1 > predictions[1]
  // -> Only predictions[1] relevant. 
  t_a = 1.1;
  t_b = 2.1;

  // Call update_with_predictions
  result = articulated_model_.update_with_predictions(t_a, t_b, predictions);

  // Verify that we got the expected number of body parts
  EXPECT_EQ(result.size(), 2);  // upper_arm and lower_arm

  // Verify that each body part has valid occupancy
  for (const auto& body_part : result) {
    auto capsule = body_part.get_occupancy();
    EXPECT_GT(capsule.r_, 0.0);  // Radius should be positive
  }

  // Check upper arm capsule
  upper_arm_capsule = result[1].get_occupancy();
  EXPECT_NEAR(upper_arm_capsule.p1_.x, 0.0, 1e-7);
  EXPECT_NEAR(upper_arm_capsule.p1_.y, 0.0, 1e-7);
  EXPECT_NEAR(upper_arm_capsule.p1_.z, 0.0, 1e-7);
  EXPECT_NEAR(upper_arm_capsule.p2_.x, 0.35, 1e-7);
  EXPECT_NEAR(upper_arm_capsule.p2_.y, 0.0, 1e-7);
  EXPECT_NEAR(upper_arm_capsule.p2_.z, 0.0, 1e-7);
  // Should use larger error: 0.03 > 0.01 -> 0.03
  pos_error = std::max(errors_t1[0], errors_t1[1]);
  t = t_b - predictions[1].first + system_.delay_;
  vel = std::max(max_v_[0], max_v_[1]);
  EXPECT_NEAR(upper_arm_capsule.r_, pos_error + t * vel + thickness_["upper_arm"]/2.0, 1e-7);
  lower_arm_capsule = result[0].get_occupancy();
  EXPECT_NEAR(lower_arm_capsule.p1_.x, 0.35, 1e-7);
  EXPECT_NEAR(lower_arm_capsule.p1_.y, 0.0, 1e-7);
  EXPECT_NEAR(lower_arm_capsule.p1_.z, 0.0, 1e-7);
  EXPECT_NEAR(lower_arm_capsule.p2_.x, 0.7, 1e-7);
  EXPECT_NEAR(lower_arm_capsule.p2_.y, 0.0, 1e-7);
  EXPECT_NEAR(lower_arm_capsule.p2_.z, 0.0, 1e-7);
  pos_error = std::max(errors_t1[1], errors_t1[2]);
  t = t_b - predictions[1].first + system_.delay_;
  vel = std::max(max_v_[1], max_v_[2]);
  EXPECT_NEAR(lower_arm_capsule.r_, pos_error + t * vel + thickness_["lower_arm"]/2.0, 1e-7);
}

TEST_F(ArticulatedVelPredictionsTest, UpdateWithPredictions_EmptyPredictionsThrows) {
  std::vector<reach_lib::Prediction> empty_predictions;

  EXPECT_THROW(
    articulated_model_.update_with_predictions(1.0, 2.0, empty_predictions),
    std::invalid_argument
  );
}

TEST_F(ArticulatedVelPredictionsTest, UpdateWithPredictions_FirstPredictionTimeInvalid) {
  // Create prediction with time > t_a
  std::vector<reach_lib::Point> positions = {
    reach_lib::Point(0.0, 0.0, 0.0),
    reach_lib::Point(0.3, 0.0, 0.0),
    reach_lib::Point(0.6, 0.0, 0.0)
  };

  std::vector<reach_lib::Prediction> predictions = {
    createPrediction(2.0, positions)  // time=2.0 > t_a=1.0
  };

  EXPECT_THROW(
    articulated_model_.update_with_predictions(1.0, 2.0, predictions),
    std::invalid_argument
  );
}
}  // namespace vel
}  // namespace articulated
}  // namespace obstacles