#include <gtest/gtest.h>
#include "skid_slip_controller/slip_detector.hpp"

using skid_slip_controller::SlipDetector;

class SlipDetectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    SlipDetector::Parameters params;
    params.velocity_threshold = 0.01;
    params.filter_window_size = 5;
    detector_ = std::make_unique<SlipDetector>(params);
  }

  std::unique_ptr<SlipDetector> detector_;
};

TEST_F(SlipDetectorTest, NoSlipWhenVelocitiesMatch)
{
  double encoder_vel = 0.5;
  double imu_vel = 0.5;
  double encoder_ang = 0.2;
  double imu_ang = 0.2;

  auto slip = detector_->detectSlip(encoder_vel, imu_vel, encoder_ang, imu_ang, 0.0);

  EXPECT_NEAR(slip.longitudinal_slip, 0.0, 1e-6);
  EXPECT_NEAR(slip.lateral_slip, 0.0, 1e-6);
}

TEST_F(SlipDetectorTest, FullSlipDetection)
{
  // Encoder reports movement, IMU shows no movement = 100% slip
  double encoder_vel = 0.5;
  double imu_vel = 0.0;

  auto slip = detector_->detectSlip(encoder_vel, imu_vel, 0.0, 0.0, 0.0);

  EXPECT_NEAR(slip.longitudinal_slip, 1.0, 1e-6);
}

TEST_F(SlipDetectorTest, PartialSlip)
{
  // Encoder reports 0.5 m/s, IMU measures 0.3 m/s
  double encoder_vel = 0.5;
  double imu_vel = 0.3;

  auto slip = detector_->detectSlip(encoder_vel, imu_vel, 0.0, 0.0, 0.0);

  // Slip = |0.5 - 0.3| / ((0.5 + 0.3)/2) = 0.2 / 0.4 = 0.5
  EXPECT_NEAR(slip.longitudinal_slip, 0.5, 1e-6);
}

TEST_F(SlipDetectorTest, LowVelocityIgnored)
{
  // Very low velocities should not register as slip
  double encoder_vel = 0.005;
  double imu_vel = 0.003;

  auto slip = detector_->detectSlip(encoder_vel, imu_vel, 0.0, 0.0, 0.0);

  EXPECT_NEAR(slip.longitudinal_slip, 0.0, 1e-6);  // Below threshold
}

TEST_F(SlipDetectorTest, ConfidenceIncreasesWithVelocity)
{
  SlipDetector::Parameters params;
  params.confidence_velocity_scale = 0.5;  // Full confidence at 0.5 m/s
  detector_ = std::make_unique<SlipDetector>(params);

  // Low velocity
  auto slip1 = detector_->detectSlip(0.1, 0.1, 0.0, 0.0, 0.0);
  EXPECT_NEAR(slip1.confidence, 0.2, 1e-2);  // 0.1 / 0.5 = 0.2

  // Medium velocity
  auto slip2 = detector_->detectSlip(0.25, 0.25, 0.0, 0.0, 0.0);
  EXPECT_NEAR(slip2.confidence, 0.5, 1e-2);  // 0.25 / 0.5 = 0.5

  // High velocity (clamped at 1.0)
  auto slip3 = detector_->detectSlip(1.0, 1.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(slip3.confidence, 1.0, 1e-2);
}

TEST_F(SlipDetectorTest, MovingAverageFilter)
{
  // Add several measurements
  for (int i = 0; i < 5; ++i) {
    detector_->detectSlip(0.5, 0.3, 0.0, 0.0, i * 0.1);  // 50% slip
  }

  auto filtered = detector_->getFilteredSlip();
  EXPECT_NEAR(filtered.longitudinal_slip, 0.5, 1e-2);
}

TEST_F(SlipDetectorTest, IsSlippingThreshold)
{
  detector_->detectSlip(0.5, 0.4, 0.0, 0.0, 0.0);  // 22% slip

  EXPECT_TRUE(detector_->isSlipping(0.2));   // Above 20% threshold
  EXPECT_FALSE(detector_->isSlipping(0.3));  // Below 30% threshold
}

TEST_F(SlipDetectorTest, ResetClearsHistory)
{
  // Add measurements
  for (int i = 0; i < 5; ++i) {
    detector_->detectSlip(0.5, 0.3, 0.0, 0.0, i * 0.1);
  }

  // Reset
  detector_->reset();

  auto filtered = detector_->getFilteredSlip();
  EXPECT_NEAR(filtered.longitudinal_slip, 0.0, 1e-6);
  EXPECT_NEAR(filtered.confidence, 0.0, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
