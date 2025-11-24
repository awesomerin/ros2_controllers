#include <gtest/gtest.h>
#include "skid_slip_controller/surface_classifier.hpp"

using skid_slip_controller::SurfaceClassifier;
using skid_slip_controller::SurfaceType;

class SurfaceClassifierTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    classifier_ = std::make_unique<SurfaceClassifier>();
  }

  std::unique_ptr<SurfaceClassifier> classifier_;
};

TEST_F(SurfaceClassifierTest, ClassifyDryConcrete)
{
  SurfaceClassifier::SensorReadings readings;
  readings.slip_ratio = 0.05;         // Low slip
  readings.vibration_level = 4.0;     // High vibration
  readings.motor_current_avg = 3.0;

  auto surface = classifier_->classify(readings);
  EXPECT_EQ(surface, SurfaceType::DRY_CONCRETE);
}

TEST_F(SurfaceClassifierTest, ClassifyDryGrass)
{
  SurfaceClassifier::SensorReadings readings;
  readings.slip_ratio = 0.15;         // Moderate slip
  readings.vibration_level = 2.0;
  readings.motor_current_avg = 4.0;

  auto surface = classifier_->classify(readings);
  EXPECT_EQ(surface, SurfaceType::DRY_GRASS);
}

TEST_F(SurfaceClassifierTest, ClassifyWetGrass)
{
  SurfaceClassifier::SensorReadings readings;
  readings.slip_ratio = 0.35;         // High slip
  readings.vibration_level = 1.5;     // Low vibration (smooth)
  readings.motor_current_avg = 5.0;

  auto surface = classifier_->classify(readings);
  EXPECT_EQ(surface, SurfaceType::WET_GRASS);
}

TEST_F(SurfaceClassifierTest, ClassifyMud)
{
  SurfaceClassifier::SensorReadings readings;
  readings.slip_ratio = 0.6;          // Very high slip
  readings.vibration_level = 2.0;
  readings.motor_current_avg = 9.0;   // High current

  auto surface = classifier_->classify(readings);
  EXPECT_EQ(surface, SurfaceType::MUD);
}

TEST_F(SurfaceClassifierTest, GetParametersForDryGrass)
{
  auto params = classifier_->getParameters(SurfaceType::DRY_GRASS);

  EXPECT_NEAR(params.friction_coeff, 0.6, 1e-6);
  EXPECT_NEAR(params.max_acceleration, 0.5, 1e-6);
  EXPECT_NEAR(params.min_turn_radius, 0.5, 1e-6);
  EXPECT_NEAR(params.max_velocity, 0.8, 1e-6);
}

TEST_F(SurfaceClassifierTest, GetParametersForWetGrass)
{
  auto params = classifier_->getParameters(SurfaceType::WET_GRASS);

  EXPECT_NEAR(params.friction_coeff, 0.4, 1e-6);
  EXPECT_NEAR(params.max_acceleration, 0.3, 1e-6);
  EXPECT_NEAR(params.min_turn_radius, 0.8, 1e-6);
  EXPECT_NEAR(params.max_velocity, 0.5, 1e-6);
}

TEST_F(SurfaceClassifierTest, ToStringConversion)
{
  EXPECT_EQ(SurfaceClassifier::toString(SurfaceType::DRY_CONCRETE), "dry_concrete");
  EXPECT_EQ(SurfaceClassifier::toString(SurfaceType::DRY_GRASS), "dry_grass");
  EXPECT_EQ(SurfaceClassifier::toString(SurfaceType::WET_GRASS), "wet_grass");
  EXPECT_EQ(SurfaceClassifier::toString(SurfaceType::MUD), "mud");
  EXPECT_EQ(SurfaceClassifier::toString(SurfaceType::UNKNOWN), "unknown");
}

TEST_F(SurfaceClassifierTest, FromStringConversion)
{
  EXPECT_EQ(SurfaceClassifier::fromString("dry_concrete"), SurfaceType::DRY_CONCRETE);
  EXPECT_EQ(SurfaceClassifier::fromString("DRY_GRASS"), SurfaceType::DRY_GRASS);
  EXPECT_EQ(SurfaceClassifier::fromString("Wet_Grass"), SurfaceType::WET_GRASS);
  EXPECT_EQ(SurfaceClassifier::fromString("mud"), SurfaceType::MUD);
  EXPECT_EQ(SurfaceClassifier::fromString("invalid"), SurfaceType::UNKNOWN);
}

TEST_F(SurfaceClassifierTest, UnknownSurfaceUsesDefaults)
{
  auto params = classifier_->getParameters(SurfaceType::UNKNOWN);

  // Should use DRY_GRASS defaults
  auto dry_grass_params = classifier_->getParameters(SurfaceType::DRY_GRASS);
  EXPECT_NEAR(params.friction_coeff, dry_grass_params.friction_coeff, 1e-6);
  EXPECT_NEAR(params.max_acceleration, dry_grass_params.max_acceleration, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
