#include <gtest/gtest.h>
#include "skid_slip_controller/turn_radius_constraint.hpp"

using skid_slip_controller::TurnRadiusConstraint;

class TurnRadiusConstraintTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    TurnRadiusConstraint::Parameters params;
    params.min_radius = 0.5;
    params.wheelbase = 0.5;
    params.velocity_dependent = false;
    params.enable = true;
    constraint_ = std::make_unique<TurnRadiusConstraint>(params);
  }

  std::unique_ptr<TurnRadiusConstraint> constraint_;
};

TEST_F(TurnRadiusConstraintTest, StraightLineNoConstraint)
{
  double v_linear = 0.5;
  double v_angular = 0.0;

  bool applied = constraint_->apply(v_linear, v_angular);

  EXPECT_FALSE(applied);
  EXPECT_DOUBLE_EQ(v_linear, 0.5);
  EXPECT_DOUBLE_EQ(v_angular, 0.0);
}

TEST_F(TurnRadiusConstraintTest, LargeTurnRadiusSatisfiesConstraint)
{
  double v_linear = 0.5;
  double v_angular = 0.5;  // Radius = 1.0m > 0.5m min

  bool applied = constraint_->apply(v_linear, v_angular);

  EXPECT_FALSE(applied);  // No modification needed
  EXPECT_DOUBLE_EQ(v_linear, 0.5);
  EXPECT_DOUBLE_EQ(v_angular, 0.5);
}

TEST_F(TurnRadiusConstraintTest, SmallTurnRadiusViolatesConstraint)
{
  double v_linear = 0.5;
  double v_angular = 2.0;  // Radius = 0.25m < 0.5m min

  bool applied = constraint_->apply(v_linear, v_angular);

  EXPECT_TRUE(applied);  // Constraint was applied
  EXPECT_DOUBLE_EQ(v_linear, 0.5);  // Linear velocity unchanged
  EXPECT_NEAR(v_angular, 1.0, 1e-6);  // Angular reduced: 0.5 / 0.5 = 1.0
}

TEST_F(TurnRadiusConstraintTest, NegativeAngularVelocity)
{
  double v_linear = 0.5;
  double v_angular = -2.0;  // Turning left, radius = 0.25m

  bool applied = constraint_->apply(v_linear, v_angular);

  EXPECT_TRUE(applied);
  EXPECT_DOUBLE_EQ(v_linear, 0.5);
  EXPECT_NEAR(v_angular, -1.0, 1e-6);  // Sign preserved
}

TEST_F(TurnRadiusConstraintTest, VelocityDependentRadius)
{
  TurnRadiusConstraint::Parameters params;
  params.min_radius = 0.5;
  params.velocity_dependent = true;
  params.velocity_scale = 1.0;
  params.enable = true;

  constraint_ = std::make_unique<TurnRadiusConstraint>(params);

  double v_linear = 1.0;  // Higher speed
  double v_angular = 1.0;  // Radius = 1.0m

  // Effective min radius = 0.5 + 1.0 * 1.0 = 1.5m
  // Current radius = 1.0m < 1.5m → should be constrained

  bool applied = constraint_->apply(v_linear, v_angular);

  EXPECT_TRUE(applied);
  EXPECT_DOUBLE_EQ(v_linear, 1.0);
  // New angular = 1.0 / 1.5 = 0.667
  EXPECT_NEAR(v_angular, 0.667, 1e-2);
}

TEST_F(TurnRadiusConstraintTest, DisabledConstraint)
{
  TurnRadiusConstraint::Parameters params;
  params.enable = false;
  constraint_ = std::make_unique<TurnRadiusConstraint>(params);

  double v_linear = 0.5;
  double v_angular = 5.0;  // Would violate if enabled

  bool applied = constraint_->apply(v_linear, v_angular);

  EXPECT_FALSE(applied);
  EXPECT_DOUBLE_EQ(v_linear, 0.5);
  EXPECT_DOUBLE_EQ(v_angular, 5.0);  // Unchanged
}

TEST_F(TurnRadiusConstraintTest, GetTurnRadiusCalculation)
{
  EXPECT_DOUBLE_EQ(constraint_->getTurnRadius(0.5, 0.5), 1.0);
  EXPECT_DOUBLE_EQ(constraint_->getTurnRadius(1.0, 0.5), 2.0);
  EXPECT_DOUBLE_EQ(constraint_->getTurnRadius(0.5, 1.0), 0.5);
}

TEST_F(TurnRadiusConstraintTest, SatisfiesConstraintCheck)
{
  EXPECT_TRUE(constraint_->satisfiesConstraint(0.5, 0.5));   // R=1.0 > 0.5
  EXPECT_FALSE(constraint_->satisfiesConstraint(0.5, 2.0));  // R=0.25 < 0.5
  EXPECT_TRUE(constraint_->satisfiesConstraint(0.5, 0.0));   // Straight line
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
