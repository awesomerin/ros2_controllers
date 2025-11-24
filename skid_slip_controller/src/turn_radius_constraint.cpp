#include "skid_slip_controller/turn_radius_constraint.hpp"
#include <limits>

namespace skid_slip_controller
{

TurnRadiusConstraint::TurnRadiusConstraint(const Parameters & params)
: params_(params)
{
}

bool TurnRadiusConstraint::apply(double & v_linear, double & v_angular)
{
  if (!params_.enable || std::abs(v_angular) < 1e-6) {
    return false;  // Straight line or disabled, no constraint needed
  }

  // Compute current turn radius
  double current_radius = getTurnRadius(v_linear, v_angular);

  // Compute effective minimum radius (may be velocity-dependent)
  double min_radius = getEffectiveMinRadius(v_linear);

  // Check constraint violation
  if (current_radius < min_radius) {
    // Reduce angular velocity to meet constraint
    // Keep linear velocity unchanged to maintain forward progress
    double sign = (v_angular > 0.0) ? 1.0 : -1.0;
    v_angular = sign * std::abs(v_linear) / min_radius;
    return true;  // Constraint was applied
  }

  return false;  // No modification needed
}

double TurnRadiusConstraint::getMaxAngularVelocity(double v_linear) const
{
  if (!params_.enable) {
    return std::numeric_limits<double>::infinity();
  }

  double min_radius = getEffectiveMinRadius(v_linear);

  // Avoid division by zero
  if (min_radius < 1e-6) {
    return std::numeric_limits<double>::infinity();
  }

  return std::abs(v_linear) / min_radius;
}

bool TurnRadiusConstraint::satisfiesConstraint(double v_linear, double v_angular) const
{
  if (!params_.enable || std::abs(v_angular) < 1e-6) {
    return true;  // Straight line always OK or constraint disabled
  }

  double current_radius = getTurnRadius(v_linear, v_angular);
  double min_radius = getEffectiveMinRadius(v_linear);

  return current_radius >= min_radius;
}

double TurnRadiusConstraint::getTurnRadius(double v_linear, double v_angular) const
{
  if (std::abs(v_angular) < 1e-6) {
    return std::numeric_limits<double>::infinity();  // Straight line
  }

  return std::abs(v_linear / v_angular);
}

void TurnRadiusConstraint::setParameters(const Parameters & params)
{
  params_ = params;
}

const TurnRadiusConstraint::Parameters & TurnRadiusConstraint::getParameters() const
{
  return params_;
}

double TurnRadiusConstraint::getEffectiveMinRadius(double v_linear) const
{
  double min_radius = params_.min_radius;

  if (params_.velocity_dependent) {
    // Higher speeds need larger turn radius for stability
    // R_effective = R_base + scale * |v|
    min_radius += params_.velocity_scale * std::abs(v_linear);
  }

  return min_radius;
}

}  // namespace skid_slip_controller
