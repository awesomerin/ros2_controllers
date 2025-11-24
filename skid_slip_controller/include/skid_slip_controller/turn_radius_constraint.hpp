#ifndef SKID_SLIP_CONTROLLER__TURN_RADIUS_CONSTRAINT_HPP_
#define SKID_SLIP_CONTROLLER__TURN_RADIUS_CONSTRAINT_HPP_

#include <cmath>
#include <algorithm>

namespace skid_slip_controller
{

/**
 * @brief Enforces minimum turn radius constraint for skid-steer robots
 *
 * Prevents sharp turns that cause excessive wheel slippage on low-traction
 * surfaces like grass. Can be velocity-dependent for added safety at high speeds.
 */
class TurnRadiusConstraint
{
public:
  struct Parameters
  {
    double min_radius{0.5};           ///< Minimum turn radius [m]
    double wheelbase{0.5};            ///< Robot wheelbase [m]
    bool velocity_dependent{true};    ///< Scale radius with velocity
    double velocity_scale{1.0};       ///< Scaling factor for velocity dependence
    bool enable{true};                ///< Enable/disable constraint
  };

  /**
   * @brief Constructor
   * @param params Configuration parameters
   */
  explicit TurnRadiusConstraint(const Parameters & params = Parameters{});

  /**
   * @brief Apply turn radius constraint to commanded velocities
   *
   * Modifies angular velocity if the resulting turn radius would be too small.
   * Linear velocity is preserved to maintain forward progress.
   *
   * @param v_linear Linear velocity [m/s] (modified in place if needed)
   * @param v_angular Angular velocity [rad/s] (modified in place)
   * @return true if constraint was applied (velocities were modified)
   */
  bool apply(double & v_linear, double & v_angular);

  /**
   * @brief Get maximum angular velocity for given linear velocity
   * @param v_linear Linear velocity [m/s]
   * @return Maximum allowed angular velocity [rad/s]
   */
  double getMaxAngularVelocity(double v_linear) const;

  /**
   * @brief Check if velocities satisfy the constraint
   * @param v_linear Linear velocity [m/s]
   * @param v_angular Angular velocity [rad/s]
   * @return true if constraint is satisfied
   */
  bool satisfiesConstraint(double v_linear, double v_angular) const;

  /**
   * @brief Get current turn radius for given velocities
   * @param v_linear Linear velocity [m/s]
   * @param v_angular Angular velocity [rad/s]
   * @return Turn radius [m], infinity for straight line
   */
  double getTurnRadius(double v_linear, double v_angular) const;

  /**
   * @brief Update parameters
   * @param params New parameters
   */
  void setParameters(const Parameters & params);

  /**
   * @brief Get current parameters
   * @return Current parameters
   */
  const Parameters & getParameters() const;

private:
  Parameters params_;

  /**
   * @brief Compute effective minimum radius including velocity scaling
   * @param v_linear Current linear velocity [m/s]
   * @return Effective minimum radius [m]
   */
  double getEffectiveMinRadius(double v_linear) const;
};

}  // namespace skid_slip_controller

#endif  // SKID_SLIP_CONTROLLER__TURN_RADIUS_CONSTRAINT_HPP_
