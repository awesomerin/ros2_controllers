#ifndef SKID_SLIP_CONTROLLER__SLIP_DETECTOR_HPP_
#define SKID_SLIP_CONTROLLER__SLIP_DETECTOR_HPP_

#include <cmath>
#include <deque>
#include <algorithm>

namespace skid_slip_controller
{

/**
 * @brief Detects wheel slip by comparing encoder odometry with IMU data
 *
 * Compares commanded/reported motion (from wheel encoders) against actual
 * motion (from IMU). Significant differences indicate wheel slip.
 */
class SlipDetector
{
public:
  struct SlipData
  {
    double longitudinal_slip{0.0};  ///< Forward/backward slip ratio [0-1]
    double lateral_slip{0.0};       ///< Sideways slip ratio [0-1]
    double confidence{0.0};         ///< Confidence in measurement [0-1]
    double timestamp{0.0};          ///< Time of measurement [s]
  };

  struct Parameters
  {
    double velocity_threshold{0.01};   ///< Min velocity for slip detection [m/s]
    double angular_threshold{0.01};    ///< Min angular vel for slip detection [rad/s]
    size_t filter_window_size{10};     ///< Moving average window size
    double confidence_velocity_scale{0.5};  ///< Velocity for full confidence [m/s]
  };

  /**
   * @brief Constructor
   * @param params Configuration parameters
   */
  explicit SlipDetector(const Parameters & params = Parameters{});

  /**
   * @brief Detect slip by comparing encoder and IMU velocities
   *
   * @param encoder_velocity Linear velocity from wheel encoders [m/s]
   * @param imu_velocity Linear velocity from IMU integration [m/s]
   * @param encoder_angular Angular velocity from wheel diff [rad/s]
   * @param imu_angular Angular velocity from IMU gyro [rad/s]
   * @param timestamp Current time [s]
   * @return SlipData struct with slip ratios and confidence
   */
  SlipData detectSlip(
    double encoder_velocity,
    double imu_velocity,
    double encoder_angular,
    double imu_angular,
    double timestamp);

  /**
   * @brief Get filtered slip estimate (moving average)
   * @return Filtered slip data
   */
  SlipData getFilteredSlip() const;

  /**
   * @brief Check if currently experiencing significant slip
   * @param threshold Slip ratio threshold [0-1]
   * @return true if slip exceeds threshold
   */
  bool isSlipping(double threshold = 0.2) const;

  /**
   * @brief Reset slip history
   */
  void reset();

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
  std::deque<SlipData> slip_history_;
  SlipData latest_slip_;

  /**
   * @brief Compute slip ratio between two velocities
   * @param measured Measured velocity (encoder)
   * @param actual Actual velocity (IMU)
   * @param threshold Minimum velocity to compute slip
   * @return Slip ratio [0-1]
   */
  double computeSlipRatio(double measured, double actual, double threshold) const;

  /**
   * @brief Compute confidence based on velocity magnitude
   * @param velocity Velocity magnitude [m/s]
   * @return Confidence [0-1]
   */
  double computeConfidence(double velocity) const;

  /**
   * @brief Add slip data to history and maintain window size
   * @param slip Slip data to add
   */
  void addToHistory(const SlipData & slip);

  /**
   * @brief Compute moving average of slip history
   * @return Averaged slip data
   */
  SlipData computeFilteredSlip() const;
};

}  // namespace skid_slip_controller

#endif  // SKID_SLIP_CONTROLLER__SLIP_DETECTOR_HPP_
