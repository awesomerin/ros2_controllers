#ifndef SKID_SLIP_CONTROLLER__SURFACE_CLASSIFIER_HPP_
#define SKID_SLIP_CONTROLLER__SURFACE_CLASSIFIER_HPP_

#include <map>
#include <string>

namespace skid_slip_controller
{

enum class SurfaceType
{
  DRY_CONCRETE,
  DRY_GRASS,
  WET_GRASS,
  MUD,
  UNKNOWN
};

/**
 * @brief Classifies surface type based on sensor readings and slip characteristics
 */
class SurfaceClassifier
{
public:
  struct SensorReadings
  {
    double slip_ratio{0.0};           ///< Slip ratio from slip detector [0-1]
    double motor_current_avg{0.0};    ///< Average motor current [A]
    double vibration_level{0.0};      ///< IMU vibration magnitude [m/s²]
    double velocity_tracking_error{0.0}; ///< Commanded vs actual velocity error [m/s]
  };

  struct SurfaceParameters
  {
    double friction_coeff{0.6};       ///< Estimated friction coefficient
    double max_acceleration{0.5};     ///< Maximum safe acceleration [m/s²]
    double min_turn_radius{0.5};      ///< Minimum safe turn radius [m]
    double max_velocity{0.8};         ///< Maximum safe velocity [m/s]
  };

  /**
   * @brief Constructor
   */
  SurfaceClassifier();

  /**
   * @brief Classify surface type based on sensor readings
   * @param readings Sensor data
   * @return Classified surface type
   */
  SurfaceType classify(const SensorReadings & readings);

  /**
   * @brief Get control parameters for a surface type
   * @param type Surface type
   * @return Recommended control parameters
   */
  SurfaceParameters getParameters(SurfaceType type) const;

  /**
   * @brief Convert surface type to string
   * @param type Surface type
   * @return String representation
   */
  static std::string toString(SurfaceType type);

  /**
   * @brief Parse string to surface type
   * @param str String representation
   * @return Surface type
   */
  static SurfaceType fromString(const std::string & str);

private:
  std::map<SurfaceType, SurfaceParameters> surface_params_;

  /**
   * @brief Initialize default parameters for each surface type
   */
  void initializeParameters();
};

}  // namespace skid_slip_controller

#endif  // SKID_SLIP_CONTROLLER__SURFACE_CLASSIFIER_HPP_
