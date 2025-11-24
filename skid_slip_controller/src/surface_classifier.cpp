#include "skid_slip_controller/surface_classifier.hpp"
#include <algorithm>
#include <cctype>

namespace skid_slip_controller
{

SurfaceClassifier::SurfaceClassifier()
{
  initializeParameters();
}

SurfaceType SurfaceClassifier::classify(const SensorReadings & readings)
{
  // Rule-based classification
  // These thresholds should be tuned based on real-world data

  // Very high slip + high current = mud
  if (readings.slip_ratio > 0.5 && readings.motor_current_avg > 8.0) {
    return SurfaceType::MUD;
  }

  // High slip + low vibration = wet/slippery surface
  if (readings.slip_ratio > 0.3 && readings.vibration_level < 2.0) {
    return SurfaceType::WET_GRASS;
  }

  // Low slip + high vibration = rough dry surface
  if (readings.slip_ratio < 0.1 && readings.vibration_level > 3.0) {
    return SurfaceType::DRY_CONCRETE;
  }

  // Moderate slip + moderate vibration = dry grass
  if (readings.slip_ratio < 0.2) {
    return SurfaceType::DRY_GRASS;
  }

  // Default: unknown
  return SurfaceType::UNKNOWN;
}

SurfaceClassifier::SurfaceParameters SurfaceClassifier::getParameters(
  SurfaceType type) const
{
  auto it = surface_params_.find(type);
  if (it != surface_params_.end()) {
    return it->second;
  }

  // Default to dry grass if unknown
  return surface_params_.at(SurfaceType::DRY_GRASS);
}

std::string SurfaceClassifier::toString(SurfaceType type)
{
  switch (type) {
    case SurfaceType::DRY_CONCRETE:
      return "dry_concrete";
    case SurfaceType::DRY_GRASS:
      return "dry_grass";
    case SurfaceType::WET_GRASS:
      return "wet_grass";
    case SurfaceType::MUD:
      return "mud";
    default:
      return "unknown";
  }
}

SurfaceType SurfaceClassifier::fromString(const std::string & str)
{
  // Convert to lowercase for comparison
  std::string lower = str;
  std::transform(lower.begin(), lower.end(), lower.begin(),
    [](unsigned char c) { return std::tolower(c); });

  if (lower == "dry_concrete") {
    return SurfaceType::DRY_CONCRETE;
  } else if (lower == "dry_grass") {
    return SurfaceType::DRY_GRASS;
  } else if (lower == "wet_grass") {
    return SurfaceType::WET_GRASS;
  } else if (lower == "mud") {
    return SurfaceType::MUD;
  }

  return SurfaceType::UNKNOWN;
}

void SurfaceClassifier::initializeParameters()
{
  // Dry concrete: high traction
  surface_params_[SurfaceType::DRY_CONCRETE] = {
    0.9,   // friction_coeff
    1.0,   // max_acceleration
    0.3,   // min_turn_radius
    1.0    // max_velocity
  };

  // Dry grass: moderate traction
  surface_params_[SurfaceType::DRY_GRASS] = {
    0.6,   // friction_coeff
    0.5,   // max_acceleration
    0.5,   // min_turn_radius
    0.8    // max_velocity
  };

  // Wet grass: low traction
  surface_params_[SurfaceType::WET_GRASS] = {
    0.4,   // friction_coeff
    0.3,   // max_acceleration
    0.8,   // min_turn_radius
    0.5    // max_velocity
  };

  // Mud: very low traction
  surface_params_[SurfaceType::MUD] = {
    0.3,   // friction_coeff
    0.2,   // max_acceleration
    1.0,   // min_turn_radius
    0.3    // max_velocity
  };

  // Unknown: conservative defaults (same as dry grass)
  surface_params_[SurfaceType::UNKNOWN] = surface_params_[SurfaceType::DRY_GRASS];
}

}  // namespace skid_slip_controller
