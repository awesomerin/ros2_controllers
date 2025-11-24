#include "skid_slip_controller/slip_detector.hpp"

namespace skid_slip_controller
{

SlipDetector::SlipDetector(const Parameters & params)
: params_(params)
{
}

SlipDetector::SlipData SlipDetector::detectSlip(
  double encoder_velocity,
  double imu_velocity,
  double encoder_angular,
  double imu_angular,
  double timestamp)
{
  SlipData slip;
  slip.timestamp = timestamp;

  // Compute longitudinal (forward/back) slip
  slip.longitudinal_slip = computeSlipRatio(
    encoder_velocity,
    imu_velocity,
    params_.velocity_threshold);

  // Compute lateral (turning) slip
  slip.lateral_slip = computeSlipRatio(
    encoder_angular,
    imu_angular,
    params_.angular_threshold);

  // Compute confidence based on velocity magnitude
  double vel_avg = (std::abs(encoder_velocity) + std::abs(imu_velocity)) / 2.0;
  slip.confidence = computeConfidence(vel_avg);

  // Update history
  addToHistory(slip);
  latest_slip_ = slip;

  return slip;
}

SlipDetector::SlipData SlipDetector::getFilteredSlip() const
{
  return computeFilteredSlip();
}

bool SlipDetector::isSlipping(double threshold) const
{
  SlipData filtered = getFilteredSlip();

  // Check if either longitudinal or lateral slip exceeds threshold
  return (filtered.longitudinal_slip > threshold) ||
         (filtered.lateral_slip > threshold);
}

void SlipDetector::reset()
{
  slip_history_.clear();
  latest_slip_ = SlipData{};
}

void SlipDetector::setParameters(const Parameters & params)
{
  params_ = params;

  // Resize history if window size changed
  while (slip_history_.size() > params_.filter_window_size) {
    slip_history_.pop_front();
  }
}

const SlipDetector::Parameters & SlipDetector::getParameters() const
{
  return params_;
}

double SlipDetector::computeSlipRatio(
  double measured,
  double actual,
  double threshold) const
{
  // Compute absolute difference
  double diff = std::abs(measured - actual);

  // Compute average magnitude
  double avg = (std::abs(measured) + std::abs(actual)) / 2.0;

  // Avoid division by zero
  if (avg < threshold) {
    return 0.0;  // Both velocities near zero, no meaningful slip
  }

  // Slip ratio = difference / average
  double slip = diff / avg;

  // Clamp to [0, 1]
  return std::clamp(slip, 0.0, 1.0);
}

double SlipDetector::computeConfidence(double velocity) const
{
  // Confidence increases with velocity magnitude
  // Full confidence at confidence_velocity_scale
  double confidence = std::abs(velocity) / params_.confidence_velocity_scale;

  // Clamp to [0, 1]
  return std::clamp(confidence, 0.0, 1.0);
}

void SlipDetector::addToHistory(const SlipData & slip)
{
  slip_history_.push_back(slip);

  // Maintain window size
  while (slip_history_.size() > params_.filter_window_size) {
    slip_history_.pop_front();
  }
}

SlipDetector::SlipData SlipDetector::computeFilteredSlip() const
{
  if (slip_history_.empty()) {
    return SlipData{};
  }

  // Compute moving average
  SlipData filtered{};
  double total_confidence = 0.0;

  for (const auto & slip : slip_history_) {
    filtered.longitudinal_slip += slip.longitudinal_slip * slip.confidence;
    filtered.lateral_slip += slip.lateral_slip * slip.confidence;
    total_confidence += slip.confidence;
  }

  // Weighted average by confidence
  if (total_confidence > 1e-6) {
    filtered.longitudinal_slip /= total_confidence;
    filtered.lateral_slip /= total_confidence;
    filtered.confidence = total_confidence / slip_history_.size();
  }

  // Use most recent timestamp
  filtered.timestamp = slip_history_.back().timestamp;

  return filtered;
}

}  // namespace skid_slip_controller
