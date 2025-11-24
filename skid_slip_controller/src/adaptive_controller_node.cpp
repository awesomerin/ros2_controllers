#include "skid_slip_controller/adaptive_controller_node.hpp"
#include <cmath>

namespace skid_slip_controller
{

AdaptiveControllerNode::AdaptiveControllerNode(const rclcpp::NodeOptions & options)
: Node("adaptive_controller", options),
  current_surface_(SurfaceType::UNKNOWN),
  surface_classify_counter_(0)
{
  // Declare and get parameters
  this->declare_parameter("update_rate", 20.0);
  this->declare_parameter("slip_threshold", 0.2);
  this->declare_parameter("enable_adaptation", true);
  this->declare_parameter("surface_classify_period", 20);  // Classify every 20 updates (1 Hz at 20 Hz)
  this->declare_parameter("min_turn_radius", 0.5);
  this->declare_parameter("velocity_dependent_radius", true);

  update_rate_ = this->get_parameter("update_rate").as_double();
  slip_threshold_ = this->get_parameter("slip_threshold").as_double();
  enable_adaptation_ = this->get_parameter("enable_adaptation").as_bool();
  surface_classify_period_ = this->get_parameter("surface_classify_period").as_int();

  // Initialize components
  TurnRadiusConstraint::Parameters turn_params;
  turn_params.min_radius = this->get_parameter("min_turn_radius").as_double();
  turn_params.velocity_dependent = this->get_parameter("velocity_dependent_radius").as_bool();
  turn_params.enable = enable_adaptation_;
  turn_constraint_ = std::make_unique<TurnRadiusConstraint>(turn_params);

  slip_detector_ = std::make_unique<SlipDetector>();
  surface_classifier_ = std::make_unique<SurfaceClassifier>();

  // Initialize with default parameters
  current_params_ = surface_classifier_->getParameters(SurfaceType::DRY_GRASS);

  // Subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_in", 10,
    std::bind(&AdaptiveControllerNode::cmdVelCallback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", 10,
    std::bind(&AdaptiveControllerNode::imuCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&AdaptiveControllerNode::odomCallback, this, std::placeholders::_1));

  current_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "motor_currents", 10,
    std::bind(&AdaptiveControllerNode::motorCurrentCallback, this, std::placeholders::_1));

  // Publishers
  adapted_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);
  surface_type_pub_ = this->create_publisher<std_msgs::msg::String>("detected_surface", 10);

  // Timer for periodic processing
  auto timer_period = std::chrono::duration<double>(1.0 / update_rate_);
  timer_ = this->create_wall_timer(
    timer_period,
    std::bind(&AdaptiveControllerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Adaptive controller initialized");
  RCLCPP_INFO(this->get_logger(), "  Update rate: %.1f Hz", update_rate_);
  RCLCPP_INFO(this->get_logger(), "  Slip threshold: %.2f", slip_threshold_);
  RCLCPP_INFO(this->get_logger(), "  Adaptation: %s", enable_adaptation_ ? "enabled" : "disabled");
}

void AdaptiveControllerNode::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  current_cmd_ = *msg;
}

void AdaptiveControllerNode::imuCallback(
  const sensor_msgs::msg::Imu::SharedPtr msg)
{
  latest_imu_ = *msg;
}

void AdaptiveControllerNode::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = *msg;
}

void AdaptiveControllerNode::motorCurrentCallback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  motor_currents_ = msg->data;
}

void AdaptiveControllerNode::timerCallback()
{
  // Update slip estimate
  updateSlipEstimate();

  // Periodically classify surface (not every update, too computationally expensive)
  if (++surface_classify_counter_ >= surface_classify_period_) {
    classifySurface();
    surface_classify_counter_ = 0;
  }

  // Adapt parameters based on current conditions
  adaptParameters();

  // Publish adapted command
  publishAdaptedCommand();
}

void AdaptiveControllerNode::updateSlipEstimate()
{
  // Extract velocities from odometry (encoder-based)
  double encoder_velocity = latest_odom_.twist.twist.linear.x;
  double encoder_angular = latest_odom_.twist.twist.angular.z;

  // Extract velocities from IMU
  // Note: IMU gives acceleration, need to integrate for velocity
  // For simplicity, we'll use angular velocity from gyro directly
  // and estimate linear velocity from acceleration (simplified)
  double imu_angular = latest_imu_.angular_velocity.z;

  // Simplified linear velocity estimate from IMU (would need proper integration in practice)
  // For now, assume we have integrated acceleration elsewhere or use visual odometry
  double imu_velocity = encoder_velocity;  // Placeholder - should come from IMU integration

  // Get current time
  double timestamp = this->now().seconds();

  // Detect slip
  current_slip_ = slip_detector_->detectSlip(
    encoder_velocity,
    imu_velocity,
    encoder_angular,
    imu_angular,
    timestamp);
}

void AdaptiveControllerNode::classifySurface()
{
  // Prepare sensor readings
  SurfaceClassifier::SensorReadings readings;
  readings.slip_ratio = current_slip_.longitudinal_slip;

  // Average motor current if available
  if (!motor_currents_.empty()) {
    double sum = 0.0;
    for (double current : motor_currents_) {
      sum += current;
    }
    readings.motor_current_avg = sum / motor_currents_.size();
  }

  // Compute vibration level from IMU
  readings.vibration_level = std::sqrt(
    latest_imu_.linear_acceleration.x * latest_imu_.linear_acceleration.x +
    latest_imu_.linear_acceleration.y * latest_imu_.linear_acceleration.y +
    latest_imu_.linear_acceleration.z * latest_imu_.linear_acceleration.z);

  // Velocity tracking error (simplified)
  readings.velocity_tracking_error = std::abs(
    current_cmd_.linear.x - latest_odom_.twist.twist.linear.x);

  // Classify surface
  current_surface_ = surface_classifier_->classify(readings);

  // Publish surface type
  std_msgs::msg::String surface_msg;
  surface_msg.data = SurfaceClassifier::toString(current_surface_);
  surface_type_pub_->publish(surface_msg);

  RCLCPP_INFO(this->get_logger(), "Detected surface: %s", surface_msg.data.c_str());
}

void AdaptiveControllerNode::adaptParameters()
{
  if (!enable_adaptation_) {
    return;
  }

  // Get base parameters for detected surface
  current_params_ = surface_classifier_->getParameters(current_surface_);

  // Real-time adaptation based on current slip
  auto filtered_slip = slip_detector_->getFilteredSlip();

  // If experiencing high slip, be more conservative
  if (filtered_slip.longitudinal_slip > slip_threshold_) {
    current_params_.max_acceleration *= 0.7;
    current_params_.max_velocity *= 0.8;
    RCLCPP_DEBUG(this->get_logger(), "High slip detected, reducing aggressiveness");
  }

  if (filtered_slip.lateral_slip > slip_threshold_) {
    current_params_.min_turn_radius *= 1.5;
    RCLCPP_DEBUG(this->get_logger(), "Lateral slip detected, increasing turn radius");
  }

  // Update turn radius constraint
  TurnRadiusConstraint::Parameters turn_params = turn_constraint_->getParameters();
  turn_params.min_radius = current_params_.min_turn_radius;
  turn_constraint_->setParameters(turn_params);
}

void AdaptiveControllerNode::publishAdaptedCommand()
{
  geometry_msgs::msg::Twist adapted_cmd = current_cmd_;

  // Apply velocity limits
  adapted_cmd.linear.x = std::clamp(
    adapted_cmd.linear.x,
    -current_params_.max_velocity,
    current_params_.max_velocity);

  // Apply turn radius constraint
  turn_constraint_->apply(adapted_cmd.linear.x, adapted_cmd.angular.z);

  // Publish adapted command
  adapted_cmd_pub_->publish(adapted_cmd);
}

}  // namespace skid_slip_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<skid_slip_controller::AdaptiveControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
