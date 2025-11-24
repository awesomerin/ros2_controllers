#ifndef SKID_SLIP_CONTROLLER__ADAPTIVE_CONTROLLER_NODE_HPP_
#define SKID_SLIP_CONTROLLER__ADAPTIVE_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "skid_slip_controller/turn_radius_constraint.hpp"
#include "skid_slip_controller/slip_detector.hpp"
#include "skid_slip_controller/surface_classifier.hpp"

namespace skid_slip_controller
{

/**
 * @brief Adaptive controller node for skid-steer robots on variable surfaces
 *
 * Monitors slip, classifies surface, and adapts control parameters in real-time.
 */
class AdaptiveControllerNode : public rclcpp::Node
{
public:
  explicit AdaptiveControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callbacks
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void motorCurrentCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void timerCallback();

  // Processing
  void updateSlipEstimate();
  void classifySurface();
  void adaptParameters();
  void publishAdaptedCommand();

  // Publishers and subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr adapted_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr surface_type_pub_;

  // Timer for periodic processing
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  geometry_msgs::msg::Twist current_cmd_;
  sensor_msgs::msg::Imu latest_imu_;
  nav_msgs::msg::Odometry latest_odom_;
  std::vector<double> motor_currents_;

  // Components
  std::unique_ptr<TurnRadiusConstraint> turn_constraint_;
  std::unique_ptr<SlipDetector> slip_detector_;
  std::unique_ptr<SurfaceClassifier> surface_classifier_;

  // Data
  SlipDetector::SlipData current_slip_;
  SurfaceType current_surface_;
  SurfaceClassifier::SurfaceParameters current_params_;

  // Parameters
  double update_rate_;
  double slip_threshold_;
  bool enable_adaptation_;
  int surface_classify_counter_;
  int surface_classify_period_;  // Classify every N updates
};

}  // namespace skid_slip_controller

#endif  // SKID_SLIP_CONTROLLER__ADAPTIVE_CONTROLLER_NODE_HPP_
