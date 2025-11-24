# Trajectory Planning for Skid-Steer Robots on Lawns

Implementation guide with code examples for minimum-slip path planning.

---

## Table of Contents
1. [Problem Overview](#problem)
2. [Kinematic Constraints](#kinematics)
3. [Minimum Turn Radius Constraint](#min-radius)
4. [Implementation in TEB Planner](#teb-implementation)
5. [Custom Planner with ICR Constraints](#custom-planner)
6. [Code Examples](#code-examples)

---

## 1. Problem Overview <a name="problem"></a>

### Why Standard Differential Drive Planning Fails on Grass

**Standard assumption:**
```
Differential drive can rotate in place (turn radius = 0)
```

**Reality on grass:**
```
Rotating in place causes MAXIMUM skidding!
```

**Visual comparison:**

**On concrete (works fine):**
```
    Start
      🤖
      ↻  Spin in place (no problem)
      🤖
    End
```

**On grass (disaster!):**
```
    Start
      🤖
      ⚠️  Wheels skid sideways
      🤖  Tears up lawn
    💥💥  Odometry errors
```

### The Physics of Skidding

**Instantaneous Center of Rotation (ICR):**

For differential drive:
```
        ICR
         │
    ─────┼─────  Wheelbase
    │    │    │
   Left  │   Right
  wheel  │   wheel
```

**ICR location determines slip:**

**Pure rotation (v=0, ω≠0):**
```
ICR at robot center → Both wheels MUST slip!

  ◄────●────►
  Left   Right

Both wheels move sideways relative to rolling direction
```

**Gentle turn (large radius):**
```
ICR far from robot → Minimal slip

                ICR
                 │
                 │ R (large)
            ─────┼─────
```

**Sharp turn (small radius):**
```
ICR near robot → More slip

       ICR
        │
        │ R (small)
   ─────┼─────
```

### Minimum Turn Radius Constraint

**Formula:**
```
R_min = f(μ, wheelbase, velocity)

where:
  μ = coefficient of friction (grass: 0.4-0.6)
  R_min = minimum radius without excessive slip
```

**Practical approximation:**
```
R_min ≈ wheelbase / 2  (conservative)
R_min ≈ wheelbase      (moderate)
R_min ≈ 2 × wheelbase  (very conservative for wet grass)
```

---

## 2. Kinematic Constraints <a name="kinematics"></a>

### Differential Drive Kinematics

**Forward kinematics:**
```cpp
// Given wheel velocities, compute robot velocity
void forwardKinematics(
    double v_left,  // Left wheel velocity [m/s]
    double v_right, // Right wheel velocity [m/s]
    double wheelbase, // Distance between wheels [m]
    double& v_linear,  // Output: linear velocity [m/s]
    double& v_angular) // Output: angular velocity [rad/s]
{
    v_linear = (v_left + v_right) / 2.0;
    v_angular = (v_right - v_left) / wheelbase;
}
```

**Instantaneous turn radius:**
```cpp
double getTurnRadius(double v_linear, double v_angular)
{
    if (std::abs(v_angular) < 1e-6) {
        return std::numeric_limits<double>::infinity(); // Straight line
    }
    return std::abs(v_linear / v_angular);
}
```

**Inverse kinematics with radius constraint:**
```cpp
// Given desired twist, compute wheel velocities with constraints
bool inverseKinematicsWithConstraints(
    double v_linear_desired,
    double v_angular_desired,
    double wheelbase,
    double min_turn_radius,
    double& v_left,
    double& v_right)
{
    // Compute desired turn radius
    double radius = getTurnRadius(v_linear_desired, v_angular_desired);

    // Check if radius is too small
    if (radius < min_turn_radius) {
        // Reduce angular velocity to meet radius constraint
        v_angular_desired = v_linear_desired / min_turn_radius;

        // Adjust sign
        if (v_angular_desired < 0) {
            v_angular_desired = -std::abs(v_angular_desired);
        }
    }

    // Compute wheel velocities
    v_left = v_linear_desired - (v_angular_desired * wheelbase / 2.0);
    v_right = v_linear_desired + (v_angular_desired * wheelbase / 2.0);

    return true;
}
```

---

## 3. Minimum Turn Radius Constraint <a name="min-radius"></a>

### Implementing in diff_drive_controller

**New parameter file:** `diff_drive_controller_skid_aware.yaml`

```yaml
diff_drive_controller:
  ros__parameters:
    # Existing parameters
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]

    wheel_separation: 0.5  # meters
    wheel_radius: 0.1      # meters

    # NEW: Skid-aware parameters
    min_turn_radius: 0.5   # Minimum turn radius [m]
                           # Set to wheelbase for moderate constraint
                           # Set to 2*wheelbase for wet grass

    enable_turn_radius_constraint: true

    # Adaptive based on velocity (optional)
    # Higher speed → need larger radius
    velocity_dependent_radius: true
    radius_velocity_scale: 1.5  # R_min = base + scale*|v|
```

**Modified controller code:**

```cpp
// In diff_drive_controller.hpp
class DiffDriveController : public controller_interface::ControllerInterface
{
private:
    // Existing members...

    // NEW: Skid prevention
    double min_turn_radius_;
    bool enable_turn_radius_constraint_;
    bool velocity_dependent_radius_;
    double radius_velocity_scale_;

    // Apply turn radius constraint
    void applyTurnRadiusConstraint(
        double& linear_command,
        double& angular_command);
};
```

```cpp
// In diff_drive_controller.cpp

void DiffDriveController::applyTurnRadiusConstraint(
    double& linear_command,
    double& angular_command)
{
    if (!enable_turn_radius_constraint_) {
        return;
    }

    // Compute current turn radius
    if (std::abs(angular_command) < 1e-6) {
        return; // Straight line, no constraint needed
    }

    double current_radius = std::abs(linear_command / angular_command);

    // Compute effective minimum radius
    double effective_min_radius = min_turn_radius_;
    if (velocity_dependent_radius_) {
        // Larger radius needed at higher speeds
        effective_min_radius += radius_velocity_scale_ * std::abs(linear_command);
    }

    // Check if violating constraint
    if (current_radius < effective_min_radius) {
        // Option 1: Reduce angular velocity (maintain linear velocity)
        double sign = (angular_command > 0) ? 1.0 : -1.0;
        angular_command = sign * std::abs(linear_command) / effective_min_radius;

        RCLCPP_DEBUG(
            get_node()->get_logger(),
            "Turn radius constraint applied: R=%.2f < R_min=%.2f, reduced omega to %.2f",
            current_radius, effective_min_radius, angular_command);
    }
}
```

**Integration into update loop:**

```cpp
controller_interface::return_type DiffDriveController::update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // ... existing code to get reference commands ...

    double linear_command = // ... from subscriber
    double angular_command = // ... from subscriber

    // NEW: Apply turn radius constraint BEFORE speed limiting
    applyTurnRadiusConstraint(linear_command, angular_command);

    // Then apply existing speed/acceleration limits
    limiter_linear_->limit(linear_command, last_linear, second_to_last_linear, period.seconds());
    limiter_angular_->limit(angular_command, last_angular, second_to_last_angular, period.seconds());

    // ... rest of update ...
}
```

---

## 4. Implementation in TEB Planner <a name="teb-implementation"></a>

### Configure TEB for Minimum Radius

**Configuration file:** `teb_skid_steer.yaml`

```yaml
TebLocalPlannerROS:
  # ===== ROBOT CONFIGURATION =====
  footprint_model:
    type: "circular"
    radius: 0.4  # Robot radius + safety margin

  # ===== KINEMATIC CONSTRAINTS =====
  # Standard constraints
  max_vel_x: 0.5
  max_vel_x_backwards: 0.2
  max_vel_theta: 0.8
  acc_lim_x: 0.3
  acc_lim_theta: 0.3

  # NEW: Minimum turn radius constraint
  min_turning_radius: 0.5  # ← KEY PARAMETER
                           # Set based on wheelbase and surface

  # Wheelbase for skid-steer kinematics
  wheelbase: 0.5

  # ===== TRAJECTORY OPTIMIZATION =====
  # Penalize sharp turns
  weight_kinematics_turning_radius: 5.0  # ← Increase this
                                          # Default: 1.0
                                          # Higher = smoother turns

  # Don't allow pure rotation
  allow_init_with_backwards_motion: false

  # ===== OBSTACLES =====
  min_obstacle_dist: 1.0  # Conservative for skid-steer

  # ===== PERFORMANCE =====
  no_inner_iterations: 5
  no_outer_iterations: 4
```

### How TEB Uses min_turning_radius

**TEB cost function includes:**
```cpp
// Pseudo-code from TEB planner internals

double computeTurnRadiusCost(const PoseSE2& pose1,
                              const PoseSE2& pose2,
                              double min_radius,
                              double weight)
{
    // Compute radius of turn between consecutive poses
    double dx = pose2.x() - pose1.x();
    double dy = pose2.y() - pose1.y();
    double dtheta = pose2.theta() - pose1.theta();

    // Approximate turn radius
    double chord_length = std::sqrt(dx*dx + dy*dy);
    double radius = chord_length / (2.0 * std::sin(dtheta / 2.0));

    // Penalize if below minimum
    if (radius < min_radius) {
        double violation = min_radius - radius;
        return weight * violation * violation;  // Quadratic penalty
    }

    return 0.0;  // No penalty if above minimum
}
```

**Result:** TEB automatically generates trajectories that respect minimum turn radius!

---

## 5. Custom Planner with ICR Constraints <a name="custom-planner"></a>

### Simple Path Smoother for Skid-Steer

**Purpose:** Post-process a global path to remove sharp turns

```cpp
#include <vector>
#include <cmath>
#include <algorithm>

struct Pose2D {
    double x, y, theta;
};

class SkidSteerPathSmoother {
public:
    SkidSteerPathSmoother(double min_radius, double wheelbase)
        : min_radius_(min_radius), wheelbase_(wheelbase) {}

    std::vector<Pose2D> smoothPath(const std::vector<Pose2D>& input_path)
    {
        if (input_path.size() < 3) {
            return input_path;  // Can't smooth short paths
        }

        std::vector<Pose2D> smoothed;
        smoothed.push_back(input_path.front());

        for (size_t i = 1; i < input_path.size() - 1; ++i) {
            const Pose2D& prev = input_path[i-1];
            const Pose2D& curr = input_path[i];
            const Pose2D& next = input_path[i+1];

            // Compute turn radius at current point
            double radius = computeTurnRadius(prev, curr, next);

            if (radius < min_radius_) {
                // Insert intermediate points to increase radius
                auto extra_points = insertIntermediatePoints(prev, curr, next);
                smoothed.insert(smoothed.end(), extra_points.begin(), extra_points.end());
            } else {
                smoothed.push_back(curr);
            }
        }

        smoothed.push_back(input_path.back());
        return smoothed;
    }

private:
    double min_radius_;
    double wheelbase_;

    double computeTurnRadius(const Pose2D& p1, const Pose2D& p2, const Pose2D& p3)
    {
        // Use Menger curvature formula
        double ax = p2.x - p1.x;
        double ay = p2.y - p1.y;
        double bx = p3.x - p2.x;
        double by = p3.y - p2.y;

        double cross = ax * by - ay * bx;
        double a_len = std::sqrt(ax*ax + ay*ay);
        double b_len = std::sqrt(bx*bx + by*by);
        double c_len = std::sqrt((p3.x-p1.x)*(p3.x-p1.x) + (p3.y-p1.y)*(p3.y-p1.y));

        if (std::abs(cross) < 1e-6) {
            return std::numeric_limits<double>::infinity();  // Straight line
        }

        // Radius of circumscribed circle
        double radius = (a_len * b_len * c_len) / (2.0 * std::abs(cross));
        return radius;
    }

    std::vector<Pose2D> insertIntermediatePoints(
        const Pose2D& p1, const Pose2D& p2, const Pose2D& p3)
    {
        std::vector<Pose2D> points;

        // Insert point between p1 and p2
        Pose2D mid1;
        mid1.x = (p1.x + p2.x) / 2.0;
        mid1.y = (p1.y + p2.y) / 2.0;
        mid1.theta = std::atan2(mid1.y - p1.y, mid1.x - p1.x);
        points.push_back(mid1);

        // Keep original p2
        points.push_back(p2);

        // Insert point between p2 and p3
        Pose2D mid2;
        mid2.x = (p2.x + p3.x) / 2.0;
        mid2.y = (p2.y + p3.y) / 2.0;
        mid2.theta = std::atan2(p3.y - mid2.y, p3.x - mid2.x);
        points.push_back(mid2);

        return points;
    }
};
```

**Usage in ROS2 node:**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

class PathSmootherNode : public rclcpp::Node {
public:
    PathSmootherNode() : Node("path_smoother_skid_steer")
    {
        // Parameters
        this->declare_parameter("min_turn_radius", 0.5);
        this->declare_parameter("wheelbase", 0.5);

        double min_radius = this->get_parameter("min_turn_radius").as_double();
        double wheelbase = this->get_parameter("wheelbase").as_double();

        smoother_ = std::make_unique<SkidSteerPathSmoother>(min_radius, wheelbase);

        // Subscribers and publishers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "input_path", 10,
            std::bind(&PathSmootherNode::pathCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("smoothed_path", 10);
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // Convert to internal format
        std::vector<Pose2D> input_poses;
        for (const auto& pose_stamped : msg->poses) {
            Pose2D p;
            p.x = pose_stamped.pose.position.x;
            p.y = pose_stamped.pose.position.y;
            // Extract yaw from quaternion
            p.theta = tf2::getYaw(pose_stamped.pose.orientation);
            input_poses.push_back(p);
        }

        // Smooth the path
        auto smoothed = smoother_->smoothPath(input_poses);

        // Convert back and publish
        nav_msgs::msg::Path smoothed_msg;
        smoothed_msg.header = msg->header;
        for (const auto& p : smoothed) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg->header;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            ps.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), p.theta));
            smoothed_msg.poses.push_back(ps);
        }

        path_pub_->publish(smoothed_msg);
    }

    std::unique_ptr<SkidSteerPathSmoother> smoother_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};
```

---

## 6. Complete Code Examples <a name="code-examples"></a>

### Example 1: Modified diff_drive_controller

**File structure:**
```
diff_drive_controller/
├── include/diff_drive_controller/
│   ├── diff_drive_controller.hpp
│   └── turn_radius_constraint.hpp  ← NEW
├── src/
│   ├── diff_drive_controller.cpp
│   └── turn_radius_constraint.cpp  ← NEW
└── config/
    └── skid_aware_params.yaml      ← NEW
```

**turn_radius_constraint.hpp:**

```cpp
#ifndef DIFF_DRIVE_CONTROLLER__TURN_RADIUS_CONSTRAINT_HPP_
#define DIFF_DRIVE_CONTROLLER__TURN_RADIUS_CONSTRAINT_HPP_

#include <cmath>
#include <algorithm>

namespace diff_drive_controller
{

class TurnRadiusConstraint
{
public:
    struct Parameters {
        double min_radius{0.5};           // Minimum turn radius [m]
        double wheelbase{0.5};            // Robot wheelbase [m]
        bool velocity_dependent{true};    // Scale radius with velocity
        double velocity_scale{1.0};       // Scaling factor
        bool enable{true};                // Enable constraint
    };

    explicit TurnRadiusConstraint(const Parameters& params)
        : params_(params) {}

    /**
     * @brief Apply turn radius constraint to commanded velocities
     *
     * @param v_linear Linear velocity [m/s] (modified in place)
     * @param v_angular Angular velocity [rad/s] (modified in place)
     * @return true if constraint was applied
     */
    bool apply(double& v_linear, double& v_angular)
    {
        if (!params_.enable || std::abs(v_angular) < 1e-6) {
            return false;  // Straight line, no constraint needed
        }

        // Compute current turn radius
        double current_radius = std::abs(v_linear / v_angular);

        // Compute effective minimum radius
        double min_radius = params_.min_radius;
        if (params_.velocity_dependent) {
            // Higher speeds need larger turn radius for stability
            min_radius += params_.velocity_scale * std::abs(v_linear);
        }

        // Check constraint violation
        if (current_radius < min_radius) {
            // Reduce angular velocity to meet constraint
            // Keep linear velocity unchanged (could also reduce both)
            double sign = (v_angular > 0.0) ? 1.0 : -1.0;
            v_angular = sign * std::abs(v_linear) / min_radius;
            return true;
        }

        return false;
    }

    /**
     * @brief Get maximum angular velocity for given linear velocity
     */
    double getMaxAngularVelocity(double v_linear) const
    {
        double min_radius = params_.min_radius;
        if (params_.velocity_dependent) {
            min_radius += params_.velocity_scale * std::abs(v_linear);
        }
        return std::abs(v_linear) / min_radius;
    }

    /**
     * @brief Check if velocities satisfy constraint
     */
    bool satisfiesConstraint(double v_linear, double v_angular) const
    {
        if (std::abs(v_angular) < 1e-6) {
            return true;  // Straight line always OK
        }

        double current_radius = std::abs(v_linear / v_angular);
        double min_radius = params_.min_radius;
        if (params_.velocity_dependent) {
            min_radius += params_.velocity_scale * std::abs(v_linear);
        }

        return current_radius >= min_radius;
    }

    void setParameters(const Parameters& params) { params_ = params; }
    const Parameters& getParameters() const { return params_; }

private:
    Parameters params_;
};

}  // namespace diff_drive_controller

#endif  // DIFF_DRIVE_CONTROLLER__TURN_RADIUS_CONSTRAINT_HPP_
```

### Example 2: Launch File Configuration

**skid_steer_mower.launch.py:**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'min_turn_radius',
            default_value='0.5',
            description='Minimum turn radius for skid-steer [m]'),

        DeclareLaunchArgument(
            'surface_type',
            default_value='dry_grass',
            choices=['dry_grass', 'wet_grass', 'concrete'],
            description='Surface type for adaptive parameters'),

        # Diff drive controller with turn radius constraint
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--controller-manager', '/controller_manager',
                '--param-file', LaunchConfiguration('controller_config')
            ],
            parameters=[{
                'min_turn_radius': LaunchConfiguration('min_turn_radius'),
                'enable_turn_radius_constraint': True,
                'velocity_dependent_radius': True,
            }]
        ),

        # TEB local planner with skid-steer constraints
        Node(
            package='nav2_bringup',
            executable='navigation_lifecycle_manager',
            name='navigation_lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['planner_server', 'controller_server']
            }]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[{
                'controller_plugins': ['FollowPath'],
                'FollowPath': {
                    'plugin': 'teb_local_planner::TebLocalPlannerROS',
                    'min_turning_radius': LaunchConfiguration('min_turn_radius'),
                    'weight_kinematics_turning_radius': 5.0,
                    # ... other TEB parameters
                }
            }]
        ),
    ])
```

---

## Summary

### Implementation Checklist

**For diff_drive_controller:**
- [x] Add `TurnRadiusConstraint` class
- [x] Add parameters: `min_turn_radius`, `velocity_dependent_radius`
- [x] Apply constraint in `update_and_write_commands()`
- [x] Test on flat surface, then grass

**For TEB planner:**
- [x] Set `min_turning_radius` parameter
- [x] Increase `weight_kinematics_turning_radius`
- [x] Test generated trajectories

**For custom path smoother:**
- [x] Implement `SkidSteerPathSmoother`
- [x] Create ROS2 node wrapper
- [x] Insert into navigation pipeline

### Expected Results

**Before (no constraints):**
- Sharp turns, rotation in place
- Skid marks on grass
- Poor odometry during turns

**After (with constraints):**
- Smooth, gentle turns
- Minimal grass damage
- Better odometry accuracy
- Slightly longer paths (trade-off)

### Next Steps

1. Tune `min_turn_radius` for your lawn conditions
2. Implement surface-adaptive control (next document!)
3. Test with IMU to detect actual slip
4. Integrate with odometry correction

Ready to implement surface-adaptive control next?
