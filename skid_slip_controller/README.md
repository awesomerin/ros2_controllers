# Skid-Slip Controller for Differential Drive Lawn Mowers

A ROS2 package providing surface-adaptive control for hydraulic differential drive lawn mowers. This controller monitors wheel slip, classifies surface conditions, and dynamically adapts control parameters to prevent excessive skidding and maintain safe operation on various terrains.

## Features

- **Real-time Slip Detection**: Compares encoder-based odometry with IMU measurements to detect wheel slip
- **Surface Classification**: Automatically identifies surface type (dry concrete, dry grass, wet grass, mud)
- **Adaptive Parameter Tuning**: Adjusts velocity limits, acceleration, and turn radius based on surface conditions
- **Turn Radius Constraints**: Enforces minimum turn radius to prevent sharp turns that cause skidding
- **Velocity-Dependent Safety**: Increases turn radius constraints at higher speeds
- **Comprehensive Unit Tests**: Full test coverage using Google Test framework

## Architecture

The package consists of four main components:

1. **SlipDetector** (`slip_detector.hpp/cpp`): Detects wheel slip by comparing encoder velocity with IMU-based velocity estimates. Uses moving average filtering for robustness.

2. **SurfaceClassifier** (`surface_classifier.hpp/cpp`): Classifies surface type based on slip ratio, motor current, vibration level, and velocity tracking error. Provides surface-specific control parameters.

3. **TurnRadiusConstraint** (`turn_radius_constraint.hpp/cpp`): Enforces minimum turn radius constraints to prevent excessive skidding during turns. Supports velocity-dependent scaling.

4. **AdaptiveControllerNode** (`adaptive_controller_node.hpp/cpp`): Main ROS2 node that integrates all components, subscribes to sensor data, and publishes adapted velocity commands.

## Dependencies

- ROS2 (Humble or later)
- C++17 compiler
- rclcpp
- geometry_msgs
- sensor_msgs
- nav_msgs
- std_msgs
- tf2
- Google Test (for unit tests)

## Building

### Clone the repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/awesomerin/ros2_controllers.git
cd ~/ros2_ws
```

### Build with colcon

```bash
# Build only this package
colcon build --packages-select skid_slip_controller

# Build with tests
colcon build --packages-select skid_slip_controller --cmake-args -DBUILD_TESTING=ON

# Source the workspace
source install/setup.bash
```

## Running Tests

```bash
# Run all tests
colcon test --packages-select skid_slip_controller

# View test results
colcon test-result --all --verbose

# Run specific test
./build/skid_slip_controller/test_slip_detector
./build/skid_slip_controller/test_surface_classifier
./build/skid_slip_controller/test_turn_radius_constraint
```

## Usage

### Launch the adaptive controller

```bash
ros2 launch skid_slip_controller adaptive_mower.launch.py
```

### Launch with custom configuration

```bash
ros2 launch skid_slip_controller adaptive_mower.launch.py \
  config_file:=/path/to/your/config.yaml \
  log_level:=debug
```

### Run the node directly

```bash
ros2 run skid_slip_controller adaptive_controller_node \
  --ros-args --params-file src/ros2_controllers/skid_slip_controller/config/default_params.yaml
```

## Topics

### Subscribed Topics

- **`cmd_vel_in`** (`geometry_msgs/Twist`): Input velocity commands from navigation or teleop
- **`odom`** (`nav_msgs/Odometry`): Encoder-based odometry from the robot
- **`imu/data`** (`sensor_msgs/Imu`): IMU data for velocity verification and vibration measurement
- **`motor_currents`** (`std_msgs/Float64MultiArray`): Hydraulic motor current measurements

### Published Topics

- **`cmd_vel_out`** (`geometry_msgs/Twist`): Adapted velocity commands sent to the hardware controller
- **`detected_surface`** (`std_msgs/String`): Currently detected surface type

## Configuration

Configuration is done via YAML file. See `config/default_params.yaml` for all available parameters.

### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `update_rate` | 20.0 | Control loop frequency (Hz) |
| `slip_threshold` | 0.2 | Slip ratio threshold for conservative behavior |
| `enable_adaptation` | true | Enable/disable adaptive control |
| `min_turn_radius` | 0.5 | Minimum turn radius (meters) |
| `velocity_dependent_radius` | true | Increase turn radius with velocity |
| `surface_classify_period` | 20 | Classify surface every N cycles |

### Slip Detector Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `velocity_threshold` | 0.01 | Minimum velocity for slip computation (m/s) |
| `filter_window_size` | 10 | Moving average window size |
| `confidence_velocity_scale` | 0.5 | Velocity for full confidence (m/s) |

## Parameter Tuning Guide

### 1. Adjust Update Rate

For hydraulic systems with 500-700ms response time:
- Start with `update_rate: 20.0` (50ms cycle)
- Can increase to 30-50 Hz if CPU allows
- Don't go below 10 Hz for safety

### 2. Tune Slip Threshold

Test on different surfaces and adjust:
- **Dry grass**: `slip_threshold: 0.15-0.20`
- **Wet grass**: `slip_threshold: 0.25-0.30`
- **Muddy conditions**: `slip_threshold: 0.35-0.45`

### 3. Configure Turn Radius

Based on your mower's wheelbase and tire width:
```yaml
min_turn_radius: 0.5              # Small mower
min_turn_radius: 1.0              # Medium commercial mower
min_turn_radius: 1.5              # Large commercial mower

velocity_scale: 1.0               # Cautious
velocity_scale: 0.5               # Balanced
velocity_scale: 0.0               # Aggressive (not recommended)
```

### 4. Calibrate Surface Classifier

To tune surface classification for your specific mower:

1. **Record sensor data** on known surfaces:
```bash
ros2 bag record /odom /imu/data /motor_currents
```

2. **Analyze the data** to find characteristic values:
   - Slip ratio on each surface
   - Motor currents on each surface
   - Vibration levels on each surface

3. **Modify classification thresholds** in `src/surface_classifier.cpp`:
```cpp
SurfaceType SurfaceClassifier::classify(const SensorReadings & readings)
{
  // Adjust these thresholds based on your data
  if (readings.slip_ratio > 0.5 && readings.motor_current_avg > 8.0) {
    return SurfaceType::MUD;
  }
  // ... adjust other conditions
}
```

4. **Adjust surface parameters** in `src/surface_classifier.cpp`:
```cpp
void SurfaceClassifier::initializeParameters()
{
  // Tune these values for your mower
  SurfaceParameters dry_grass;
  dry_grass.friction_coeff = 0.6;      // Adjust based on tire type
  dry_grass.max_acceleration = 0.5;     // Adjust for your hydraulics
  dry_grass.min_turn_radius = 0.5;      // Adjust for wheelbase
  dry_grass.max_velocity = 0.8;         // Adjust for safety
  // ...
}
```

## Integration with ROS2 Navigation Stack

To use with Nav2:

1. **Remap topics** in your launch file:
```python
remappings=[
    ('cmd_vel_in', '/cmd_vel'),
    ('cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped'),
]
```

2. **Chain controllers**: Navigation → Adaptive Controller → Hardware Controller
```
Nav2 → /cmd_vel → Adaptive Controller → /diff_drive_controller/cmd_vel_unstamped → Hardware
```

3. **Configure Nav2 planners** with appropriate safety margins:
```yaml
controller_server:
  ros__parameters:
    min_vel_x: 0.1
    max_vel_x: 0.8  # Match surface-dependent limits
    min_vel_theta: 0.2
    max_vel_theta: 1.0
```

## Hardware Integration

### Required Sensors

1. **Wheel Encoders**: For odometry (already present in most robots)
2. **IMU**: For velocity verification and vibration measurement
3. **Motor Current Sensors**: For hydraulic motor load measurement (optional but recommended)

### Sensor Placement

- **IMU**: Mount near the center of the robot, aligned with robot frame
- **Current Sensors**: Measure hydraulic motor current for all drive motors

### Calibration

1. **IMU Calibration**: Use your IMU manufacturer's calibration procedure
2. **Encoder Calibration**: Verify odometry accuracy on hard surface
3. **Current Sensors**: Record baseline currents on flat concrete

## Troubleshooting

### High slip detected on all surfaces

- Check IMU orientation and frame alignment
- Verify encoder scaling factors in odometry
- Ensure IMU data is properly integrated to velocity

### Surface classification always returns UNKNOWN

- Check that motor current topic is publishing
- Verify IMU vibration readings are reasonable (5-15 m/s² typical)
- Increase logging to debug: `log_level:=debug`

### Controller too conservative

- Decrease `slip_threshold` (try 0.15 instead of 0.2)
- Decrease `velocity_scale` for turn radius
- Adjust surface parameters in `surface_classifier.cpp`

### Controller too aggressive

- Increase `slip_threshold` (try 0.25 instead of 0.2)
- Increase `min_turn_radius`
- Enable `velocity_dependent_radius: true`

## Testing in Simulation

While this package is designed for real hardware, you can test the logic in simulation:

1. Launch your robot in Gazebo with IMU and odometry plugins
2. Add simulated motor current publisher (use joint effort sensors)
3. Launch the adaptive controller
4. Test on different simulated terrains with varying friction

## Performance Considerations

- **CPU Usage**: Surface classification runs at reduced rate (1-2 Hz) to minimize CPU load
- **Latency**: Total control latency ~50-100ms (sensor → processing → output)
- **Memory**: Slip detector maintains rolling window (~1 KB per detector)

## Future Enhancements

Potential improvements for future versions:

- [ ] Machine learning-based surface classifier
- [ ] Terrain-aware trajectory prediction
- [ ] Dynamic wheelbase adaptation (for articulated mowers)
- [ ] Integration with weather data for proactive adaptation
- [ ] Multi-layer surface detection (surface + subsurface conditions)
- [ ] ROS2 Lifecycle node support
- [ ] Parameter auto-tuning from collected data

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass: `colcon test --packages-select skid_slip_controller`
5. Submit a pull request

## License

This package is part of ros2_controllers and follows the same license (Apache 2.0).

## References

- [ROS2 Controllers Documentation](https://control.ros.org/)
- [Differential Drive Kinematics](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)
- [Slip Detection Methods](https://ieeexplore.ieee.org/document/8593868)
- [Terrain Classification for Autonomous Vehicles](https://journals.sagepub.com/doi/10.1177/0278364913509612)

## Support

For issues, questions, or contributions:
- GitHub Issues: https://github.com/awesomerin/ros2_controllers/issues
- ROS Discourse: https://discourse.ros.org/

## Authors

- Implementation: Claude (Anthropic)
- Concept and Testing: awesomerin
