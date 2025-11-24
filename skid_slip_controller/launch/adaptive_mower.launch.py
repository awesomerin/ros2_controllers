#!/usr/bin/env python3
"""
Launch file for the adaptive skid-slip controller for lawn mowers.

This launch file starts the adaptive_controller_node which monitors slip,
classifies surface types, and adapts control parameters in real-time.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for adaptive mower controller."""

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('skid_slip_controller'),
            'config',
            'default_params.yaml'
        ]),
        description='Path to the controller configuration file'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    log_level = LaunchConfiguration('log_level')

    # Adaptive controller node
    adaptive_controller_node = Node(
        package='skid_slip_controller',
        executable='adaptive_controller_node',
        name='adaptive_controller',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),           # Input from navigation/teleop
            ('cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped'),  # Output to hardware
            ('odom', '/diff_drive_controller/odom'),
            ('imu/data', '/imu/data'),
            ('motor_currents', '/hydraulic/motor_currents'),
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        log_level_arg,
        adaptive_controller_node
    ])
