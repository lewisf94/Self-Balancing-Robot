"""Bring up the real robot on a Linux SBC (e.g. Raspberry Pi 3B).

Starts the IMU driver, the motor driver and the balance controller from the
hardware parameter file. On an ESP32-S3 the balance loop runs as micro-ROS
firmware instead of these nodes; see firmware/ and docs/architecture.md.

Usage:
    ros2 launch sbr_bringup robot.launch.py
    ros2 launch sbr_bringup robot.launch.py use_rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = get_package_share_directory('sbr_bringup')
    pkg_description = get_package_share_directory('sbr_description')

    default_params = os.path.join(pkg_bringup, 'config', 'sbr_params.yaml')
    xacro_file = os.path.join(pkg_description, 'urdf', 'sbr.urdf.xacro')
    rviz_config = os.path.join(pkg_description, 'rviz', 'sbr.rviz')

    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=false']), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Node parameter file'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Open RViz alongside the robot'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description}]),

        # respawn: a crashed driver/controller must not silently end balancing.
        Node(package='sbr_drivers', executable='imu_node', output='screen',
             parameters=[params_file], respawn=True, respawn_delay=2.0),
        Node(package='sbr_drivers', executable='motor_node', output='screen',
             parameters=[params_file], respawn=True, respawn_delay=2.0),
        Node(package='sbr_control', executable='balance_controller_node',
             output='screen', parameters=[params_file],
             respawn=True, respawn_delay=2.0),

        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             condition=IfCondition(use_rviz)),
    ])
