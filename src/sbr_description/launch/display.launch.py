"""Display the robot model in RViz with interactive joint sliders.

Usage:
    ros2 launch sbr_description display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('sbr_description')
    xacro_file = os.path.join(pkg, 'urdf', 'sbr.urdf.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'sbr.rviz')

    robot_description = Command(['xacro ', xacro_file, ' use_sim:=false'])
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start joint_state_publisher_gui'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
