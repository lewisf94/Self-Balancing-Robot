"""Spin up Gazebo, spawn the robot and start the balance controller.

Usage:
    ros2 launch sbr_simulation simulation.launch.py
    ros2 launch sbr_simulation simulation.launch.py use_rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('sbr_description')
    pkg_simulation = get_package_share_directory('sbr_simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_description, 'urdf', 'sbr.urdf.xacro')
    controllers_file = os.path.join(pkg_simulation, 'config', 'controllers.yaml')
    bridge_config = os.path.join(pkg_simulation, 'config', 'gz_bridge.yaml')
    balance_params = os.path.join(pkg_simulation, 'config', 'sim_balance.yaml')
    world_file = os.path.join(pkg_simulation, 'worlds', 'empty.sdf')
    rviz_config = os.path.join(pkg_description, 'rviz', 'sbr.rviz')

    use_rviz = LaunchConfiguration('use_rviz')

    robot_description = Command([
        'xacro ', xacro_file,
        ' use_sim:=true',
        ' controllers_config:=', controllers_file,
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file]}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'sbr', '-z', '0.06'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    wheel_effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_effort_controller', '--controller-manager', '/controller_manager'],
    )

    balance_controller = Node(
        package='sbr_control',
        executable='balance_controller_node',
        output='screen',
        parameters=[balance_params, {'use_sim_time': True}],
        remappings=[('/wheel_cmd', '/wheel_effort_controller/commands')],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Open RViz alongside Gazebo'),

        robot_state_publisher,
        gz_sim,
        bridge,
        spawn_robot,

        # Start controllers only once the robot is in the world, then start the
        # balance loop once the controllers are active.
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[wheel_effort_controller])),
        RegisterEventHandler(OnProcessExit(
            target_action=wheel_effort_controller,
            on_exit=[balance_controller])),

        rviz,
    ])
