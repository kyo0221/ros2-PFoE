#!/usr/bin/env python3
"""
Launch file for pfoe teach and replay system
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = FindPackageShare('pfoe')

    # Path to params.yaml
    params_file = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])

    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([pkg_share, 'weights', 'placenet.pt']),
        description='Path to PlaceNet model file'
    )

    # Joy node (gamepad)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[params_file],
        output='screen'
    )

    # Feature extractor
    feature_extractor_node = Node(
        package='pfoe',
        executable='feature_extractor.py',
        name='feature_extractor',
        parameters=[
            params_file,
            {'model_path': LaunchConfiguration('model_path')}
        ],
        output='screen'
    )

    # Joy controller
    joy_controller_node = Node(
        package='pfoe',
        executable='joy_controller.py',
        name='joy_controller',
        parameters=[params_file],
        output='screen'
    )

    # Logger
    logger_node = Node(
        package='pfoe',
        executable='logger.py',
        name='logger',
        parameters=[params_file],
        output='screen'
    )

    # Replay node
    replay_node = Node(
        package='pfoe',
        executable='replay',
        name='replay',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        model_path_arg,
        joy_node,
        feature_extractor_node,
        joy_controller_node,
        logger_node,
        replay_node,
    ])
