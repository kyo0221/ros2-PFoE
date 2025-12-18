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

    num_particles_arg = DeclareLaunchArgument(
        'num_particles',
        default_value='1000',
        description='Number of particles for pfoe'
    )

    # Joy node (gamepad)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'autorepeat_rate': 3.0,
        }]
    )

    # Feature extractor
    feature_extractor_node = Node(
        package='pfoe',
        executable='feature_extractor.py',
        name='feature_extractor',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'image_size': 85,
            'use_gpu': True,
        }],
        output='screen'
    )

    # Joy controller
    joy_controller_node = Node(
        package='pfoe',
        executable='joy_controller.py',
        name='joy_controller',
        parameters=[{
            'linear_scale': 0.2,
            'angular_scale': 0.098,  # 3.14/32
            'button_deadman': 0,
            'button_level_up': 7,
            'button_level_down': 6,
            'axis_linear': 1,
            'axis_angular': 0,
        }],
        output='screen'
    )

    # Logger
    logger_node = Node(
        package='pfoe',
        executable='logger.py',
        name='logger',
        parameters=[{
            'bag_directory': os.path.expanduser('~/.ros/pfoe_bags'),
        }],
        output='screen'
    )

    # Replay node
    replay_node = Node(
        package='pfoe',
        executable='replay',
        name='replay',
        parameters=[{
            'num_particles': LaunchConfiguration('num_particles'),
            'loop_rate': 10.0,
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        model_path_arg,
        num_particles_arg,
        joy_node,
        feature_extractor_node,
        joy_controller_node,
        logger_node,
        replay_node,
    ])
