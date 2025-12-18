from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('pfoe')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])

    # Joy node
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
        parameters=[params_file],
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
        joy_node,
        feature_extractor_node,
        joy_controller_node,
        logger_node,
        replay_node,
    ])
