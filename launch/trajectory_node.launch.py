#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare(package='slam_ros_node').find('slam_ros_node')
    
    # Declare launch arguments
    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value=os.path.join(pkg_share, 'example_trajectory.json'),
        description='Path to the JSON trajectory file'
    )
    
    target_service_arg = DeclareLaunchArgument(
        'target_service',
        default_value='/get_plan',
        description='Name of the target service to call'
    )
    
    send_frequency_arg = DeclareLaunchArgument(
        'send_frequency',
        default_value='1.0',
        description='Frequency to send trajectory (Hz)'
    )
    
    auto_send_arg = DeclareLaunchArgument(
        'auto_send',
        default_value='false',
        description='Whether to automatically send trajectory periodically'
    )
    
    # Create the trajectory node
    trajectory_node = Node(
        package='slam_ros_node',
        executable='TrajectoryNode_exe',
        name='trajectory_node',
        output='screen',
        parameters=[{
            'trajectory_file_path': LaunchConfiguration('trajectory_file'),
            'target_service_name': LaunchConfiguration('target_service'),
            'send_frequency': LaunchConfiguration('send_frequency'),
            'auto_send': LaunchConfiguration('auto_send'),
        }],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    return LaunchDescription([
        trajectory_file_arg,
        target_service_arg,
        send_frequency_arg,
        auto_send_arg,
        trajectory_node,
    ])
