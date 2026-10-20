"""Vendor SLAM with both chassis velocity inputs remapped behind the local gate."""
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vendor_slam_launch', default_value='/home/ubuntu/ros2_ws/src/slam/launch/slam.launch.py'),
        GroupAction([
            SetRemap(src='odom_publisher:controller/cmd_vel', dst='/mediassist/cmd_vel_safe'),
            SetRemap(src='odom_publisher:cmd_vel', dst='/mediassist/unused_cmd_vel'),
            SetRemap(src='odom_publisher:app/cmd_vel', dst='/mediassist/unused_app_cmd_vel'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(LaunchConfiguration('vendor_slam_launch')),
                launch_arguments={'sim': 'false', 'robot_name': '/', 'master_name': '/', 'use_joy': 'false'}.items())
        ])
    ])
