"""Wrap stock navigation with node-scoped chassis remaps. No vendor files changed."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", description="Saved map basename, without .yaml"),
        DeclareLaunchArgument("vendor_launch", default_value="/home/ubuntu/ros2_ws/src/navigation/launch/navigation.launch.py"),
        GroupAction([
            SetRemap(src="odom_publisher:controller/cmd_vel", dst="/mediassist/cmd_vel_safe"),
            SetRemap(src="odom_publisher:cmd_vel", dst="/mediassist/unused_cmd_vel"),
            SetRemap(src="odom_publisher:app/cmd_vel", dst="/mediassist/unused_app_cmd_vel"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(LaunchConfiguration("vendor_launch")),
                launch_arguments={"map": LaunchConfiguration("map"), "sim": "false", "robot_name": "/", "master_name": "/", "use_teb": "false", "use_joy": "false"}.items()),
        ]),
    ])
