"""Hiwonder hardware + standard Humble Nav2 using an explicit measured cart profile."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition,UnlessCondition
from launch_ros.actions import SetRemap


def generate_launch_description():
    nav2 = Path(get_package_share_directory('nav2_bringup')) / 'launch/bringup_launch.py'
    localization = nav2.parent/'localization_launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute saved map YAML path'),
        DeclareLaunchArgument('params_file', description='Absolute generated nav2.demo.yaml path'),
        DeclareLaunchArgument('localization_only',default_value='false'),
        DeclareLaunchArgument('vendor_robot_launch', default_value='/home/ubuntu/ros2_ws/src/slam/launch/include/robot.launch.py'),
        GroupAction([
            SetRemap(src='odom_publisher:controller/cmd_vel', dst='/mediassist/cmd_vel_safe'),
            SetRemap(src='odom_publisher:cmd_vel', dst='/mediassist/unused_cmd_vel'),
            SetRemap(src='odom_publisher:app/cmd_vel', dst='/mediassist/unused_app_cmd_vel'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(LaunchConfiguration('vendor_robot_launch')),
                launch_arguments={'sim': 'false', 'robot_name': '/', 'master_name': '/', 'use_joy': 'false'}.items()),
        ]),
        TimerAction(period=10.0, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav2)),condition=UnlessCondition(LaunchConfiguration('localization_only')), launch_arguments={
                'map': LaunchConfiguration('map'), 'params_file': LaunchConfiguration('params_file'),
                'namespace': '', 'use_namespace': 'false', 'slam': 'False', 'use_sim_time': 'false',
                'autostart': 'true', 'use_composition': 'False', 'use_respawn': 'False'}.items()),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(localization)),condition=IfCondition(LaunchConfiguration('localization_only')),launch_arguments={
                'map':LaunchConfiguration('map'),'params_file':LaunchConfiguration('params_file'),
                'namespace':'','use_sim_time':'false','autostart':'true','use_composition':'False'}.items())
        ]),
    ])
