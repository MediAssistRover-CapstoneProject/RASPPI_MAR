"""Vendor SLAM with both chassis velocity inputs remapped behind the local gate."""
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument,ExecuteProcess,TimerAction,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from pathlib import Path
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def engine_environment(root):
    # ROS message packages and vendor libraries live on the sourced PYTHONPATH.
    # ExecuteProcess.additional_env replaces a variable instead of extending it.
    paths=[str(root/'pi'),*os.environ.get('PYTHONPATH','').split(os.pathsep)]
    return {'PYTHONPATH':os.pathsep.join(dict.fromkeys(path for path in paths if path))}


def generate_launch_description():
    root=Path(__file__).resolve().parents[2]
    engine=ExecuteProcess(cmd=[str(root/'pi/.venv/bin/python'),'-m','mediassist.slam_session',
        '--config',LaunchConfiguration('worker_config')],output='screen',
        additional_env=engine_environment(root))
    return LaunchDescription([
        DeclareLaunchArgument('worker_config',default_value=str(root/'pi/config/mqtt.worker.json')),
        DeclareLaunchArgument('vendor_robot_launch', default_value='/home/ubuntu/ros2_ws/src/slam/launch/include/robot.launch.py'),
        GroupAction([
            SetRemap(src='odom_publisher:controller/cmd_vel', dst='/mediassist/cmd_vel_safe'),
            SetRemap(src='odom_publisher:cmd_vel', dst='/mediassist/unused_cmd_vel'),
            SetRemap(src='odom_publisher:app/cmd_vel', dst='/mediassist/unused_app_cmd_vel'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(LaunchConfiguration('vendor_robot_launch')),
                launch_arguments={'sim': 'false', 'robot_name': '/', 'master_name': '/', 'use_joy': 'false'}.items())
        ]),
        RegisterEventHandler(OnProcessExit(target_action=engine,on_exit=[EmitEvent(event=Shutdown(reason='SLAM session controller exited'))])),
        TimerAction(period=10.,actions=[engine])
    ])
