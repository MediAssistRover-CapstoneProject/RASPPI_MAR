"""Hiwonder accel calibration + checked gyro bias + original complementary filter."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess,TimerAction,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(__file__).resolve().parents[2]
    base=(Path(get_package_share_directory('calibration')) if os.environ.get('need_compile')=='True'
          else Path('/home/ubuntu/ros2_ws/src/calibration'))
    calibration=base/'config/imu_calib.yaml'
    if not calibration.is_file():raise FileNotFoundError(calibration)
    accel=Node(package='imu_calib',executable='apply_calib',name='imu_calib',output='screen',
        parameters=[{'calib_file':str(calibration),'calibrate_gyros':False}],
        remappings=[('raw','/ros_robot_controller/imu_raw'),('corrected','/mediassist/imu_accel_calibrated')])
    gyro=ExecuteProcess(cmd=[str(root/'pi/.venv/bin/python'),'-m','mediassist.imu_stability'],output='screen',
        additional_env={'PYTHONPATH':str(root/'pi')+os.pathsep+os.environ.get('PYTHONPATH','')})
    filtered=Node(package='imu_complementary_filter',executable='complementary_filter_node',name='imu_filter',output='screen',
        parameters=[{'use_mag':False,'do_bias_estimation':True,'do_adaptive_gain':True,'publish_debug_topics':False}],
        remappings=[('/tf','tf'),('/imu/data_raw','/imu_corrected'),('imu/data','imu')])
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(target_action=gyro,on_exit=[EmitEvent(event=Shutdown(reason='IMU stability process exited'))])),
        TimerAction(period=5.,actions=[accel,gyro,filtered])])
