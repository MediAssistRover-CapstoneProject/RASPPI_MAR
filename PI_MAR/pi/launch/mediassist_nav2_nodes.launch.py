"""Humble navigation nodes with one isolated velocity output. No hardware or SLAM launch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params=LaunchConfiguration('params_file');nodes=[]
    definitions=[('nav2_controller','controller_server'),('nav2_smoother','smoother_server'),
        ('nav2_planner','planner_server'),('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator'),
        ('nav2_velocity_smoother','velocity_smoother')]
    for package,name in definitions:
        remaps=[('/tf','tf'),('/tf_static','tf_static')]
        if name in ('controller_server','behavior_server'):remaps.append(('cmd_vel','/mediassist/nav2_raw'))
        if name=='velocity_smoother':remaps.extend([('cmd_vel','/mediassist/nav2_raw'),('cmd_vel_smoothed','/mediassist/nav2_cmd_vel')])
        nodes.append(Node(package=package,executable=name,name=name,output='screen',parameters=[params],remappings=remaps))
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',
        output='screen',parameters=[{'use_sim_time':False,'autostart':True,'node_names':[n for _,n in definitions]}]))
    return LaunchDescription([DeclareLaunchArgument('params_file'),*nodes])
