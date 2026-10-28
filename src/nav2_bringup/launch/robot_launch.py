from launch import LaunchDescription
from launch_ros.actions import Node

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='plc_communication',
            executable='plc_communication.py',
            name='Plc_communication',
            output='screen'
        ),
        Node(
            package='motor_control',
            executable='motor_control.py',
            name='odom_calculator',
            output='screen'
        ),
        Node(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_node",
            name="sick_safetyscanners2_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "base_scan",
                 "sensor_ip": "192.168.2.226",
                 "host_ip": "192.168.2.240",
                 "interface_ip": "0.0.0.0",
                 "host_udp_port": 0,
                 "channel": 0,
                 "channel_enabled": True,
                 "skip": 0,
                 "angle_start": -1.6,
                 "angle_end": 1.6,
                 "time_offset": 0.0,
                 "general_system_state": True,
                 "derived_settings": True,
                 "measurement_data": True,
                 "intrusion_data": True,
                 "application_io_data": True,
                 "use_persistent_config": False,
                 "min_intensities": 0.0}
            ]
        ),
        Node(
            package='simple_command',
            executable='nav2_command.py',
            #name='nav2_commander',
            output='screen'
        )
    ])
