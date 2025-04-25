#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    franka_teleopeartion = get_package_share_directory('franka_teleopeartion')
    param_file = os.path.join(franka_custom_impedance_control, 'config', 'local_impedance_params.yaml')
    # Launch your impedance controller node
    franka_teleoperation_local_node_launcher = Node(
        package='franka_teleopeartion',                    # <-- replace with your package name
        executable='franka_teleoperation_local_cpp',   # <-- your node executable
        name='franka_teleoperation_local_node',
        output='screen',
        parameters=[param_file]
    )

    return LaunchDescription([
        franka_teleoperation_local_node_launcher
    ])
