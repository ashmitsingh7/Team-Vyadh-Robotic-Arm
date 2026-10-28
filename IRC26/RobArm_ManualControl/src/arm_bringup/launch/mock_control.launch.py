"""Control stack + MOCK bridge: exercise the whole ROS pipeline with no hardware.

All /arm/state and /arm/diagnostics data is SIMULATED (simulated=true). Ramping is not modelled.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('arm_bringup')
    cfg = os.path.join(share, 'config')
    return LaunchDescription([
        LogInfo(msg='MOCK MODE: simulated bridge, no hardware. Start the keyboard in another terminal: '
                    'ros2 run arm_teleop keyboard_teleop_node'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'manual_control.launch.py'))),
        Node(package='arm_hardware', executable='mock_arm_bridge', name='mock_arm_bridge',
             output='screen', parameters=[os.path.join(cfg, 'hardware.yaml')]),
    ])
