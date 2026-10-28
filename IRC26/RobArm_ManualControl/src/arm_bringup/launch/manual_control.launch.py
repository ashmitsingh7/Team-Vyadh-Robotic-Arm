"""Control stack only: manual_control_node + arm_state_monitor (no bridge, no teleop).

Normally you launch mock_control or hardware_control instead, which include this file.
Run the keyboard in its own terminal (it needs a real TTY):
    ros2 run arm_teleop keyboard_teleop_node --ros-args --params-file <share>/arm_bringup/config/teleop.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('arm_bringup'), 'config')
    return LaunchDescription([
        Node(package='arm_control', executable='manual_control_node', name='manual_control_node',
             output='screen', parameters=[os.path.join(cfg, 'channels.yaml'), os.path.join(cfg, 'control.yaml')]),
        Node(package='arm_control', executable='arm_state_monitor', name='arm_state_monitor',
             output='screen', parameters=[os.path.join(cfg, 'control.yaml')]),
    ])
