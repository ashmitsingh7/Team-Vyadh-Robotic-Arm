"""Control stack + REAL serial bridge to the STM32. Never falls back to mock mode.

    ros2 launch arm_bringup hardware_control.launch.py serial_port:=/dev/ttyACM0 baudrate:=115200

If the port cannot be opened, serial_arm_bridge logs an ERROR and exits (visible in the launch
output); nothing silently substitutes simulated data. Commission with motor power disabled first.
This has NOT been tested against real STM32 firmware in this repository.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('arm_bringup')
    cfg = os.path.join(share, 'config')
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0', description='STM32 serial device'),
        DeclareLaunchArgument('baudrate', default_value='115200', description='must match the firmware'),
        LogInfo(msg='HARDWARE MODE: real serial bridge. Make sure motor power is disabled until the '
                    'link, watchdog and stop behaviour have been verified.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'manual_control.launch.py'))),
        Node(package='arm_hardware', executable='serial_arm_bridge', name='serial_arm_bridge',
             output='screen',
             parameters=[os.path.join(cfg, 'hardware.yaml'),
                         {'serial_port': LaunchConfiguration('serial_port'),
                          'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int)}]),
    ])
