# How to run

Target: Ubuntu 24.04, ROS 2 Jazzy. Nothing here has been run under real ROS yet.
Dependencies: ROS 2 Jazzy, `python3-colcon-common-extensions`, `python3-rosdep`, `python3-serial`, `python3-yaml`.

## Build
```bash
source /opt/ros/jazzy/setup.bash
cd RobArm_ManualControl            # this folder (contains src/)
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test && colcon test-result --verbose
```

## Plain-Python tests (no ROS)
```bash
python3 tests/check_interfaces.py
python3 -m unittest discover -s tests
for p in arm_teleop arm_control arm_hardware; do (cd src/$p && python3 -m unittest discover -s test); done
```

## Run (after `source install/setup.bash`)
```bash
# 1. Mock mode (simulated data, no hardware)
ros2 launch arm_bringup mock_control.launch.py
# second terminal (needs a real TTY):
ros2 run arm_teleop keyboard_teleop_node --ros-args --params-file src/arm_bringup/config/teleop.yaml

# 2. Serial bridge against the fake STM32 (no hardware)
python3 -m arm_hardware.fake_stm32            # prints /dev/pts/N
ros2 launch arm_bringup hardware_control.launch.py serial_port:=/dev/pts/N

# 3. Real hardware: only with authorization, firmware commissioned, physical E-stop in place
ros2 launch arm_bringup hardware_control.launch.py serial_port:=/dev/ttyACM0 baudrate:=115200
```
Keys: q/a w/s e/d r/f t/g y/h u/j = M1..M7 +/-, SPACE stop, x ESTOP request, Z rearm, +/- level, ? help, Q quit.
