# Team Vyadh — IRC 2027 Robotic Arm (ROS 2 software)

ROS 2 manual-control stack for the 7-channel arm. A Jetson Orin Nano runs ROS 2 and
sends motion **requests**; an STM32 NUCLEO-F446ZE owns ramping, the command watchdog and
motor safety. Architecture: `docs/Team_Vyadh_IRC_2027_Robotic_Arm_Data_Flow_Architecture.docx`.

## Status (ROS side complete; firmware not written)

| Item | State |
|---|---|
| `arm_interfaces` messages | Written; **not yet built with colcon** |
| Teleop, control core, state monitor, mock bridge, serial protocol, serial bridge, fake STM32, launch files, configs | Written; tested only in plain Python with a fake ROS harness |
| Real `colcon build` / `colcon test` / `rclpy` / launch files | **Never run** — verify on Ubuntu 24.04 + Jazzy |
| STM32 firmware | **Not written.** Requirements in `firmware/README.md` |
| Hardware | **Nothing tested on hardware.** No ramping or motor operation is claimed. |

The fake-ROS tests are not ROS tests. The serial protocol is a **draft** (`docs/serial_protocol.md`).

## Target environment

Ubuntu 24.04, ROS 2 **Jazzy**. (Verify the Jetson's JetPack/OS actually supports this.)

External dependencies:
- ROS 2 Jazzy (`ros-jazzy-ros-base` or `-desktop`) — provides `rclpy`, `rosidl`, `std_msgs`, `launch`, `launch_ros`
- `python3-colcon-common-extensions`, `python3-rosdep`
- `python3-pytest` (ROS Python test runner)
- `python3-serial` (pyserial) — serial bridge
- `python3-yaml`

## Layout

```
README.md  HOW_TO_RUN.md
src/
  arm_interfaces    messages (ament_cmake)
  arm_teleop        keyboard -> /arm/user_input
  arm_control       intent -> validated /arm/command, state monitor
  arm_hardware      mock + serial bridges, protocol, fake STM32
  arm_bringup       launch files + YAML configs
firmware/README.md  STM32 requirements (no code yet)
docs/               protocol, interface spec, ramping, manual control, testing, architecture
tests/              plain-Python tests and the .msg checker
```

Build, test and run commands: `HOW_TO_RUN.md`.

## Key decisions

- `ArmCommand` is a full seven-channel snapshot; see `docs/ros_interface_spec.md`.
- Normal STOP, emergency-stop request (ESTOP), communication-loss response and fault
  response are distinct. A software ESTOP is **not** a physical emergency stop.
- Polarity, demand limits and joint limits are uncalibrated placeholders in
  `arm_bringup/config/channels.yaml`.
- M5 and M6 are independent channels; no wrist roll/yaw mapping exists.

## Known limitations

- Package `license` and `maintainer` fields are TODO placeholders in every `package.xml`/`setup.py`.
- Nothing has been built with colcon or run under ROS yet.
- No firmware; no hardware test; polarity, limits, rates, baud and timeouts are placeholders.
- Software ESTOP is not a physical emergency stop.

Docs: `docs/manual_control.md`, `docs/ramping_behavior.md`.
