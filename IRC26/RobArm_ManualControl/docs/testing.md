# Testing

## Levels

| Level | Where it can run | Proves | Does **not** prove |
|---|---|---|---|
| Static interface check (`tests/check_interfaces.py`) | any Python 3 | `.msg` files parse, are listed in CMake, use known types, have bounded 7-channel arrays, safe zero defaults, power-of-two flag bits, `NUM_CHANNELS` matches `channels.py` | That `rosidl` accepts them or that generated code works |
| Plain-Python unit tests (`tests/`, `src/*/test/`) | any Python 3 | Channel table logic, checker behaviour, packages import | ROS runtime behaviour |
| `colcon build` / `colcon test` | Ubuntu 24.04 + ROS 2 Jazzy | Interfaces generate; packages build; ROS-side tests | Hardware behaviour |
| Mock-mode integration | Jazzy machine | ROS pipeline end to end with **simulated** data | Anything physical |
| Serial-simulator tests | Jazzy or plain Python | Protocol parsing/framing | STM32 behaviour |
| Hardware commissioning | Real arm, secured, drivers disabled first | Ramping, watchdog, stops, directions | — (not started) |

## Commands

Plain Python, from the repo root:

```bash
python3 tests/check_interfaces.py
python3 -m unittest discover -s tests -v
for p in arm_teleop arm_control arm_hardware; do
  (cd src/$p && python3 -m unittest discover -s test -v)
done
```

ROS (Jazzy machine) — see README for build and test commands. Inspect generated interfaces:

```bash
source install/setup.bash
ros2 interface list | grep arm_interfaces
ros2 interface show arm_interfaces/msg/ArmCommand
ros2 interface show arm_interfaces/msg/ArmState
```

## Fake-ROS harness and mutation checks

`tests/support/fake_ros.py` is a strict fake `rclpy`; `fake_msgs.py` generates message classes
from the real `.msg` files (type/range checked). It tests node wiring only; it is **not** ROS.
Pure cores (control, teleop, protocol, bridges) are tested directly. `FakeStm32` models no ramp.
Safety behaviours (motion gate, hold-off, polarity, ESTOP not blockable) were mutation-checked:
disabling each makes tests fail.

## Validation tests

The list of cases that must be unit-tested (malformed or missing channel data must never
produce motion) is in `docs/ros_interface_spec.md` §7.
