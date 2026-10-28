"""manual_control_node: /arm/user_input -> validated /arm/command (ROS wrapper).

All decisions live in manual_control_core.py / command_validation.py (pure Python, unit
tested without ROS). This file only moves messages, parameters and time in and out.

Subscribes: /arm/user_input (UserInput), /arm/state (ArmState), /arm/diagnostics (ArmDiagnostics)
Publishes:  /arm/command (ArmCommand) at `command_rate_hz`, always a full 7-channel snapshot.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from arm_interfaces.msg import ArmCommand, ArmDiagnostics, ArmState, MotorCommand, UserInput

from arm_control.arm_state_monitor import assess
from arm_control.channels import CHANNEL_COUNT
from arm_control.command_validation import validate_arm_state
from arm_control.control_config import ControlConfig, load_config, validate_config
from arm_control.manual_control_core import ManualControlCore

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # pragma: no cover - older distros
    ExternalShutdownException = KeyboardInterrupt

_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=10)


def declare_control_parameters(node: Node) -> None:
    """Declare every parameter with its placeholder default (see arm_bringup/config/*.yaml)."""
    d = ControlConfig()
    node.declare_parameter('command_rate_hz', d.command_rate_hz)
    node.declare_parameter('input_timeout_s', d.input_timeout_s)
    node.declare_parameter('require_fresh_state', d.require_fresh_state)
    node.declare_parameter('state_stale_after_s', d.state_stale_after_s)
    node.declare_parameter('diag_stale_after_s', d.diag_stale_after_s)
    node.declare_parameter('blocking_faults_mask', d.blocking_faults_mask)
    node.declare_parameter('rearm_repeat', d.rearm_repeat)
    node.declare_parameter('stop_repeat', d.stop_repeat)
    for n in range(1, CHANNEL_COUNT + 1):
        c = d.channels[n]
        p = f'channels.m{n}.'
        node.declare_parameter(p + 'polarity', c.polarity)
        node.declare_parameter(p + 'max_demand', c.max_demand)
        node.declare_parameter(p + 'enabled', c.enabled)
        node.declare_parameter(p + 'joint_limits_enabled', c.joint_limits_enabled)
        node.declare_parameter(p + 'joint_min_rad', c.joint_min_rad)
        node.declare_parameter(p + 'joint_max_rad', c.joint_max_rad)
        node.declare_parameter(p + 'angle_sign', c.angle_sign)


class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')
        declare_control_parameters(self)
        cfg = load_config(lambda name: self.get_parameter(name).value)
        problems = validate_config(cfg)
        if problems:
            for p in problems:
                self.get_logger().error(f'invalid configuration: {p}')
            raise ValueError('invalid manual_control_node configuration: ' + '; '.join(problems))
        self._cfg = cfg
        self._core = ManualControlCore(cfg)
        self._state = self._diag = None
        self._state_rx = self._diag_rx = None

        self._pub = self.create_publisher(ArmCommand, '/arm/command', _RELIABLE)
        self.create_subscription(UserInput, '/arm/user_input', self._on_user_input, _RELIABLE)
        self.create_subscription(ArmState, '/arm/state', self._on_state, _RELIABLE)
        self.create_subscription(ArmDiagnostics, '/arm/diagnostics', self._on_diag, _RELIABLE)
        self.create_timer(1.0 / cfg.command_rate_hz, self._on_timer)
        self.get_logger().info(
            f'manual_control_node up: {cfg.command_rate_hz:.0f} Hz snapshots, input timeout '
            f'{cfg.input_timeout_s:.2f} s, require_fresh_state={cfg.require_fresh_state}. '
            'Polarity/limits are UNCALIBRATED placeholders. STM32 owns ramping and the '
            'hardware watchdog; this node only requests.')
        if not cfg.require_fresh_state:
            self.get_logger().warning('require_fresh_state=false: motion is NOT gated on telemetry/link '
                                      'health (bring-up only).')

    # ---- time / state ------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _assessment(self, now: float):
        c = self._cfg
        return assess(self._state, self._state_rx, self._diag, self._diag_rx, now,
                      c.state_stale_after_s, c.diag_stale_after_s, c.blocking_faults_mask)

    def _flush_events(self) -> None:
        log = self.get_logger()
        for level, text in self._core.drain_events():
            {'info': log.info, 'warn': log.warning, 'error': log.error}.get(level, log.info)(text)

    # ---- callbacks ---------------------------------------------------------------
    def _on_user_input(self, msg: UserInput) -> None:
        now = self._now()
        self._core.handle_user_input(msg, now, self._assessment(now))
        self._flush_events()

    def _on_state(self, msg: ArmState) -> None:
        problems = validate_arm_state(msg)
        if problems:   # drop it: an invalid state must not refresh freshness
            self.get_logger().warning('ignoring malformed /arm/state: ' + '; '.join(problems),
                                      throttle_duration_sec=2.0)
            return
        self._state, self._state_rx = msg, self._now()

    def _on_diag(self, msg: ArmDiagnostics) -> None:
        self._diag, self._diag_rx = msg, self._now()

    def _on_timer(self) -> None:
        now = self._now()
        data = self._core.tick(now, self._assessment(now))
        out = ArmCommand()
        out.header.stamp = self.get_clock().now().to_msg()
        out.sequence = data.sequence
        out.command_type = data.command_type
        out.motors = [MotorCommand(channel=m.channel, direction=m.direction, demand=float(m.demand))
                      for m in data.motors]
        self._pub.publish(out)
        self._flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ManualControlNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
