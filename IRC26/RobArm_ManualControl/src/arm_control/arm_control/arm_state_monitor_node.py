"""arm_state_monitor_node: operator-facing telemetry report (ROS wrapper).

Logs a periodic summary of /arm/state and /arm/diagnostics. Stale data is marked STALE and
simulated data is marked SIMULATED DATA; link/protocol problems are logged as warnings.
It does not command anything and does not replace firmware protection.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from arm_interfaces.msg import ArmDiagnostics, ArmState

from arm_control.arm_state_monitor import LINK_NAMES, assess, format_summary
from arm_control.control_config import DEFAULT_BLOCKING_FAULTS_MASK

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # pragma: no cover
    ExternalShutdownException = KeyboardInterrupt

_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=10)


class ArmStateMonitorNode(Node):
    def __init__(self):
        super().__init__('arm_state_monitor')
        self.declare_parameter('report_period_s', 1.0)
        self.declare_parameter('state_stale_after_s', 0.5)
        self.declare_parameter('diag_stale_after_s', 5.0)
        self.declare_parameter('blocking_faults_mask', DEFAULT_BLOCKING_FAULTS_MASK)
        self._state = self._diag = None
        self._state_rx = self._diag_rx = None
        self._last_link = None
        self.create_subscription(ArmState, '/arm/state', self._on_state, _RELIABLE)
        self.create_subscription(ArmDiagnostics, '/arm/diagnostics', self._on_diag, _RELIABLE)
        self.create_timer(float(self.get_parameter('report_period_s').value), self._report)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, msg: ArmState) -> None:
        self._state, self._state_rx = msg, self._now()

    def _on_diag(self, msg: ArmDiagnostics) -> None:
        self._diag, self._diag_rx = msg, self._now()
        if msg.link_state != self._last_link:
            self._last_link = msg.link_state
            text = (f'bridge link state -> {LINK_NAMES.get(msg.link_state, msg.link_state)}'
                    f'{" (SIMULATED)" if msg.simulated else ""}; tx={msg.frames_transmitted} '
                    f'rx_ok={msg.frames_received_valid} crc_err={msg.frames_crc_error} '
                    f'malformed={msg.frames_malformed} rejected_cmds={msg.commands_rejected}')
            (self.get_logger().info if msg.link_state == 2 else self.get_logger().warning)(text)

    def _report(self) -> None:
        a = assess(self._state, self._state_rx, self._diag, self._diag_rx, self._now(),
                   float(self.get_parameter('state_stale_after_s').value),
                   float(self.get_parameter('diag_stale_after_s').value),
                   int(self.get_parameter('blocking_faults_mask').value))
        if self._state is None:
            self.get_logger().warning('no /arm/state received yet (bridge not running or no telemetry)')
            return
        text = format_summary(self._state, a)
        if a.blocked_reason:
            text += f'\n  NEW MOTION BLOCKED: {a.blocked_reason}'
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArmStateMonitorNode()
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
