"""mock_arm_bridge: stands in for the serial bridge so the ROS pipeline runs with NO hardware.

Same ROS interface as serial_arm_bridge: subscribes /arm/command, publishes /arm/state and
/arm/diagnostics. Everything it publishes is SIMULATED (`simulated = true`); it must never be
read as a hardware measurement, and it does not model ramping (that belongs to the STM32).

Fault injection while running (no restart needed):
  ros2 param set /mock_arm_bridge simulate_comm_loss true
  ros2 param set /mock_arm_bridge simulate_invalid_sensor_channel 2
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from arm_interfaces.msg import ArmCommand, ArmDiagnostics, ArmState

from arm_hardware.mock_bridge_core import MockBridgeCore
from arm_hardware.msg_convert import stats_to_diagnostics_msg, telemetry_to_state_msg

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # pragma: no cover
    ExternalShutdownException = KeyboardInterrupt

_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=10)


class MockArmBridge(Node):
    def __init__(self):
        super().__init__('mock_arm_bridge')
        self.declare_parameter('telemetry_rate_hz', 20.0)
        self.declare_parameter('diagnostics_rate_hz', 2.0)
        self.declare_parameter('sim_command_timeout_s', 0.5)
        self.declare_parameter('telemetry_stale_after_s', 0.5)
        self.declare_parameter('simulate_comm_loss', False)
        self.declare_parameter('simulate_invalid_sensor_channel', 0)
        self._core = MockBridgeCore(float(self.get_parameter('sim_command_timeout_s').value),
                                    float(self.get_parameter('telemetry_stale_after_s').value))
        self._last_link = None
        self._state_pub = self.create_publisher(ArmState, '/arm/state', _RELIABLE)
        self._diag_pub = self.create_publisher(ArmDiagnostics, '/arm/diagnostics', _RELIABLE)
        self.create_subscription(ArmCommand, '/arm/command', self._on_command, _RELIABLE)
        self.create_timer(1.0 / float(self.get_parameter('telemetry_rate_hz').value), self._on_telemetry_timer)
        self.create_timer(1.0 / float(self.get_parameter('diagnostics_rate_hz').value), self._publish_diagnostics)
        self.get_logger().warning(
            '*** MOCK MODE *** no hardware is connected. All /arm/state and /arm/diagnostics data is '
            'SIMULATED (simulated=true). Ramping is NOT modelled; nothing here shows real motor behaviour.')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _flush(self) -> None:
        for text in self._core.drain_events():
            self.get_logger().warning(f'[mock] {text}', throttle_duration_sec=1.0)

    def _on_command(self, msg: ArmCommand) -> None:
        self._core.on_command(msg, self._now())
        self._flush()

    def _on_telemetry_timer(self) -> None:
        now = self._now()
        self._core.set_comm_loss(bool(self.get_parameter('simulate_comm_loss').value), now)
        self._core.set_invalid_sensor_channel(int(self.get_parameter('simulate_invalid_sensor_channel').value))
        data = self._core.tick(now)
        if data is not None:
            self._state_pub.publish(telemetry_to_state_msg(data, self.get_clock().now().to_msg(), simulated=True))
        if self._core.stats.link_state != self._last_link:   # publish diagnostics on link change
            self._last_link = self._core.stats.link_state
            self._publish_diagnostics()
        self._flush()

    def _publish_diagnostics(self) -> None:
        self._diag_pub.publish(stats_to_diagnostics_msg(self._core.stats, self.get_clock().now().to_msg()))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MockArmBridge()
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
