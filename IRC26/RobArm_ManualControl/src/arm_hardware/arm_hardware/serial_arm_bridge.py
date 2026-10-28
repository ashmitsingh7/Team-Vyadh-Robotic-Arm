"""serial_arm_bridge: Jetson <-> STM32 USB-serial bridge (ROS wrapper).

Same ROS interface as mock_arm_bridge. All protocol, link and safety-gating logic is in
serial_bridge_core.py / serial_protocol.py (pure Python, unit tested without ROS or hardware).

Never falls back to mock mode. If the port cannot be opened at startup the node logs an ERROR
and exits with a failure (`exit_on_initial_open_failure`, default true) so the operator sees it.
After a successful first connection it reconnects automatically, but a reconnect never resumes
motion (see command_gate.py).

Requires pyserial (apt: python3-serial). NOT tested against a real STM32 in this repository.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from arm_interfaces.msg import ArmCommand, ArmDiagnostics, ArmState

from arm_hardware.msg_convert import stats_to_diagnostics_msg, telemetry_to_state_msg
from arm_hardware.serial_bridge_core import BridgeConfig, SerialBridgeCore

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # pragma: no cover
    ExternalShutdownException = KeyboardInterrupt

_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=10)


def make_serial_opener(port: str, baudrate: int, write_timeout_s: float):
    """Return a function that opens the pyserial port non-blocking (read timeout 0)."""
    def opener():
        try:
            import serial  # pyserial
        except ImportError as exc:
            raise RuntimeError('pyserial is not installed (apt install python3-serial)') from exc
        return serial.Serial(port=port, baudrate=baudrate, timeout=0, write_timeout=write_timeout_s)
    return opener


class SerialArmBridge(Node):
    def __init__(self, opener=None):
        super().__init__('serial_arm_bridge')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('write_timeout_s', 0.1)
        self.declare_parameter('poll_rate_hz', 100.0)
        self.declare_parameter('diagnostics_rate_hz', 2.0)
        self.declare_parameter('telemetry_stale_after_s', 0.5)
        self.declare_parameter('reconnect_interval_s', 1.0)
        self.declare_parameter('exit_on_initial_open_failure', True)
        port = str(self.get_parameter('serial_port').value)
        baud = int(self.get_parameter('baudrate').value)
        if opener is None:
            opener = make_serial_opener(port, baud, float(self.get_parameter('write_timeout_s').value))
        self._core = SerialBridgeCore(opener, BridgeConfig(
            telemetry_stale_after_s=float(self.get_parameter('telemetry_stale_after_s').value),
            reconnect_interval_s=float(self.get_parameter('reconnect_interval_s').value)))
        self._last_link = None
        self._state_pub = self.create_publisher(ArmState, '/arm/state', _RELIABLE)
        self._diag_pub = self.create_publisher(ArmDiagnostics, '/arm/diagnostics', _RELIABLE)
        self.create_subscription(ArmCommand, '/arm/command', self._on_command, _RELIABLE)

        self.get_logger().info(f'HARDWARE bridge: opening {port} at {baud} baud (no mock fallback).')
        if not self._core.open(self._now()):
            self._flush()
            if bool(self.get_parameter('exit_on_initial_open_failure').value):
                raise RuntimeError(f'could not open serial port {port}; exiting (set '
                                   'exit_on_initial_open_failure:=false to keep retrying)')
            self.get_logger().error('continuing without a port; will keep retrying. All motion is blocked.')
        self.create_timer(1.0 / float(self.get_parameter('poll_rate_hz').value), self._on_poll)
        self.create_timer(1.0 / float(self.get_parameter('diagnostics_rate_hz').value), self._publish_diagnostics)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _flush(self) -> None:
        log = self.get_logger()
        for level, text in self._core.drain_events():
            {'info': log.info, 'warn': log.warning, 'error': log.error}.get(level, log.info)(text)

    def _on_command(self, msg: ArmCommand) -> None:
        self._core.on_command(msg, self._now())
        self._flush()

    def _on_poll(self) -> None:
        for data in self._core.poll(self._now()):
            self._state_pub.publish(telemetry_to_state_msg(data, self.get_clock().now().to_msg(), simulated=False))
        if self._core.stats.link_state != self._last_link:
            self._last_link = self._core.stats.link_state
            self._publish_diagnostics()
        self._flush()

    def _publish_diagnostics(self) -> None:
        self._diag_pub.publish(stats_to_diagnostics_msg(self._core.stats, self.get_clock().now().to_msg()))

    def destroy_node(self):
        self._core.close(send_stop=True)      # best effort; STM32 timeout is the real protection
        self._flush()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialArmBridge()
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
