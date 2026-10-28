"""keyboard_teleop_node: terminal keys -> /arm/user_input (ROS wrapper).

Run it directly in an interactive terminal (it needs a real TTY, so it cannot be started from
a launch file):   ros2 run arm_teleop keyboard_teleop_node

The node never touches hardware, serial or PWM. It only publishes operator INTENT.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from arm_interfaces.msg import UserInput

from arm_teleop.key_map import help_text
from arm_teleop.teleop_core import ACTION_ESTOP, Intent, TeleopCore
from arm_teleop.terminal_input import TerminalError, TerminalInput

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # pragma: no cover
    ExternalShutdownException = KeyboardInterrupt

_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=10)
ESTOP_COPIES = 3   # an ESTOP request is published several times; duplicates are harmless


class KeyboardTeleopNode(Node):
    def __init__(self, key_source=None):
        super().__init__('keyboard_teleop_node')
        self.declare_parameter('hold_window_s', 0.6)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('initial_level', 0.5)
        self.declare_parameter('level_step', 0.1)
        self.declare_parameter('min_level', 0.1)
        hold = float(self.get_parameter('hold_window_s').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate < 1.0 or 1.0 / rate >= hold:
            raise ValueError('publish_rate_hz must be >= 1 and its period shorter than hold_window_s')
        self._core = TeleopCore(hold_window_s=hold,
                                level=float(self.get_parameter('initial_level').value),
                                level_step=float(self.get_parameter('level_step').value),
                                min_level=float(self.get_parameter('min_level').value))
        self._keys = key_source
        self._seq = 0
        self._pub = self.create_publisher(UserInput, '/arm/user_input', _RELIABLE)
        self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(help_text(hold))

    @property
    def quit_requested(self) -> bool:
        return self._core.quit_requested

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish(self, intent: Intent) -> None:
        msg = UserInput()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sequence = self._seq
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        msg.action = intent.action
        msg.channel = intent.channel
        msg.direction = intent.direction
        msg.level = float(intent.level)
        self._pub.publish(msg)

    def _emit(self, intents) -> None:
        for intent in intents:
            copies = ESTOP_COPIES if intent.action == ACTION_ESTOP else 1
            for _ in range(copies):
                self._publish(intent)

    def _on_timer(self) -> None:
        now = self._now()
        if self._keys is not None:
            for key in self._keys.read_keys():
                self._emit(self._core.on_key(key, now))
        self._emit(self._core.tick(now))
        for text in self._core.drain_messages():
            self.get_logger().info(text)

    def send_shutdown_stop(self) -> None:
        """Best-effort normal STOP on exit (the STM32 timeout remains the real protection)."""
        self._emit(self._core.shutdown_intents())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        with TerminalInput() as terminal:
            node = KeyboardTeleopNode(terminal)
            while rclpy.ok() and not node.quit_requested:
                rclpy.spin_once(node, timeout_sec=0.05)
            node.send_shutdown_stop()
    except TerminalError as exc:
        print(f'keyboard_teleop_node: {exc}')
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None and rclpy.ok():
            node.send_shutdown_stop()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
