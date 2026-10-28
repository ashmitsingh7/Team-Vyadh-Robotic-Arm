"""A minimal in-memory fake of the rclpy API surface the nodes use.

NOT ROS. It lets node wiring (parameters, publishers, subscriptions, timers, QoS objects,
logging, shutdown handling) run in a plain Python process with manual time. It mimics a few
real rclpy behaviours that catch bugs: declared parameters must match the type of their
default (a YAML integer for a double parameter raises), reading an undeclared parameter raises,
publishers reject messages of the wrong type, and message delivery is queued (asynchronous
relative to the publisher's callback) like a real executor.

Differences from real ROS that this cannot cover: DDS discovery/QoS matching, threading,
executors, real clocks, launch files, parameter services, signal handling.
"""
import sys
import types
from enum import Enum
from types import SimpleNamespace
from typing import Any, Callable, Dict, List, Optional

import yaml

import fake_msgs  # noqa: E402  (same directory; tests add tests/support to sys.path)

CURRENT_BUS: 'FakeBus' = None  # type: ignore


class ExternalShutdownException(Exception):
    pass


def flatten(d: Dict[str, Any], prefix: str = '') -> Dict[str, Any]:
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(flatten(v, f'{prefix}{k}.'))
        else:
            out[f'{prefix}{k}'] = v
    return out


class FakeClockTime:
    def __init__(self, seconds: float):
        self.nanoseconds = int(round(seconds * 1e9))

    def to_msg(self):
        from std_msgs.msg import Header  # noqa
        return fake_msgs.Time(self.nanoseconds // 10**9, self.nanoseconds % 10**9)


class FakeClock:
    def __init__(self, bus):
        self._bus = bus

    def now(self):
        return FakeClockTime(self._bus.now)


class FakeLogger:
    def __init__(self, bus, name):
        self._bus, self.name = bus, name

    def _rec(self, level, text, **kw):
        self._bus.logs.append((self._bus.now, self.name, level, str(text)))

    def debug(self, text, **kw):
        self._rec('debug', text)

    def info(self, text, **kw):
        self._rec('info', text)

    def warning(self, text, **kw):
        self._rec('warning', text)

    def warn(self, text, **kw):
        raise AssertionError('Logger.warn is deprecated in Jazzy; use warning()')

    def error(self, text, **kw):
        self._rec('error', text)


class FakePublisher:
    def __init__(self, bus, msg_type, topic):
        self._bus, self._type, self.topic = bus, msg_type, topic

    def publish(self, msg):
        if not isinstance(msg, self._type):
            raise TypeError(f'publisher on {self.topic} expects {self._type.__name__}, got {type(msg).__name__}')
        self._bus.enqueue(self.topic, msg)


class FakeBus:
    """Message bus + manual clock + timers shared by all fake nodes in one test."""

    def __init__(self):
        self.now = 0.0
        self.subs: Dict[str, List] = {}
        self.timers: List[dict] = []
        self.queue: List = []
        self.logs: List = []
        self.sent: Dict[str, List] = {}           # topic -> every message published
        self.overrides: Dict[str, Dict[str, Any]] = {}   # node name -> flat params from YAML
        self.nodes: Dict[str, 'Node'] = {}

    # ---- parameters ----
    def load_yaml(self, path):
        with open(path) as fh:
            data = yaml.safe_load(fh)
        for node_name, body in data.items():
            self.overrides.setdefault(node_name.lstrip('/'), {}).update(flatten(body['ros__parameters']))

    # ---- messaging ----
    def enqueue(self, topic, msg):
        import copy
        self.sent.setdefault(topic, []).append(copy.deepcopy(msg))
        self.queue.append((topic, copy.deepcopy(msg)))

    def drain(self):
        guard = 0
        while self.queue:
            guard += 1
            assert guard < 100000, 'message loop'
            topic, msg = self.queue.pop(0)
            for node, cb in list(self.subs.get(topic, [])):
                if not node.destroyed:
                    cb(msg)

    # ---- time ----
    def advance(self, seconds: float, dt: float = 0.005):
        end = self.now + seconds
        while self.now < end - 1e-9:
            self.now = round(self.now + dt, 9)
            for t in list(self.timers):
                if t['node'].destroyed:
                    continue
                if self.now + 1e-9 >= t['next']:
                    t['next'] += t['period']
                    t['cb']()
                    self.drain()
            self.drain()

    def messages(self, topic):
        return self.sent.get(topic, [])

    def log_text(self, level=None):
        return [t for (_, _, lv, t) in self.logs if level is None or lv == level]


class Node:
    def __init__(self, node_name, **kwargs):
        bus = CURRENT_BUS
        assert bus is not None, 'fake_ros.new_bus() must be called first'
        self._bus = bus
        self._name = node_name
        self._params: Dict[str, Any] = {}
        self.destroyed = False
        bus.nodes[node_name] = self
        self._declared: set = set()

    def get_name(self):
        return self._name

    def get_logger(self):
        return FakeLogger(self._bus, self._name)

    def get_clock(self):
        return FakeClock(self._bus)

    def create_publisher(self, msg_type, topic, qos):
        assert isinstance(qos, QoSProfile), 'qos must be a QoSProfile (or int depth)'
        return FakePublisher(self._bus, msg_type, topic)

    def create_subscription(self, msg_type, topic, callback, qos):
        assert isinstance(qos, QoSProfile)

        def checked(msg, _t=msg_type, _cb=callback):
            assert isinstance(msg, _t), f'{topic}: expected {_t.__name__}'
            _cb(msg)
        self._bus.subs.setdefault(topic, []).append((self, checked))
        return object()

    def create_timer(self, period, callback):
        assert period > 0
        self._bus.timers.append({'node': self, 'period': period, 'cb': callback, 'next': self._bus.now + period})
        return object()

    def declare_parameter(self, name, value=None):
        if name in self._declared:
            raise RuntimeError(f'parameter {name} declared twice')
        self._declared.add(name)
        over = self._bus.overrides.get(self._name, {})
        if name in over:
            ov = over[name]
            if type(ov) is not type(value):
                raise TypeError(f'parameter {name}: override {ov!r} ({type(ov).__name__}) does not match '
                                f'declared type {type(value).__name__} (write doubles with a decimal point)')
            value = ov
        self._params[name] = value
        return SimpleNamespace(name=name, value=value)

    def get_parameter(self, name):
        if name not in self._params:
            raise RuntimeError(f'parameter {name} is not declared')
        return SimpleNamespace(name=name, value=self._params[name])

    def set_param(self, name, value):
        """Simulate `ros2 param set` (type must match the declared type)."""
        if type(self._params[name]) is not type(value):
            raise TypeError(f'cannot change type of parameter {name}')
        self._params[name] = value

    def unused_overrides(self):
        return sorted(set(self._bus.overrides.get(self._name, {})) - self._declared)

    def destroy_node(self):
        self.destroyed = True
        return True


class QoSProfile:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class ReliabilityPolicy(Enum):
    RELIABLE = 1
    BEST_EFFORT = 2


class DurabilityPolicy(Enum):
    VOLATILE = 1
    TRANSIENT_LOCAL = 2


class HistoryPolicy(Enum):
    KEEP_LAST = 1
    KEEP_ALL = 2


_ok = True


def new_bus() -> FakeBus:
    global CURRENT_BUS, _ok
    CURRENT_BUS = FakeBus()
    _ok = True
    return CURRENT_BUS


def install():
    """Register fake `rclpy`, `rclpy.node`, `rclpy.qos`, `rclpy.executors` modules."""
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda args=None: None
    rclpy.ok = lambda: _ok

    def shutdown():
        global _ok
        _ok = False
    rclpy.shutdown = shutdown
    rclpy.spin = lambda node: (_ for _ in ()).throw(RuntimeError('fake rclpy: use bus.advance()'))
    rclpy.spin_once = lambda node, timeout_sec=None: None
    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = Node
    qos_mod = types.ModuleType('rclpy.qos')
    qos_mod.QoSProfile, qos_mod.ReliabilityPolicy = QoSProfile, ReliabilityPolicy
    qos_mod.DurabilityPolicy, qos_mod.HistoryPolicy = DurabilityPolicy, HistoryPolicy
    exe_mod = types.ModuleType('rclpy.executors')
    exe_mod.ExternalShutdownException = ExternalShutdownException
    rclpy.node, rclpy.qos, rclpy.executors = node_mod, qos_mod, exe_mod
    sys.modules.update({'rclpy': rclpy, 'rclpy.node': node_mod, 'rclpy.qos': qos_mod,
                        'rclpy.executors': exe_mod})
