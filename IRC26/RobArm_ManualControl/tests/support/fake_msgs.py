"""Strict stand-ins for the generated ROS message classes, built from the real .msg files.

NOT the real rosidl output: this exists so node code can be exercised without ROS. It mimics
the generated Python classes' behaviour that matters for catching bugs:
  * attribute assignment is type- and range-checked (an int for a float32 field raises,
    out-of-range integers raise, wrong nested message types raise, unknown attributes raise)
  * bounded sequences enforce their bound; defaults are zero/empty/NaN-free like the real ones
  * constants are class attributes
It cannot prove that `colcon build` accepts the .msg files.
"""
import copy
import math
import sys
import types
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'tests'))
import check_interfaces as ci  # noqa: E402

_INT_RANGES = {k: v for k, v in ci.PRIMITIVE_RANGES.items() if v is not None and k != 'bool'}
_FLOAT_MAX = 3.4028234663852886e38


class Time:
    def __init__(self, sec=0, nanosec=0):
        self.sec, self.nanosec = sec, nanosec

    def __eq__(self, other):
        return isinstance(other, Time) and (self.sec, self.nanosec) == (other.sec, other.nanosec)

    def __repr__(self):
        return f'Time({self.sec}.{self.nanosec:09d})'


def _check(cls_name, field, typ, value, registry):
    base, bound = typ, None
    is_array = '[' in typ
    if is_array:
        base, rest = typ.split('[', 1)
        rest = rest.rstrip(']')
        bound = int(rest[2:]) if rest.startswith('<=') else (int(rest) if rest else None)
        if not isinstance(value, (list, tuple)):
            raise AssertionError(f"The '{field}' field of {cls_name} must be a list or tuple")
        if bound is not None and len(value) > bound:
            raise AssertionError(f"The '{field}' field of {cls_name} must have at most {bound} items")
        for item in value:
            _check(cls_name, field, base, item, registry)
        return
    if base in _INT_RANGES:
        lo, hi = _INT_RANGES[base]
        if isinstance(value, bool) or not isinstance(value, int):
            raise AssertionError(f"The '{field}' field of {cls_name} must be of type 'int', got {type(value).__name__}")
        if not lo <= value <= hi:
            raise AssertionError(f"The '{field}' field of {cls_name} must be an integer in [{lo}, {hi}]")
    elif base in ('float32', 'float64'):
        if not isinstance(value, float):
            raise AssertionError(f"The '{field}' field of {cls_name} must be of type 'float', got {type(value).__name__}")
        if base == 'float32' and math.isfinite(value) and abs(value) > _FLOAT_MAX:
            raise AssertionError(f"The '{field}' field of {cls_name} must be a float32 in range")
    elif base == 'bool':
        if not isinstance(value, bool):
            raise AssertionError(f"The '{field}' field of {cls_name} must be of type 'bool'")
    elif base in ('string', 'wstring'):
        if not isinstance(value, str):
            raise AssertionError(f"The '{field}' field of {cls_name} must be of type 'str'")
    else:
        short = base.split('/')[-1]
        target = registry[short]
        if not isinstance(value, target):
            raise AssertionError(f"The '{field}' field of {cls_name} must be a {short}, got {type(value).__name__}")


def _default(typ, registry):
    if '[' in typ:
        return []
    if typ in _INT_RANGES:
        return 0
    if typ in ('float32', 'float64'):
        return 0.0
    if typ == 'bool':
        return False
    if typ in ('string', 'wstring'):
        return ''
    return registry[typ.split('/')[-1]]()


def _make_class(name, fields, constants, registry):
    def __init__(self, **kwargs):
        for f, t in fields.items():
            object.__setattr__(self, f, _default(t, registry))
        for k, v in kwargs.items():
            setattr(self, k, v)

    def __setattr__(self, key, value):
        if key not in fields:
            raise AttributeError(f'{name} has no field {key!r}')
        _check(name, key, fields[key], value, registry)
        object.__setattr__(self, key, value)

    def __eq__(self, other):
        return type(other) is type(self) and all(getattr(self, f) == getattr(other, f) for f in fields)

    def __repr__(self):
        return f'{name}(' + ', '.join(f'{f}={getattr(self, f)!r}' for f in fields) + ')'

    ns = {'__init__': __init__, '__setattr__': __setattr__, '__eq__': __eq__, '__repr__': __repr__,
          '__hash__': None, '_fields': dict(fields)}
    ns.update({k: v for k, (_, v) in constants.items()})
    return type(name, (), ns)


class _Header:
    def __init__(self):
        object.__setattr__(self, 'stamp', Time())
        object.__setattr__(self, 'frame_id', '')

    def __setattr__(self, key, value):
        if key == 'stamp' and not isinstance(value, Time):
            raise AssertionError("The 'stamp' field of Header must be a builtin_interfaces Time")
        if key == 'frame_id' and not isinstance(value, str):
            raise AssertionError("The 'frame_id' field of Header must be a str")
        if key not in ('stamp', 'frame_id'):
            raise AttributeError(key)
        object.__setattr__(self, key, value)

    def __eq__(self, other):
        return isinstance(other, _Header) and (self.stamp, self.frame_id) == (other.stamp, other.frame_id)

    def __deepcopy__(self, memo):
        h = _Header()
        object.__setattr__(h, 'stamp', copy.deepcopy(self.stamp))
        object.__setattr__(h, 'frame_id', self.frame_id)
        return h


def install(msg_dir: Path = None):
    """Create fake `std_msgs.msg` and `arm_interfaces.msg` modules from the .msg files."""
    msg_dir = msg_dir or REPO / 'src' / 'arm_interfaces' / 'msg'
    registry = {'Header': _Header}
    parsed = {}
    problems = []
    for p in sorted(msg_dir.glob('*.msg')):
        parsed[p.stem] = ci.parse_msg(p, problems)
    assert not problems, problems
    classes = {}
    for name, m in parsed.items():            # two passes so nested types resolve in any order
        registry[name] = None
    # Resolve in dependency order (MotorCommand/MotorState before Arm*).
    order = sorted(parsed, key=lambda n: sum(1 for t in parsed[n].fields.values() if t.split('[')[0] in parsed))
    for name in order:
        cls = _make_class(name, parsed[name].fields, parsed[name].constants, registry)
        registry[name] = cls
        classes[name] = cls
    std = types.ModuleType('std_msgs')
    std_msg = types.ModuleType('std_msgs.msg')
    std_msg.Header = _Header
    std.msg = std_msg
    pkg = types.ModuleType('arm_interfaces')
    pkg_msg = types.ModuleType('arm_interfaces.msg')
    for name, cls in classes.items():
        setattr(pkg_msg, name, cls)
    pkg.msg = pkg_msg
    sys.modules.update({'std_msgs': std, 'std_msgs.msg': std_msg,
                        'arm_interfaces': pkg, 'arm_interfaces.msg': pkg_msg})
    return classes
