#!/usr/bin/env python3
"""Static checker for arm_interfaces (pure Python, standard library only, no ROS needed).

This is NOT a substitute for rosidl generation (`colcon build`); it catches the mistakes
that can be caught without ROS:
  * every msg/*.msg is listed in CMakeLists.txt and vice versa
  * every line parses as a field, constant, or comment
  * field types are known primitives, bounded/fixed arrays of them, std_msgs/Header or a
    message in this package; no free-form `string` fields (core control data must be typed)
  * constants fit their type; names follow ROS conventions
  * enum-style constant groups: first value 0 is the INVALID/UNKNOWN/safe default where required
  * *_FLAGS-style bit constants are distinct powers of two
  * NUM_CHANNELS agrees with arm_control.channels.CHANNEL_COUNT, and seven-channel arrays are
    bounded to exactly that size

Usage: python3 tests/check_interfaces.py [path/to/arm_interfaces]
Exit status 0 = no problems, 1 = problems printed.
"""
import re
import sys
from pathlib import Path
from typing import Dict, List

PRIMITIVE_RANGES = {
    'bool': (0, 1), 'byte': (0, 255), 'char': (0, 255),
    'int8': (-128, 127), 'uint8': (0, 255),
    'int16': (-32768, 32767), 'uint16': (0, 65535),
    'int32': (-2**31, 2**31 - 1), 'uint32': (0, 2**32 - 1),
    'int64': (-2**63, 2**63 - 1), 'uint64': (0, 2**64 - 1),
    'float32': None, 'float64': None,
}
FIELD_NAME = re.compile(r'^[a-z][a-z0-9_]*$')
CONST_NAME = re.compile(r'^[A-Z][A-Z0-9_]*$')
TYPE_RE = re.compile(r'^(?P<base>[A-Za-z0-9_/]+)(?P<arr>\[(?:<=)?\d*\])?$')


class Msg:
    def __init__(self, name: str):
        self.name = name
        self.fields: Dict[str, str] = {}      # field name -> full type text
        self.constants: Dict[str, tuple] = {}  # const name -> (type, int/float value)


def parse_msg(path: Path, problems: List[str]) -> Msg:
    msg = Msg(path.stem)
    if not re.fullmatch(r'[A-Z][A-Za-z0-9]*', path.stem):
        problems.append(f'{path.name}: message name must be CamelCase')
    for n, raw in enumerate(path.read_text().splitlines(), 1):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        where = f'{path.name}:{n}'
        m = re.fullmatch(r'(\S+)\s+([A-Za-z0-9_]+)\s*=\s*(\S+)', line)
        if m:  # constant
            typ, name, val = m.groups()
            if typ not in PRIMITIVE_RANGES:
                problems.append(f'{where}: constant type {typ!r} not a primitive')
                continue
            if not CONST_NAME.match(name):
                problems.append(f'{where}: constant {name!r} must be UPPER_SNAKE_CASE')
            try:
                value = float(val) if typ.startswith('float') else int(val, 0)
            except ValueError:
                problems.append(f'{where}: bad constant value {val!r}')
                continue
            rng = PRIMITIVE_RANGES[typ]
            if rng and not rng[0] <= value <= rng[1]:
                problems.append(f'{where}: {name}={value} outside {typ} range')
            if name in msg.constants or name in msg.fields:
                problems.append(f'{where}: duplicate name {name}')
            msg.constants[name] = (typ, value)
            continue
        m = re.fullmatch(r'(\S+)\s+([A-Za-z0-9_]+)', line)
        if not m:
            problems.append(f'{where}: cannot parse line {raw.strip()!r}')
            continue
        typ, name = m.groups()
        if not FIELD_NAME.match(name):
            problems.append(f'{where}: field {name!r} must be lower_snake_case')
        if name in msg.fields or name in msg.constants:
            problems.append(f'{where}: duplicate name {name}')
        msg.fields[name] = typ
    return msg


def check_types(msgs: Dict[str, Msg], problems: List[str]) -> None:
    for msg in msgs.values():
        for name, typ in msg.fields.items():
            m = TYPE_RE.match(typ)
            if not m:
                problems.append(f'{msg.name}.{name}: unparseable type {typ!r}')
                continue
            base = m.group('base')
            if base in ('string', 'wstring'):
                problems.append(f'{msg.name}.{name}: free-form string not allowed for control data')
            elif base in PRIMITIVE_RANGES or base == 'std_msgs/Header':
                pass
            elif base in msgs:
                pass
            else:
                problems.append(f'{msg.name}.{name}: unknown type {base!r}')


def check_enum_groups(msg: Msg, problems: List[str]) -> None:
    """Constants sharing a prefix with a field (e.g. ACTION_* for `action`) form an enum."""
    for field in msg.fields:
        prefix = field.upper() + '_'
        group = {k: v[1] for k, v in msg.constants.items() if k.startswith(prefix)}
        if not group or field.endswith('_flags'):
            continue
        if len(set(group.values())) != len(group):
            problems.append(f'{msg.name}: duplicate values among {prefix}* constants')
    # Fields that must default to a safe/invalid value 0.
    safe_zero = {'action': 'ACTION_INVALID', 'command_type': 'COMMAND_TYPE_INVALID',
                 'firmware_state': 'FW_STATE_UNKNOWN', 'link_state': 'LINK_DISCONNECTED'}
    for field, const in safe_zero.items():
        if field in msg.fields and msg.constants.get(const, (None, None))[1] != 0:
            problems.append(f'{msg.name}: {const} must exist and equal 0 (safe default for {field})')


def check_flag_bits(msg: Msg, problems: List[str]) -> None:
    groups: Dict[str, Dict[str, int]] = {}
    for k, (typ, val) in msg.constants.items():
        for prefix in ('VALID_', 'FAULT_'):
            if k.startswith(prefix):
                groups.setdefault(prefix, {})[k] = val
    for prefix, g in groups.items():
        for k, v in g.items():
            if v <= 0 or v & (v - 1):
                problems.append(f'{msg.name}: {k}={v} is not a power of two')
        if len(set(g.values())) != len(g):
            problems.append(f'{msg.name}: duplicate bit values among {prefix}* constants')


def check_channel_arrays(msgs: Dict[str, Msg], problems: List[str], channel_count) -> None:
    for msg in msgs.values():
        if 'NUM_CHANNELS' in msg.constants:
            if msg.constants['NUM_CHANNELS'][1] != channel_count:
                problems.append(f'{msg.name}: NUM_CHANNELS != arm_control.channels.CHANNEL_COUNT ({channel_count})')
            for name, typ in msg.fields.items():
                if typ.endswith(']') and typ != f'{typ.split("[")[0]}[<={channel_count}]':
                    problems.append(f'{msg.name}.{name}: channel array must be bounded [<={channel_count}], got {typ}')


def check_cmake(pkg: Path, msgs: Dict[str, Msg], problems: List[str]) -> None:
    cmake = (pkg / 'CMakeLists.txt').read_text()
    listed = set(re.findall(r'"msg/([A-Za-z0-9]+)\.msg"', cmake))
    if listed != set(msgs):
        problems.append(f'CMakeLists.txt msg list {sorted(listed)} != msg/ files {sorted(msgs)}')
    xml = (pkg / 'package.xml').read_text()
    for needle in ('rosidl_default_generators', 'rosidl_default_runtime', 'rosidl_interface_packages', 'std_msgs'):
        if needle not in xml:
            problems.append(f'package.xml missing {needle}')


def load_channel_count(repo: Path) -> int:
    sys.path.insert(0, str(repo / 'src' / 'arm_control'))
    try:
        from arm_control.channels import CHANNEL_COUNT
    finally:
        sys.path.pop(0)
    return CHANNEL_COUNT


def run_checks(pkg: Path, channel_count: int) -> List[str]:
    problems: List[str] = []
    msgs = {p.stem: parse_msg(p, problems) for p in sorted((pkg / 'msg').glob('*.msg'))}
    if not msgs:
        return [f'no .msg files under {pkg / "msg"}']
    check_types(msgs, problems)
    for msg in msgs.values():
        check_enum_groups(msg, problems)
        check_flag_bits(msg, problems)
    check_channel_arrays(msgs, problems, channel_count)
    check_cmake(pkg, msgs, problems)
    return problems


def main(argv: List[str]) -> int:
    repo = Path(__file__).resolve().parent.parent
    pkg = Path(argv[1]) if len(argv) > 1 else repo / 'src' / 'arm_interfaces'
    problems = run_checks(pkg, load_channel_count(repo))
    for p in problems:
        print(f'PROBLEM: {p}')
    print(f'{len(problems)} problem(s) found in {pkg}')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
