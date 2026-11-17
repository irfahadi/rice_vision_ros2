"""Inti message palsu untuk simulasi tanpa ROS 2.

Meniru perilaku message rclpy yang penting untuk menangkap bug integrasi:
- field punya default sesuai tipe ROS (string -> '', int32 -> 0, ...)
- assignment dicek tipenya seperti rclpy (AssertionError bila salah tipe,
  mis. float ke field int32, atau int ke field float32)
- range integer dicek (int32/int64/uint8)
- atribut yang tidak ada di definisi .msg ditolak
"""

import copy
import math
import os
import re

_INT_RANGES = {
    'int8': (-2 ** 7, 2 ** 7 - 1), 'uint8': (0, 2 ** 8 - 1),
    'int16': (-2 ** 15, 2 ** 15 - 1), 'uint16': (0, 2 ** 16 - 1),
    'int32': (-2 ** 31, 2 ** 31 - 1), 'uint32': (0, 2 ** 32 - 1),
    'int64': (-2 ** 63, 2 ** 63 - 1), 'uint64': (0, 2 ** 64 - 1),
}
_FLOATS = ('float32', 'float64')


class Message:
    """Base class; subclass mengisi _fields = [(name, ros_type, default_factory)]."""

    _fields = []

    def __init__(self, **kwargs):
        for name, _rtype, factory in self._fields:
            object.__setattr__(self, name, factory())
        for k, v in kwargs.items():
            setattr(self, k, v)

    def __setattr__(self, name, value):
        spec = {f[0]: f[1] for f in self._fields}
        if name not in spec:
            raise AttributeError(f'{type(self).__name__} tidak punya field "{name}"')
        value = _check(type(self).__name__, name, spec[name], value)
        object.__setattr__(self, name, value)

    def __eq__(self, other):
        return type(self) is type(other) and all(
            getattr(self, f[0]) == getattr(other, f[0]) for f in self._fields)

    def __repr__(self):
        body = ', '.join(f'{f[0]}={getattr(self, f[0])!r}' for f in self._fields)
        return f'{type(self).__name__}({body})'

    def __deepcopy__(self, memo):
        new = type(self)()
        for name, _t, _f in self._fields:
            object.__setattr__(new, name, copy.deepcopy(getattr(self, name), memo))
        return new


def _check(cls, name, rtype, value):
    where = f'{cls}.{name} ({rtype})'
    if rtype.endswith('[]'):
        base = rtype[:-2]
        if base == 'uint8' and isinstance(value, (bytes, bytearray)):
            return bytes(value)
        assert isinstance(value, (list, tuple)), f'{where}: harus list, didapat {type(value).__name__}'
        return [_check(cls, name, base, v) for v in value]
    if rtype in _INT_RANGES:
        assert isinstance(value, int), f'{where}: harus int, didapat {type(value).__name__} ({value!r})'
        lo, hi = _INT_RANGES[rtype]
        assert lo <= int(value) <= hi, f'{where}: nilai {value} di luar range [{lo}, {hi}]'
        return int(value)
    if rtype in _FLOATS:
        assert isinstance(value, float), f'{where}: harus float, didapat {type(value).__name__} ({value!r})'
        if rtype == 'float32' and not math.isnan(value) and not math.isinf(value):
            assert abs(value) <= 3.4028235e38, f'{where}: di luar range float32'
        return value
    if rtype == 'string':
        assert isinstance(value, str), f'{where}: harus str, didapat {type(value).__name__}'
        return value
    if rtype == 'bool':
        assert isinstance(value, bool), f'{where}: harus bool'
        return value
    # tipe message bersarang: cek nama kelasnya
    expected = rtype.split('/')[-1]
    assert type(value).__name__ == expected, f'{where}: harus {expected}, didapat {type(value).__name__}'
    return value


def make_message(name, fields):
    return type(name, (Message,), {'_fields': fields})


def default_factory(rtype, resolver):
    if rtype.endswith('[]'):
        return list
    if rtype in _INT_RANGES:
        return int
    if rtype in _FLOATS:
        return float
    if rtype == 'string':
        return str
    if rtype == 'bool':
        return bool
    return lambda: resolver(rtype)()


def load_msg_dir(msg_dir, package, resolver):
    """Bangun kelas message dari file .msg (urutan dependensi diselesaikan lazy)."""
    classes = {}
    for fname in sorted(os.listdir(msg_dir)):
        if not fname.endswith('.msg'):
            continue
        name = fname[:-4]
        fields = []
        with open(os.path.join(msg_dir, fname)) as fh:
            for line in fh:
                line = line.split('#', 1)[0].strip()
                if not line:
                    continue
                m = re.match(r'^([\w/]+(?:\[\])?)\s+(\w+)$', line)
                if not m:
                    raise ValueError(f'{fname}: baris tidak dikenal "{line}"')
                rtype, fname_ = m.groups()
                if '/' not in rtype.rstrip('[]') and rtype.rstrip('[]') not in _INT_RANGES \
                        and rtype.rstrip('[]') not in _FLOATS and rtype.rstrip('[]') not in ('string', 'bool'):
                    rtype = f'{package}/{rtype}'
                fields.append((fname_, rtype, default_factory(rtype, resolver)))
        classes[name] = make_message(name, fields)
    return classes
