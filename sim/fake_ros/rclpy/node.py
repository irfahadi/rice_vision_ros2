import rclpy as _r


class Node:
    def __init__(self, node_name, **_kwargs):
        self._name = node_name
        self._params = {}
        self._overrides = _r._param_overrides.get(node_name, {})
        self._logger = _r._Logger(node_name)
        self._subscriptions = []
        self._timers_owned = []

    def get_name(self):
        return self._name

    # ---- parameter ----
    def declare_parameter(self, name, value=None, *_args, **_kwargs):
        if name in self._overrides:
            override = self._overrides[name]
            if value is not None and override is not None and type(override) is not type(value) \
                    and not (isinstance(value, float) and isinstance(override, int)):
                raise TypeError(
                    f'[{self._name}] parameter "{name}": tipe override {type(override).__name__} '
                    f'!= default {type(value).__name__}')
            value = float(override) if isinstance(value, float) else override
        self._params[name] = _r._Param(name, value)
        return self._params[name]

    def get_parameter(self, name):
        if name not in self._params:
            raise KeyError(f'[{self._name}] parameter "{name}" belum dideklarasikan')
        return self._params[name]

    # ---- komunikasi ----
    def create_publisher(self, msg_type, topic, qos):
        _r._register_type(topic, msg_type)
        return _r._Publisher(topic, msg_type)

    def create_subscription(self, msg_type, topic, callback, qos):
        _r._register_type(topic, msg_type)
        sub = _r._Subscription(topic, msg_type, callback)
        with _r._lock:
            _r._subs.setdefault(topic, []).append(sub)
        self._subscriptions.append(sub)
        return sub

    def create_timer(self, period, callback):
        t = _r._Timer(period, callback)
        _r._register_timer(t)
        self._timers_owned.append(t)
        return t

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _r._Clock()

    def destroy_node(self):
        with _r._lock:
            for sub in self._subscriptions:
                lst = _r._subs.get(sub.topic, [])
                if sub in lst:
                    lst.remove(sub)
        for t in self._timers_owned:
            t.cancel()
