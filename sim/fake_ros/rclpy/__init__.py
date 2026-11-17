"""rclpy palsu untuk simulasi integrasi tanpa instalasi ROS 2.

Yang ditiru (cukup untuk node rice_vision):
- Satu "graph" in-process: semua node berbagi bus topic.
- Satu executor single-thread (seperti rclpy.spin): callback subscription &
  timer dijalankan berurutan di thread executor. publish() dari thread lain
  (mis. callback paho-mqtt) aman, karena pesan masuk antrean thread-safe.
- Pesan di-deepcopy saat dikirim (seperti serialisasi DDS).
- Satu topic hanya boleh punya SATU tipe message; publisher/subscriber dengan
  tipe berbeda -> error (ROS 2 asli juga tidak akan menghubungkannya).
- Parameter: declare_parameter + override via set_parameter_overrides().

Bukan pengganti ROS 2 asli: QoS, DDS discovery, multi-proses, launch file,
dan colcon build TIDAK diuji di sini.
"""

import copy
import heapq
import itertools
import queue
import sys
import threading
import time

_lock = threading.RLock()
_subs = {}          # topic -> list[_Subscription]
_topic_types = {}   # topic -> msg class
_timers = []        # heap (due, seq, _Timer)
_seq = itertools.count()
_inbox = queue.Queue()
_running = threading.Event()
_param_overrides = {}  # node_name -> {param: value}
_log_sink = []      # (t, node, level, text) untuk asersi test
_log_echo = True


def set_parameter_overrides(node_name, params):
    _param_overrides[node_name] = dict(params)


def set_log_echo(enabled):
    global _log_echo
    _log_echo = enabled


def logs():
    return list(_log_sink)


def init(args=None, context=None):
    _running.set()


def ok(context=None):
    return _running.is_set()


def shutdown(context=None):
    _running.clear()


def reset():
    """Kosongkan graph (antar skenario test)."""
    with _lock:
        _subs.clear()
        _topic_types.clear()
        _timers.clear()
        _log_sink.clear()
    while not _inbox.empty():
        _inbox.get_nowait()


def _register_type(topic, msg_type):
    with _lock:
        known = _topic_types.get(topic)
        if known is None:
            _topic_types[topic] = msg_type
        elif known is not msg_type:
            raise TypeError(
                f'Topic "{topic}" dipakai dengan dua tipe berbeda: '
                f'{known.__name__} dan {msg_type.__name__}')


def _deliver(topic, msg):
    with _lock:
        targets = list(_subs.get(topic, ()))
    for sub in targets:
        _inbox.put((sub.callback, copy.deepcopy(msg)))


def spin_once(node=None, timeout_sec=0.05):
    """Jalankan semua pekerjaan yang siap; tunggu maksimal timeout_sec."""
    deadline = time.monotonic() + (timeout_sec or 0.0)
    did = False
    while True:
        now = time.monotonic()
        fired = None
        with _lock:
            if _timers and _timers[0][0] <= now:
                _, _, timer = heapq.heappop(_timers)
                if not timer.cancelled:
                    timer.due += timer.period
                    if timer.due < now:
                        timer.due = now + timer.period
                    heapq.heappush(_timers, (timer.due, next(_seq), timer))
                    fired = timer
        if fired is not None:
            _safe_call(fired.callback)
            did = True
            continue
        try:
            cb, msg = _inbox.get_nowait()
            _safe_call(cb, msg)
            did = True
            continue
        except queue.Empty:
            pass
        if did or time.monotonic() >= deadline:
            return
        with _lock:
            next_due = _timers[0][0] if _timers else deadline
        wait = max(0.0, min(deadline, next_due) - time.monotonic())
        try:
            cb, msg = _inbox.get(timeout=min(wait, 0.01) if wait else 0.0)
            _safe_call(cb, msg)
            did = True
        except queue.Empty:
            pass


def _safe_call(cb, *args):
    try:
        cb(*args)
    except Exception as exc:  # executor asli juga akan menghentikan spin; di sim kita catat
        import traceback
        traceback.print_exc()
        _log_sink.append((time.time(), 'executor', 'EXCEPTION', repr(exc)))
        raise


def spin(node=None, executor=None):
    while _running.is_set():
        spin_once(timeout_sec=0.05)


def spin_until(predicate, timeout_sec):
    """Utilitas test: spin sampai predicate() True atau timeout. Return bool."""
    end = time.monotonic() + timeout_sec
    while time.monotonic() < end:
        spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def spin_for(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        spin_once(timeout_sec=min(0.05, max(0.0, end - time.monotonic())))


class _Subscription:
    def __init__(self, topic, msg_type, callback):
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback


class _Publisher:
    def __init__(self, topic, msg_type):
        self.topic = topic
        self.msg_type = msg_type

    def publish(self, msg):
        if type(msg) is not self.msg_type:
            raise TypeError(
                f'publish ke "{self.topic}" harus {self.msg_type.__name__}, '
                f'didapat {type(msg).__name__}')
        _deliver(self.topic, msg)

    def get_subscription_count(self):
        with _lock:
            return len(_subs.get(self.topic, ()))


class _Timer:
    def __init__(self, period, callback):
        self.period = float(period)
        self.callback = callback
        self.due = time.monotonic() + self.period
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


def _register_timer(timer):
    with _lock:
        heapq.heappush(_timers, (timer.due, next(_seq), timer))


class _Param:
    def __init__(self, name, value):
        self.name = name
        self.value = value


class _Logger:
    def __init__(self, name):
        self.name = name
        self._last = {}

    def _log(self, level, msg, throttle_duration_sec=None, once=False, **_):
        key = (level, msg if once else None)
        now = time.monotonic()
        if throttle_duration_sec:
            tkey = (level, sys._getframe(2).f_lineno)
            if now - self._last.get(tkey, -1e9) < throttle_duration_sec:
                return
            self._last[tkey] = now
        if once:
            if key in self._last:
                return
            self._last[key] = now
        _log_sink.append((time.time(), self.name, level, str(msg)))
        if _log_echo:
            print(f'[{time.strftime("%H:%M:%S")}] [{level:5s}] [{self.name}]: {msg}', flush=True)

    def debug(self, msg, **kw):
        pass

    def info(self, msg, **kw):
        self._log('INFO', msg, **kw)

    def warn(self, msg, **kw):
        self._log('WARN', msg, **kw)

    warning = warn

    def error(self, msg, **kw):
        self._log('ERROR', msg, **kw)

    def fatal(self, msg, **kw):
        self._log('FATAL', msg, **kw)


class _TimeNow:
    def __init__(self):
        self.t = time.time()

    def to_msg(self):
        from builtin_interfaces.msg import Time
        sec = int(self.t)
        return Time(sec=sec, nanosec=int((self.t - sec) * 1e9))


class _Clock:
    def now(self):
        return _TimeNow()
