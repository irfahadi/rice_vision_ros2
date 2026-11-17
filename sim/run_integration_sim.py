#!/usr/bin/env python3
"""Simulasi integrasi end-to-end: Cloud <-> Sink Node RW612 <-> rice_vision (ROS 2).

    Cloud (sim, paho)                 Sink Node RW612                 rice_vision (node ASLI)
    gh01/mission/command  ──MQTT──►  MqttSubscribeService.java  ──►  gh01/agent/mission/command
                                     (kode firmware ASLI, JVM)        sink_bridge_node
    gh01/mission/status   ◄─────────  relay  ◄───────────────────     collaboration_agent_node
    gh01/mission/result   ◄─────────  relay  ◄───────────────────     classifier_onnx_node (model .onnx)
                                                                      FrameFeeder (pengganti kamera)

Yang ASLI: semua node rice_vision, kontrak data, model models_bibit_padi.onnx,
firmware MQTT Sink Node (Java), broker MQTT (mosquitto), protokol MQTT.
Yang DISIMULASIKAN: ROS 2 (shim rclpy in-process di sim/fake_ros), kamera
(gambar sintetis per tray), Cloud backend, WiFi/LoRa/UI board.

Pemakaian:
    # 1. sekali: build simulator Sink Node dari repo npa-vee-rw612
    sim/rw612_sink/build.sh ../npa-vee-rw612
    # 2. jalankan (butuh: mosquitto, java, pip install paho-mqtt onnxruntime opencv-python numpy)
    python3 sim/run_integration_sim.py
"""

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, 'fake_ros'))
sys.path.insert(0, os.path.join(REPO, 'rice_vision'))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import paho.mqtt.client as mqtt  # noqa: E402

import rclpy  # noqa: E402  (shim)
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import Empty, Int32, String  # noqa: E402

from rice_vision import image_codec  # noqa: E402
from rice_vision.classifier_onnx_node import ClassifierONNXNode  # noqa: E402
from rice_vision.collaboration_agent_node import CollaborationAgentNode  # noqa: E402
from rice_vision.contracts import (  # noqa: E402
    MAX_BYTES_MISSION_RESULT, MAX_BYTES_MISSION_STATUS, MissionResult, MissionStatus)
from rice_vision.sink_bridge_node import SinkBridgeNode  # noqa: E402

try:
    from importlib.metadata import version as _pkg_version
    PAHO_VERSION = _pkg_version('paho-mqtt')
except Exception:  # noqa: BLE001
    PAHO_VERSION = '?'

CLOUD_CMD = 'gh01/mission/command'
CLOUD_STATUS = 'gh01/mission/status'
CLOUD_RESULT = 'gh01/mission/result'
DEFAULT_MODEL = os.path.join(REPO, 'rice_vision', 'models', 'models_bibit_padi.onnx')

# Waktu dipercepat untuk simulasi (produksi: status 3 s, tray 15-30 s)
SIM_TRAY_SEC = 3.0
SIM_STATUS_SEC = 2.0
SIM_GRACE_SEC = 3.0


# ----------------------------------------------------------------------------
# Gambar sintetis "isi tray" (hanya untuk menguji pipeline, BUKAN akurasi model)
# ----------------------------------------------------------------------------
def make_tray_image(kind, rng):
    if kind == 'good':     # model saat ini memprediksi "Bagus" untuk hijau rata
        img = np.full((480, 640, 3), (40, 170, 50), np.uint8)
        noise = 4
    else:                  # bibit kuning layu -> "Tidak Bagus"
        img = np.full((480, 640, 3), (40, 70, 100), np.uint8)
        for _ in range(60):
            x = int(rng.integers(20, 620)); y = int(rng.integers(200, 470))
            cv2.line(img, (x, y), (x + int(rng.integers(-40, 40)), y - int(rng.integers(60, 180))),
                     (40, 190, 210), int(rng.integers(2, 5)))
        noise = 12
    return (img + rng.normal(0, noise, img.shape)).clip(0, 255).astype(np.uint8)


class FrameFeeder(Node):
    """Pengganti camera_node: kirim gambar sesuai tray tempat robot berada."""

    def __init__(self, tray_content, fps=10.0):
        super().__init__('camera_node')
        self.tray_content = tray_content
        self.enabled = True
        self.cur_tray = 0
        self.rng = np.random.default_rng(42)
        self.pub = self.create_publisher(Image, '/camera/image_raw', None)
        self.create_subscription(String, '/mission/status/raw', self._on_status, 10)
        self.create_timer(1.0 / fps, self._tick)

    def _on_status(self, msg):
        self.cur_tray = json.loads(msg.data).get('cur_tray', 0)

    def _tick(self):
        if not self.enabled or self.cur_tray not in self.tray_content:
            return
        frame = make_tray_image(self.tray_content[self.cur_tray], self.rng)
        self.pub.publish(image_codec.cv2_to_imgmsg(frame))


# ----------------------------------------------------------------------------
# Cloud backend palsu
# ----------------------------------------------------------------------------
class CloudSim:
    def __init__(self, host, port):
        api = getattr(mqtt, 'CallbackAPIVersion', None)
        self.client = (mqtt.Client(api.VERSION1, client_id='cloud_sim') if api
                       else mqtt.Client(client_id='cloud_sim'))
        self.lock = threading.Lock()
        self.status, self.result, self.raw = [], [], []
        self.connected = threading.Event()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect(host, port, 30)
        self.client.loop_start()
        if not self.connected.wait(10):
            raise RuntimeError('Cloud sim gagal konek ke broker')

    def _on_connect(self, client, userdata, flags, rc):
        client.subscribe([(CLOUD_STATUS, 0), (CLOUD_RESULT, 0)])
        self.connected.set()

    def _on_message(self, client, userdata, msg):
        t = time.monotonic()
        with self.lock:
            self.raw.append((t, msg.topic, msg.payload))
            data = json.loads(msg.payload.decode())
            (self.status if msg.topic == CLOUD_STATUS else self.result).append((t, data, len(msg.payload)))

    def clear(self):
        with self.lock:
            self.status, self.result, self.raw = [], [], []

    def send(self, payload):
        raw = payload if isinstance(payload, (bytes, str)) else json.dumps(payload, separators=(',', ':'))
        self.client.publish(CLOUD_CMD, raw, qos=0)

    def snapshot(self, m_id=None):
        with self.lock:
            st = [s for s in self.status if m_id is None or s[1]['m_id'] == m_id]
            rs = [r for r in self.result if m_id is None or r[1]['m_id'] == m_id]
            return st, rs

    def close(self):
        self.client.loop_stop()
        self.client.disconnect()


# ----------------------------------------------------------------------------
# Infrastruktur
# ----------------------------------------------------------------------------
def port_open(host, port):
    try:
        with socket.create_connection((host, port), timeout=1):
            return True
    except OSError:
        return False


def ensure_broker(host, port, procs):
    if port_open(host, port):
        return
    exe = shutil.which('mosquitto')
    if not exe:
        raise RuntimeError('Broker MQTT tidak ada di %s:%d dan mosquitto tidak terpasang' % (host, port))
    procs.append(subprocess.Popen([exe, '-p', str(port)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    for _ in range(50):
        if port_open(host, port):
            return
        time.sleep(0.1)
    raise RuntimeError('mosquitto gagal start')


def start_sink(host, port, log_path, procs):
    build = os.path.join(HERE, 'rw612_sink', 'build')
    cp = os.path.join(build, 'classes') + os.pathsep + os.path.join(build, 'lib', 'paho-unsigned.jar')
    if not os.path.isdir(os.path.join(build, 'classes')):
        raise RuntimeError('Simulator Sink Node belum di-build: sim/rw612_sink/build.sh /path/npa-vee-rw612')
    log = open(log_path, 'w')
    p = subprocess.Popen(['java', f'-Dsim.broker=tcp://{host}:{port}', '-cp', cp, 'SinkNodeSim'],
                         stdout=log, stderr=subprocess.STDOUT, cwd=build)  # Paho Java menulis folder persistence di cwd
    procs.append(p)
    for _ in range(100):
        time.sleep(0.1)
        with open(log_path) as fh:
            txt = fh.read()
        if txt.count('MQTT subscription successful') >= 5:
            return p
    raise RuntimeError('Sink Node sim tidak berhasil subscribe; lihat ' + log_path)


class Stack:
    """Satu set node rice_vision (seperti mission_collaboration.launch.py)."""

    def __init__(self, args, tray_content, auto_inspect=True):
        rclpy.reset()
        rclpy.init()
        rclpy.set_parameter_overrides('sink_bridge_node', {
            'transport': 'mqtt', 'mqtt_broker': args.host, 'mqtt_port': args.port,
            'mqtt_client_id': 'rice_vision_collab_agent'})
        rclpy.set_parameter_overrides('collaboration_agent_node', {
            'robot_id': 'RB01', 'greenhouse_id': 'GH01',
            'status_interval_sec': SIM_STATUS_SEC, 'tray_inspection_interval_sec': SIM_TRAY_SEC,
            'detection_grace_sec': SIM_GRACE_SEC, 'auto_inspect': auto_inspect,
            'simulate_battery_drain': True})
        rclpy.set_parameter_overrides('classifier_onnx_node', {
            'model_path': args.model, 'max_classification_rate': 5.0})
        self.feeder = FrameFeeder(tray_content)
        self.classifier = ClassifierONNXNode()
        self.agent = CollaborationAgentNode()
        self.bridge = SinkBridgeNode()
        self.nodes = [self.feeder, self.classifier, self.agent, self.bridge]
        helper = Node('sim_helper')
        self.trigger_pub = helper.create_publisher(Int32, '/mission/trigger_tray', 10)
        self.step_pub = helper.create_publisher(Empty, '/mission/step_tray', 10)
        self.nodes.append(helper)
        if not rclpy.spin_until(lambda: self.bridge.is_connected, 10):
            raise RuntimeError('sink_bridge_node gagal konek ke broker')
        rclpy.spin_for(0.5)

    def trigger(self, tray_id):
        self.trigger_pub.publish(Int32(data=tray_id))

    def close(self):
        for n in self.nodes:
            n.destroy_node()
        rclpy.spin_for(0.3)


# ----------------------------------------------------------------------------
# Skenario
# ----------------------------------------------------------------------------
class Check:
    def __init__(self):
        self.items = []

    def __call__(self, cond, text):
        self.items.append((bool(cond), text))
        print(('  [PASS] ' if cond else '  [FAIL] ') + text, flush=True)
        return bool(cond)

    @property
    def ok(self):
        return all(c for c, _ in self.items)


def ts_now():
    return int(time.time())


def cmd(m_id, action, trays=(), ts=None):
    return {'m_id': m_id, 'h_id': 'GH01', 'cmd': action, 'trays': list(trays), 'crit_lvl': 1,
            'ts': ts or ts_now()}


def wait_cloud(cloud, pred, timeout):
    return rclpy.spin_until(lambda: pred(*cloud.snapshot()), timeout)


def check_contracts(check, statuses, results):
    bad = []
    for _, d, n in statuses:
        try:
            MissionStatus.from_dict(d).validate()
            assert n <= MAX_BYTES_MISSION_STATUS
        except Exception as exc:
            bad.append(f'status {d}: {exc}')
    for _, d, n in results:
        try:
            MissionResult.from_dict(d).validate()
            assert n <= MAX_BYTES_MISSION_RESULT
        except Exception as exc:
            bad.append(f'result {d}: {exc}')
    check(not bad, f'semua {len(statuses)} status & {len(results)} result di Cloud valid sesuai kontrak & batas byte'
          + (f' -> {bad[:2]}' if bad else ''))


def count_logs(node, needle):
    return sum(1 for _, n, _, t in rclpy.logs() if n == node and needle in t)


def scenario_happy_path(args, cloud, check):
    """INSPECT 3 tray, mode auto, isi tray beda-beda."""
    content = {1: 'good', 2: 'bad', 3: 'good'}
    expected = {1: 'BAGUS', 2: 'TIDAK BAGUS', 3: 'BAGUS'}
    stack = Stack(args, content)
    try:
        cloud.clear()
        m_id = 'M-SIM-HAPPY'
        cloud.send(cmd(m_id, 'INSPECT', [1, 2, 3]))
        done = wait_cloud(cloud, lambda st, rs: any(s[1]['m_id'] == m_id and s[1]['st'] == 'COMPLETED' for s in st),
                          3 * (SIM_TRAY_SEC + 2) + 10)
        rclpy.spin_for(SIM_STATUS_SEC + 0.5)  # pastikan tidak ada status lanjutan setelah COMPLETED
        st, rs = cloud.snapshot(m_id)
        check(done, 'misi mencapai COMPLETED di Cloud (lewat Sink Node)')
        check([r[1]['tray_id'] for r in rs] == [1, 2, 3], f'result per tray tepat 1x & berurutan: {[r[1]["tray_id"] for r in rs]}')
        got = {r[1]['tray_id']: r[1]['label'] for r in rs}
        check(got == expected, f'label sesuai isi tray (model ONNX asli): {got}')
        check(all(r[1]['anom'] == (1 if r[1]['label'] != 'BAGUS' else 0) for r in rs), 'anom konsisten dengan label')
        check(all(0.5 <= r[1]['conf'] <= 1.0 for r in rs), f'conf dari model: {[r[1]["conf"] for r in rs]}')
        check([r[1]['seq'] for r in rs] == [1, 2, 3], 'seq result 1..3 tanpa duplikat')
        seqs = [s[1]['seq'] for s in st]
        check(len(seqs) == len(set(seqs)), f'tidak ada status duplikat di Cloud ({len(seqs)} status, seq unik)')
        check(seqs == sorted(seqs), 'seq status naik monoton')
        check(st and st[-1][1]['st'] == 'COMPLETED' and st[-1][1]['pct'] == 100, 'status terakhir COMPLETED pct=100')
        pcts = [s[1]['pct'] for s in st]
        check(pcts == sorted(pcts), f'progres pct monoton: {sorted(set(pcts))}')
        check(all(r[1]['h_id'] == 'GH01' for r in rs) and all(s[1]['r_id'] == 'RB01' for s in st), 'h_id=GH01, r_id=RB01')
        check(count_logs('collaboration_agent_node', 'memproses perintah') == 1, 'command INSPECT diproses tepat 1x oleh agent')
        periodic = [b[0] - a[0] for a, b in zip(st, st[1:])]
        check(max(periodic) <= 5.0 + 0.5, f'jeda status maks {max(periodic):.1f}s (kontrak Ti 2-5 s)')
        check_contracts(check, st, rs)
    finally:
        stack.close()


def scenario_no_vision(args, cloud, check):
    """Kamera mati: agent tidak boleh mengarang BAGUS."""
    stack = Stack(args, {7: 'good'})
    stack.feeder.enabled = False
    try:
        cloud.clear()
        m_id = 'M-SIM-NOCAM'
        cloud.send(cmd(m_id, 'INSPECT', [7]))
        t0 = time.monotonic()
        done = wait_cloud(cloud, lambda st, rs: len([r for r in rs if r[1]['m_id'] == m_id]) >= 1,
                          SIM_TRAY_SEC + SIM_GRACE_SEC + 6)
        elapsed = time.monotonic() - t0
        _, rs = cloud.snapshot(m_id)
        check(done, 'result tetap dikirim setelah masa tunggu')
        if rs:
            r = rs[0][1]
            check(r['label'] == 'UNKNOWN' and r['anom'] == 1 and r['conf'] == 0.0,
                  f'tanpa deteksi -> label=UNKNOWN anom=1 conf=0.0 (bukan BAGUS 0.94): {r}')
        check(elapsed >= SIM_TRAY_SEC + SIM_GRACE_SEC - 0.5,
              f'agent menunggu interval+grace ({elapsed:.1f}s >= {SIM_TRAY_SEC + SIM_GRACE_SEC:.0f}s)')
    finally:
        stack.close()


def scenario_pause_resume_abort(args, cloud, check):
    stack = Stack(args, {1: 'good', 2: 'bad', 3: 'good', 4: 'good'})
    try:
        cloud.clear()
        m_id = 'M-SIM-CTRL'
        cloud.send(cmd(m_id, 'INSPECT', [1, 2, 3, 4]))
        wait_cloud(cloud, lambda st, rs: len(rs) >= 1, SIM_TRAY_SEC + 6)
        cloud.send(cmd(m_id, 'PAUSE'))
        ok = wait_cloud(cloud, lambda st, rs: st and st[-1][1]['st'] == 'PAUSED', 5)
        check(ok, 'PAUSE -> status PAUSED di Cloud')
        n_before = len(cloud.snapshot(m_id)[1])
        cloud.send(cmd('M-LAIN', 'ABORT'))
        rclpy.spin_for(SIM_TRAY_SEC + 2)
        st, rs = cloud.snapshot(m_id)
        check(len(rs) == n_before, f'selama PAUSED tidak ada result baru ({len(rs)} == {n_before})')
        check(st[-1][1]['st'] == 'PAUSED', 'ABORT untuk m_id lain diabaikan (misi tetap PAUSED)')
        check(any(s[1]['st'] == 'PAUSED' for s in st[-2:]), 'status berkala tetap terkirim saat PAUSED')
        cloud.send(cmd(m_id, 'RESUME'))
        ok = wait_cloud(cloud, lambda st, rs: len([r for r in rs if r[1]['m_id'] == m_id]) >= n_before + 1,
                        SIM_TRAY_SEC + 6)
        check(ok, 'RESUME -> inspeksi berlanjut (result baru muncul)')
        cloud.send(cmd(m_id, 'ABORT'))
        ok = wait_cloud(cloud, lambda st, rs: st and st[-1][1]['st'] == 'ABORTED', 5)
        check(ok, 'ABORT -> status ABORTED di Cloud')
        n_abort = len(cloud.snapshot(m_id)[1])
        rclpy.spin_for(SIM_TRAY_SEC + 2)
        st, rs = cloud.snapshot(m_id)
        check(len(rs) == n_abort and n_abort < 4, f'setelah ABORT tidak ada result lagi ({n_abort} dari 4 tray)')
        check(st[-1][1]['st'] == 'ABORTED', 'tidak ada status berkala setelah ABORTED')
        check_contracts(check, st, rs)
    finally:
        stack.close()


def scenario_trigger_mode(args, cloud, check):
    """auto_inspect=false: tray hanya diinspeksi saat /mission/trigger_tray."""
    stack = Stack(args, {1: 'good', 2: 'bad', 3: 'bad'}, auto_inspect=False)
    try:
        cloud.clear()
        m_id = 'M-SIM-TRIG'
        cloud.send(cmd(m_id, 'INSPECT', [1, 2, 3]))
        rclpy.spin_for(SIM_TRAY_SEC + 2)
        check(not cloud.snapshot(m_id)[1], 'tanpa trigger tidak ada inspeksi')
        order = []
        for tray in (3, 9, 1, 2):
            stack.trigger(tray)
            if tray == 9:
                rclpy.spin_for(1.0)
                continue
            order.append(tray)
            wait_cloud(cloud, lambda st, rs, n=len(order): len([r for r in rs if r[1]['m_id'] == m_id]) >= n,
                       SIM_TRAY_SEC + 6)
        st, rs = cloud.snapshot(m_id)
        got = [r[1]['tray_id'] for r in rs]
        check(got == [3, 1, 2], f'tray diinspeksi sesuai trigger navigasi: {got} (tray 9 diabaikan)')
        labels = {r[1]['tray_id']: r[1]['label'] for r in rs}
        check(labels == {3: 'TIDAK BAGUS', 1: 'BAGUS', 2: 'TIDAK BAGUS'}, f'label sesuai tray yang di-trigger: {labels}')
        check(count_logs('collaboration_agent_node', 'trigger_tray 9 diabaikan') == 1, 'trigger tray di luar misi ditolak')
        wait_cloud(cloud, lambda st, rs: st and st[-1][1]['st'] == 'COMPLETED', 4)
        check(cloud.snapshot(m_id)[0][-1][1]['st'] == 'COMPLETED', 'misi COMPLETED setelah semua tray di-trigger')
    finally:
        stack.close()


def scenario_bad_payloads(args, cloud, check):
    stack = Stack(args, {1: 'good'})
    try:
        cloud.clear()
        before = count_logs('collaboration_agent_node', 'memproses perintah')
        big = cmd('M-SIM-BIG', 'INSPECT', list(range(1, 60)))
        cloud.send(big)                                                    # > 192 byte -> drop di Sink
        cloud.send('{"m_id":"M-SIM-NOCMD","h_id":"GH01","trays":[1],"ts":1}')  # tanpa cmd -> drop di Sink
        cloud.send('{"m_id":"M-SIM-BADTYPE","h_id":"GH01","cmd":"INSPECT","trays":"1,2","crit_lvl":1,"ts":1}')
        rclpy.spin_for(3)
        check(count_logs('collaboration_agent_node', 'memproses perintah') == before,
              'payload tidak valid tidak pernah sampai ke agent')
        check(count_logs('sink_bridge_node', 'REJECT') >= 1,
              'payload yang lolos Sink (cek ringan JsonLite) ditolak validasi schema di sink_bridge_node')
        m_id = 'M-SIM-DUP'
        dup = cmd(m_id, 'INSPECT', [1])
        cloud.send(dup)
        cloud.send(dup)   # Cloud kirim ulang command yang sama
        wait_cloud(cloud, lambda st, rs: any(s[1]['st'] == 'COMPLETED' and s[1]['m_id'] == m_id for s in st),
                   SIM_TRAY_SEC + 8)
        _, rs = cloud.snapshot(m_id)
        check(count_logs('collaboration_agent_node', 'Duplikat command diabaikan') == 1 and len(rs) == 1,
              f'command duplikat (kirim ulang) hanya dieksekusi 1x -> {len(rs)} result')
    finally:
        stack.close()


SCENARIOS = [
    ('1. Alur normal INSPECT 3 tray (auto)', scenario_happy_path),
    ('2. Kamera/visi mati -> UNKNOWN', scenario_no_vision),
    ('3. PAUSE / RESUME / ABORT', scenario_pause_resume_abort),
    ('4. Mode trigger navigasi (auto_inspect=false)', scenario_trigger_mode),
    ('5. Payload tidak valid & command duplikat', scenario_bad_payloads),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=1883)
    ap.add_argument('--model', default=DEFAULT_MODEL)
    ap.add_argument('--only', type=int, nargs='*', help='nomor skenario yang dijalankan')
    ap.add_argument('--quiet', action='store_true', help='sembunyikan log node')
    ap.add_argument('--report', default='', help='simpan ringkasan JSON ke file ini')
    args = ap.parse_args()
    rclpy.set_log_echo(not args.quiet)

    print(f'paho-mqtt {PAHO_VERSION} | model {os.path.basename(args.model)}')
    procs = []
    results = []
    cloud = None
    sink_log = os.path.join(HERE, 'rw612_sink', 'build', 'sink_node_sim.log')
    try:
        ensure_broker(args.host, args.port, procs)
        start_sink(args.host, args.port, sink_log, procs)
        print(f'Sink Node RW612 (firmware Java asli) aktif, log: {sink_log}')
        cloud = CloudSim(args.host, args.port)
        for i, (name, fn) in enumerate(SCENARIOS, 1):
            if args.only and i not in args.only:
                continue
            print(f'\n=== SKENARIO {name} ===', flush=True)
            check = Check()
            try:
                fn(args, cloud, check)
            except Exception as exc:  # noqa: BLE001
                traceback.print_exc()
                check(False, f'exception: {exc!r}')
            results.append({'name': name, 'ok': check.ok,
                            'checks': [{'ok': c, 'text': t} for c, t in check.items]})
    finally:
        if cloud:
            cloud.close()
        for p in procs:
            p.terminate()
    print('\n=== RINGKASAN ===')
    for r in results:
        n_ok = sum(c['ok'] for c in r['checks'])
        print(f'{"PASS" if r["ok"] else "FAIL"}  {r["name"]}  ({n_ok}/{len(r["checks"])} cek)')
    if args.report:
        with open(args.report, 'w') as fh:
            json.dump({'paho': PAHO_VERSION, 'scenarios': results}, fh, indent=2)
    sys.exit(0 if results and all(r['ok'] for r in results) else 1)


if __name__ == '__main__':
    main()
