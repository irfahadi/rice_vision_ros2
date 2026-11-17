"""Unit test node collaboration_agent & sink_bridge memakai shim ROS (sim/fake_ros).

Tidak butuh ROS 2, broker MQTT, maupun kamera. Waktu agent dikendalikan manual.
Jalankan: python3 -m pytest tests  (atau python3 -m unittest discover -s tests)
"""

import json
import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(ROOT, 'sim', 'fake_ros'))
sys.path.insert(0, os.path.join(ROOT, 'rice_vision'))

import rclpy  # noqa: E402  (shim)
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import BatteryState  # noqa: E402
from std_msgs.msg import Int32, String  # noqa: E402

from rice_vision_interfaces.msg import (  # noqa: E402
    Detection, DetectionArray, MissionCommand, MissionResult, MissionStatus)
from rice_vision.collaboration_agent_node import (  # noqa: E402
    CollaborationAgentNode, aggregate_samples, classify_label)
from rice_vision.sink_bridge_node import SinkBridgeNode  # noqa: E402

TRAY_SEC = 15.0
GRACE_SEC = 10.0


def det_array(class_name, conf):
    d = Detection(class_name=class_name, confidence=float(conf), class_id=0)
    return DetectionArray(detections=[d], count=1)


class AgentHarness:
    def __init__(self, **params):
        rclpy.reset()
        rclpy.init()
        rclpy.set_log_echo(False)
        base = {'status_interval_sec': 3.0, 'tray_inspection_interval_sec': TRAY_SEC,
                'detection_grace_sec': GRACE_SEC, 'simulate_battery_drain': False}
        base.update(params)
        rclpy.set_parameter_overrides('collaboration_agent_node', base)
        self.agent = CollaborationAgentNode()
        self.clock = 1000.0
        self.agent._now = lambda: self.clock
        self.results, self.statuses = [], []
        io = Node('test_io')
        io.create_subscription(MissionResult, '/mission/result', self.results.append, 10)
        io.create_subscription(MissionStatus, '/mission/status', self.statuses.append, 10)
        self.cmd_pub = io.create_publisher(MissionCommand, '/mission/command', 10)
        self.cmd_raw_pub = io.create_publisher(String, '/mission/command/raw', 10)
        self.det_pub = io.create_publisher(DetectionArray, '/rice_vision/detections', 10)
        self.trig_pub = io.create_publisher(Int32, '/mission/trigger_tray', 10)
        self.bat_pub = io.create_publisher(BatteryState, '/battery_state', 10)

    def pump(self):
        rclpy.spin_for(0.05)

    def send(self, m_id, cmd, trays=(), ts=1761200010):
        msg = MissionCommand(m_id=m_id, h_id='GH01', cmd=cmd, trays=list(trays), crit_lvl=1, ts=ts)
        self.cmd_pub.publish(msg)
        # sink_bridge_node juga mem-publish versi raw JSON dari command yang sama
        raw = {'m_id': m_id, 'h_id': 'GH01', 'cmd': cmd, 'trays': list(trays), 'crit_lvl': 1, 'ts': ts}
        self.cmd_raw_pub.publish(String(data=json.dumps(raw)))
        self.pump()

    def detect(self, name, conf, n=1):
        for _ in range(n):
            self.det_pub.publish(det_array(name, conf))
        self.pump()

    def advance(self, seconds):
        self.clock += seconds
        self.agent.on_inspect_timer()
        self.pump()


class TestHelpers(unittest.TestCase):
    def test_classify_label(self):
        self.assertEqual(classify_label('Bibit Padi (Bagus)'), ('BAGUS', 0))
        self.assertEqual(classify_label('Bibit Padi (Tidak Bagus)'), ('TIDAK BAGUS', 1))
        self.assertEqual(classify_label('bibit_rusak'), ('TIDAK BAGUS', 1))

    def test_aggregate_majority_and_tie(self):
        self.assertIsNone(aggregate_samples([]))
        s = [('BAGUS', 0.9, 0), ('BAGUS', 0.8, 0), ('TIDAK BAGUS', 0.99, 1)]
        self.assertEqual(aggregate_samples(s), ('BAGUS', 0.85, 0))
        tie = [('BAGUS', 0.9, 0), ('TIDAK BAGUS', 0.6, 1)]
        self.assertEqual(aggregate_samples(tie)[0], 'TIDAK BAGUS')


class TestAgent(unittest.TestCase):
    def test_command_processed_once_even_if_typed_and_raw_arrive(self):
        h = AgentHarness()
        h.send('M1', 'INSPECT', [1, 2])
        logs = [t for _, n, _, t in rclpy.logs() if 'memproses perintah' in t]
        self.assertEqual(len(logs), 1)
        h.send('M1', 'INSPECT', [1, 2])  # kirim ulang identik (redelivery)
        self.assertTrue(any('Duplikat command' in t for _, _, _, t in rclpy.logs()))
        self.assertEqual(len([t for _, _, _, t in rclpy.logs() if 'memproses perintah' in t]), 1)

    def test_tray_result_uses_samples_from_that_tray_only(self):
        h = AgentHarness()
        h.detect('Bibit Padi (Tidak Bagus)', 0.99, n=5)  # sebelum misi -> tidak dihitung
        h.send('M1', 'INSPECT', [1, 2])
        h.detect('Bibit Padi (Bagus)', 0.9, n=3)
        h.detect('Bibit Padi (Tidak Bagus)', 0.7, n=1)
        h.advance(TRAY_SEC - 1)
        self.assertEqual(h.results, [])
        h.advance(1.0)
        self.assertEqual(len(h.results), 1)
        r = h.results[0]
        self.assertEqual((r.tray_id, r.label, r.anom, r.seq), (1, 'BAGUS', 0, 1))
        self.assertAlmostEqual(r.conf, 0.9, places=5)
        # tray 2 mulai dengan buffer kosong
        h.detect('Bibit Padi (Tidak Bagus)', 0.95, n=2)
        h.advance(TRAY_SEC)
        self.assertEqual((h.results[1].tray_id, h.results[1].label), (2, 'TIDAK BAGUS'))
        self.assertEqual(h.statuses[-1].st, 'COMPLETED')
        self.assertEqual(h.statuses[-1].pct, 100)

    def test_no_detection_waits_grace_then_reports_unknown(self):
        h = AgentHarness()
        h.send('M1', 'INSPECT', [4])
        h.advance(TRAY_SEC)
        h.advance(GRACE_SEC - 1)
        self.assertEqual(h.results, [], 'tidak boleh langsung mengarang hasil')
        h.advance(1.0)
        r = h.results[0]
        self.assertEqual((r.label, r.conf, r.anom), ('UNKNOWN', 0.0, 1))

    def test_late_detection_within_grace_is_used(self):
        h = AgentHarness()
        h.send('M1', 'INSPECT', [4])
        h.advance(TRAY_SEC + 2)
        h.detect('Bibit Padi (Bagus)', 0.8)
        h.advance(0.5)
        self.assertEqual(h.results[0].label, 'BAGUS')

    def test_trigger_tray_inspects_requested_tray(self):
        h = AgentHarness(auto_inspect=False)
        h.send('M1', 'INSPECT', [1, 2, 3])
        h.advance(TRAY_SEC * 2)
        self.assertEqual(h.results, [], 'mode trigger: tidak ada inspeksi otomatis')
        h.trig_pub.publish(Int32(data=3))
        h.pump()
        h.detect('Bibit Padi (Tidak Bagus)', 0.9)
        h.advance(TRAY_SEC)
        self.assertEqual(h.results[0].tray_id, 3)
        h.trig_pub.publish(Int32(data=3))  # sudah selesai -> ditolak
        h.trig_pub.publish(Int32(data=99))  # bukan bagian misi -> ditolak
        h.pump()
        h.advance(TRAY_SEC + GRACE_SEC)
        self.assertEqual(len(h.results), 1)
        self.assertEqual(h.statuses[-1].pct, 33)

    def test_control_commands_require_matching_mission(self):
        h = AgentHarness()
        h.send('M1', 'INSPECT', [1, 2])
        h.send('M2', 'ABORT', ts=1761200020)
        self.assertEqual(h.agent.state, 'IN_PROGRESS')
        h.send('M1', 'PAUSE', ts=1761200030)
        self.assertEqual(h.agent.state, 'PAUSED')
        h.advance(TRAY_SEC * 3)
        self.assertEqual(h.results, [])
        h.send('M1', 'RESUME', ts=1761200040)
        h.detect('Bibit Padi (Bagus)', 0.9)
        h.advance(TRAY_SEC)
        self.assertEqual(len(h.results), 1)
        h.send('M1', 'ABORT', ts=1761200050)
        self.assertEqual(h.agent.state, 'ABORTED')

    def test_battery_nan_ignored(self):
        h = AgentHarness(battery_initial=84)
        h.bat_pub.publish(BatteryState(percentage=float('nan')))
        h.pump()
        self.assertEqual(h.agent.battery_lvl, 84)
        h.bat_pub.publish(BatteryState(percentage=0.5))
        h.pump()
        self.assertEqual(h.agent.battery_lvl, 50)


class TestBridgeWiring(unittest.TestCase):
    def test_bridge_uses_rw612_topics_and_single_ros_path(self):
        rclpy.reset()
        rclpy.init()
        rclpy.set_log_echo(False)
        rclpy.set_parameter_overrides('sink_bridge_node', {'transport': 'none'})
        bridge = SinkBridgeNode()
        self.assertEqual(bridge.t_cmd, 'gh01/agent/mission/command')
        self.assertEqual(bridge.t_status, 'gh01/agent/mission/status')
        self.assertEqual(bridge.t_result, 'gh01/agent/mission/result')
        subscribed = {s.topic for s in bridge._subscriptions}
        self.assertEqual(subscribed, {'/mission/status', '/mission/result'},
                         'bridge tidak boleh juga subscribe /raw (penyebab kirim ganda)')
        bridge.destroy_node()


if __name__ == '__main__':
    unittest.main()
