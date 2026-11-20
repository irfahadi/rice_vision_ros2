"""Tests for MQTT mission reception and collaboration end-to-end flow."""

import json
import os
import sys
import unittest
from unittest.mock import MagicMock

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(ROOT, 'sim', 'fake_ros'))
sys.path.insert(0, os.path.join(ROOT, 'rice_vision'))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rice_vision_interfaces.msg import Detection, DetectionArray, MissionCommand, MissionResult, MissionStatus
from rice_vision.sink_bridge_node import SinkBridgeNode
from rice_vision.collaboration_agent_node import CollaborationAgentNode
from rice_vision.camera_node import CameraNode
from rice_vision import image_codec


class TestCollaborationMqtt(unittest.TestCase):
    def setUp(self):
        rclpy.reset()
        rclpy.init()
        rclpy.set_log_echo(False)

    def tearDown(self):
        rclpy.shutdown()

    def test_bridge_subscribes_all_command_variants(self):
        rclpy.set_parameter_overrides('sink_bridge_node', {
            'transport': 'none',
            'greenhouse_id': 'GH01',
        })
        bridge = SinkBridgeNode()
        self.assertIn('gh01/agent/mission/command', bridge.cmd_sub_topics)
        self.assertIn('gh01/mission/command', bridge.cmd_sub_topics)
        self.assertIn('mission/command', bridge.cmd_sub_topics)
        bridge.destroy_node()

    def test_bridge_incoming_command_direct_and_relay(self):
        rclpy.set_parameter_overrides('sink_bridge_node', {
            'transport': 'none',
            'greenhouse_id': 'GH01',
        })
        bridge = SinkBridgeNode()
        received_ros_cmds = []

        io = Node('test_listener')
        io.create_subscription(MissionCommand, '/mission/command', received_ros_cmds.append, 10)

        # 1. Pesan dari direct cloud topic: gh01/mission/command
        payload_1 = {
            "m_id": "M20261023-01",
            "h_id": "GH01",
            "cmd": "INSPECT",
            "trays": [1],
            "crit_lvl": 1,
            "ts": 1761200010,
        }
        raw_bytes_1 = json.dumps(payload_1).encode('utf-8')

        msg_mock_1 = MagicMock()
        msg_mock_1.topic = 'gh01/mission/command'
        msg_mock_1.payload = raw_bytes_1

        bridge._on_mqtt_message(None, None, msg_mock_1)
        rclpy.spin_for(0.05)

        self.assertEqual(len(received_ros_cmds), 1)
        self.assertEqual(received_ros_cmds[0].m_id, "M20261023-01")
        self.assertEqual(received_ros_cmds[0].h_id, "GH01")
        self.assertEqual(received_ros_cmds[0].cmd, "INSPECT")
        self.assertEqual(received_ros_cmds[0].trays, [1])

        bridge.destroy_node()

    def test_bridge_publishes_result_to_direct_and_relay_topics(self):
        rclpy.set_parameter_overrides('sink_bridge_node', {
            'transport': 'mqtt',
            'greenhouse_id': 'GH01',
            'publish_direct_cloud': True,
        })
        bridge = SinkBridgeNode()
        # Mock mqtt client
        mock_mqtt = MagicMock()
        bridge.mqtt_client = mock_mqtt
        bridge.is_connected = True

        res = MissionResult(
            m_id="M20261023-01",
            h_id="GH01",
            tray_id=1,
            label="BAGUS",
            conf=0.94,
            anom=0,
            seq=3,
            ts=1761200050,
        )
        bridge.on_ros_result(res)

        # Periksa topik yang dipublish oleh mock MQTT client
        calls = mock_mqtt.publish.call_args_list
        published_topics = [c[0][0] for c in calls]
        self.assertIn('gh01/agent/mission/result', published_topics)
        self.assertIn('gh01/mission/result', published_topics)

        # Periksa payload yang dikirim
        sent_payload = json.loads(calls[0][0][1].decode('utf-8'))
        self.assertEqual(sent_payload['m_id'], "M20261023-01")
        self.assertEqual(sent_payload['h_id'], "GH01")
        self.assertEqual(sent_payload['tray_id'], 1)
        self.assertEqual(sent_payload['label'], "BAGUS")
        self.assertEqual(sent_payload['conf'], 0.94)
        self.assertEqual(sent_payload['anom'], 0)
        self.assertEqual(sent_payload['seq'], 3)
        self.assertEqual(sent_payload['ts'], 1761200050)

        bridge.destroy_node()

    def test_end_to_end_collaboration_mission_flow(self):
        """Uji alur lengkap: MQTT command -> ROS bridge -> Agent -> Detections -> Result -> MQTT."""
        rclpy.set_parameter_overrides('sink_bridge_node', {
            'transport': 'mqtt',
            'greenhouse_id': 'GH01',
            'publish_direct_cloud': True,
        })
        rclpy.set_parameter_overrides('collaboration_agent_node', {
            'robot_id': 'RB01',
            'greenhouse_id': 'GH01',
            'tray_inspection_interval_sec': 5.0,
            'detection_grace_sec': 5.0,
            'simulate_battery_drain': False,
        })

        bridge = SinkBridgeNode()
        mock_mqtt = MagicMock()
        bridge.mqtt_client = mock_mqtt
        bridge.is_connected = True

        agent = CollaborationAgentNode()
        clock = 1761200010.0
        agent._now = lambda: clock

        det_pub = agent.create_publisher(DetectionArray, '/rice_vision/detections', 10)

        # 1. Cloud mengirim payload misi melalui MQTT
        cmd_payload = {
            "m_id": "M20261023-01",
            "h_id": "GH01",
            "cmd": "INSPECT",
            "trays": [1],
            "crit_lvl": 1,
            "ts": 1761200010,
        }
        msg_mock = MagicMock()
        msg_mock.topic = 'gh01/mission/command'
        msg_mock.payload = json.dumps(cmd_payload).encode('utf-8')

        bridge._on_mqtt_message(None, None, msg_mock)
        rclpy.spin_for(0.05)

        self.assertEqual(agent.mission_id, "M20261023-01")
        self.assertEqual(agent.state, "IN_PROGRESS")
        self.assertEqual(agent.current_tray_id, 1)

        # 2. Kamera dan Model CV mendeteksi bibit padi
        d = Detection(class_name='Bibit Padi (Bagus)', confidence=0.94, class_id=0)
        det_arr = DetectionArray(detections=[d], count=1)
        det_pub.publish(det_arr)
        rclpy.spin_for(0.05)

        self.assertEqual(len(agent.tray_samples), 1)

        # 3. Waktu inspeksi tray selesai
        clock += 5.5
        agent.on_inspect_timer()
        rclpy.spin_for(0.05)

        # 4. Verifikasi bahwa result dipublish kembali ke Cloud melalui MQTT
        result_calls = [c for c in mock_mqtt.publish.call_args_list if 'result' in c[0][0]]
        self.assertTrue(len(result_calls) >= 1)
        res_payload = json.loads(result_calls[0][0][1].decode('utf-8'))
        self.assertEqual(res_payload['m_id'], "M20261023-01")
        self.assertEqual(res_payload['h_id'], "GH01")
        self.assertEqual(res_payload['tray_id'], 1)
        self.assertEqual(res_payload['label'], "BAGUS")
        self.assertAlmostEqual(res_payload['conf'], 0.94, places=2)
        self.assertEqual(res_payload['anom'], 0)

        bridge.destroy_node()
        agent.destroy_node()

    def test_camera_dummy_fallback(self):
        """Memastikan camera_node tidak crash bila camera_id invalid dan dummy_fallback aktif."""
        rclpy.set_parameter_overrides('camera_node', {
            'camera_id': 9999,  # index kamera tidak ada
            'dummy_fallback': True,
        })
        cam = CameraNode()
        self.assertTrue(cam.use_dummy)
        frame = cam._generate_dummy_frame()
        self.assertEqual(frame.shape, (cam.height, cam.width, 3))
        cam.destroy_node()


if __name__ == '__main__':
    unittest.main()
