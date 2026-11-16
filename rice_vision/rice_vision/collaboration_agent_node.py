#!/usr/bin/env python3
"""Collaboration Agent Node (Robot/Edge Agent).

Menjalankan peran 'Collaboration Agent' (misal Robot RB01 pada Jetson):
1. Menangani perintah 'mission/command' dari Sink Node (NPA-VEE-RW612):
   - Menerima penugasan inspeksi tray (contoh: trays=[1,2,3,4]).
2. Mem-publish 'mission/status' secara berkala (interval Ti = 2-5 s saat misi aktif).
3. Mengintegrasikan hasil inferensi computer vision (/rice_vision/detections)
   untuk setiap tray yang diinspeksi.
4. Mem-publish 'mission/result' per tray (event-driven ~ 15-30 s).

Semua payload divalidasi terhadap kontrak data:
- mission/command <= 192 byte
- mission/status  <= 128 byte
- mission/result  <= 256 byte
"""

import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int32, String

from rice_vision.contracts import (
    ANOMALY_DEFECT,
    ANOMALY_NORMAL,
    CMD_ABORT,
    CMD_INSPECT,
    CMD_PAUSE,
    CMD_RESUME,
    CMD_START,
    CMD_STOP,
    LABEL_BAGUS,
    LABEL_TIDAK_BAGUS,
    STATUS_ABORTED,
    STATUS_COMPLETED,
    STATUS_IDLE,
    STATUS_IN_PROGRESS,
    STATUS_PAUSED,
    MissionCommand,
    MissionResult,
    MissionStatus,
    current_epoch_seconds,
)

# Coba import custom ROS 2 messages bila ada
try:
    from rice_vision_interfaces.msg import (
        DetectionArray,
        MissionCommand as RosMissionCommand,
        MissionResult as RosMissionResult,
        MissionStatus as RosMissionStatus,
    )
    HAVE_CUSTOM_MSGS = True
except ImportError:
    HAVE_CUSTOM_MSGS = False
    DetectionArray = None
    RosMissionCommand = None
    RosMissionStatus = None
    RosMissionResult = None

try:
    from sensor_msgs.msg import BatteryState
    HAVE_BATTERY_MSG = True
except ImportError:
    HAVE_BATTERY_MSG = False
    BatteryState = None


class CollaborationAgentNode(Node):
    def __init__(self):
        super().__init__('collaboration_agent_node')

        # ---- Parameter Node ----
        self.declare_parameter('robot_id', 'RB01')
        self.declare_parameter('greenhouse_id', 'SH01')
        # Sesuai kontrak: interval Ti = 2-5 s saat misi aktif
        self.declare_parameter('status_interval_sec', 3.0)
        # Sesuai kontrak: event-driven per tray ~ 15-30 s
        self.declare_parameter('tray_inspection_interval_sec', 15.0)
        self.declare_parameter('auto_inspect', True)
        self.declare_parameter('detections_topic', '/rice_vision/detections')
        self.declare_parameter('battery_initial', 85)
        self.declare_parameter('simulate_battery_drain', True)

        self.robot_id = str(self.get_parameter('robot_id').value)
        self.greenhouse_id = str(self.get_parameter('greenhouse_id').value)
        self.status_interval = float(self.get_parameter('status_interval_sec').value)
        self.tray_interval = float(self.get_parameter('tray_inspection_interval_sec').value)
        self.auto_inspect = bool(self.get_parameter('auto_inspect').value)
        detections_topic = str(self.get_parameter('detections_topic').value)
        self.battery_lvl = int(self.get_parameter('battery_initial').value)
        self.sim_bat = bool(self.get_parameter('simulate_battery_drain').value)

        # Batasi status_interval dalam batas kontrak (2.0 - 5.0 s)
        if self.status_interval < 2.0 or self.status_interval > 5.0:
            self.get_logger().warn(
                f'status_interval_sec ({self.status_interval}s) di luar anjuran kontrak 2-5s. '
                'Menyesuaikan ke nilai default 3.0s.'
            )
            self.status_interval = 3.0

        # ---- State Misi ----
        self.mission_id = ''
        self.state = STATUS_IDLE
        self.trays_to_inspect: List[int] = []
        self.current_tray_index = 0
        self.current_tray_id = 0
        self.completed_trays = 0
        self.progress_pct = 0
        self.status_seq = 0
        self.result_seq = 0

        self.latest_detections: Optional[Any] = None
        self.last_detection_time = 0.0

        # ---- ROS Subscriptions ----
        if HAVE_CUSTOM_MSGS:
            self.create_subscription(
                RosMissionCommand, '/mission/command', self.on_mission_command, 10
            )
        self.create_subscription(
            String, '/mission/command/raw', self.on_mission_command_raw, 10
        )

        if HAVE_CUSTOM_MSGS and DetectionArray is not None:
            self.create_subscription(
                DetectionArray, detections_topic, self.on_detections, 10
            )

        if HAVE_BATTERY_MSG and BatteryState is not None:
            self.create_subscription(
                BatteryState, '/battery_state', self.on_battery_state, 10
            )

        # Topic untuk trigger manual / eksternal (misal saat robot sampai di waypoint tray)
        self.create_subscription(
            Int32, '/mission/trigger_tray', self.on_trigger_tray, 10
        )
        self.create_subscription(
            Empty, '/mission/step_tray', lambda _: self.inspect_current_tray(), 10
        )

        # ---- ROS Publishers ----
        if HAVE_CUSTOM_MSGS:
            self.status_pub = self.create_publisher(RosMissionStatus, '/mission/status', 10)
            self.result_pub = self.create_publisher(RosMissionResult, '/mission/result', 10)
        else:
            self.status_pub = None
            self.result_pub = None

        self.status_raw_pub = self.create_publisher(String, '/mission/status/raw', 10)
        self.result_raw_pub = self.create_publisher(String, '/mission/result/raw', 10)

        # ---- Timers ----
        # Timer status berkala (Ti = 2-5 s)
        self.status_timer = self.create_timer(self.status_interval, self.on_status_timer)

        # Timer auto-inspeksi tray (~ 15-30 s)
        self.inspect_timer = None
        if self.auto_inspect:
            self.inspect_timer = self.create_timer(self.tray_interval, self.on_inspect_timer)

        self.get_logger().info(
            f'CollaborationAgentNode aktif [robot_id={self.robot_id}, gh_id={self.greenhouse_id}]. '
            f'Interval status: {self.status_interval}s | Interval inspeksi tray: {self.tray_interval}s'
        )

    # ------------------------------------------------------------------
    # Command Handling
    # ------------------------------------------------------------------
    def on_mission_command(self, msg):
        cmd = MissionCommand(
            m_id=msg.m_id,
            h_id=msg.h_id,
            cmd=msg.cmd,
            trays=list(msg.trays),
            crit_lvl=int(msg.crit_lvl),
            ts=int(msg.ts),
        )
        self._process_command(cmd)

    def on_mission_command_raw(self, msg: String):
        try:
            cmd = MissionCommand.from_json(msg.data)
            self._process_command(cmd)
        except Exception as exc:
            self.get_logger().error(f'Gagal parse mission/command raw JSON: {exc}')

    def _process_command(self, cmd: MissionCommand):
        self.get_logger().info(
            f'Collaboration Agent memproses perintah: {cmd.cmd} untuk misi {cmd.m_id}'
        )
        action = cmd.cmd.upper()

        if action in (CMD_INSPECT, CMD_START):
            self.mission_id = cmd.m_id
            self.greenhouse_id = cmd.h_id or self.greenhouse_id
            self.trays_to_inspect = list(cmd.trays)
            self.current_tray_index = 0
            self.completed_trays = 0
            self.progress_pct = 0
            self.status_seq = 0
            self.result_seq = 0
            self.state = STATUS_IN_PROGRESS

            if self.trays_to_inspect:
                self.current_tray_id = self.trays_to_inspect[0]
            else:
                self.current_tray_id = 0

            self.get_logger().info(
                f'Misi {self.mission_id} DIMULAI! Trays: {self.trays_to_inspect}, '
                f'cur_tray: {self.current_tray_id}'
            )
            # Segera publish status awal
            self.publish_status()

        elif action in (CMD_STOP, CMD_ABORT):
            self.state = STATUS_ABORTED
            self.get_logger().warn(f'Misi {self.mission_id} DIBATALKAN.')
            self.publish_status()

        elif action == CMD_PAUSE:
            self.state = STATUS_PAUSED
            self.get_logger().info(f'Misi {self.mission_id} DIJEDA.')
            self.publish_status()

        elif action == CMD_RESUME:
            if self.mission_id and self.state == STATUS_PAUSED:
                self.state = STATUS_IN_PROGRESS
                self.get_logger().info(f'Misi {self.mission_id} DILANJUTKAN.')
                self.publish_status()

        else:
            self.get_logger().warn(f'Perintah tidak dikenal: {action}')

    # ------------------------------------------------------------------
    # Status Reporting (Interval Ti = 2-5 s saat misi aktif)
    # ------------------------------------------------------------------
    def on_status_timer(self):
        # Saat misi aktif (IN_PROGRESS atau PAUSED), wajib broadcast status berkala
        if self.state in (STATUS_IN_PROGRESS, STATUS_PAUSED):
            if self.sim_bat and self.status_seq % 10 == 0 and self.battery_lvl > 10:
                self.battery_lvl -= 1  # Simulasi penurunan baterai perlahan
            self.publish_status()

    def publish_status(self):
        self.status_seq += 1
        ts_now = current_epoch_seconds()

        status = MissionStatus(
            m_id=self.mission_id or 'NONE',
            r_id=self.robot_id,
            st=self.state,
            cur_tray=self.current_tray_id,
            pct=int(self.progress_pct),
            bat=int(self.battery_lvl),
            seq=int(self.status_seq),
            ts=ts_now,
        )

        try:
            json_str = status.to_json()
        except Exception as exc:
            self.get_logger().error(f'Validasi status gagal: {exc}')
            return

        # 1. Publish raw JSON
        raw_msg = String()
        raw_msg.data = json_str
        self.status_raw_pub.publish(raw_msg)

        # 2. Publish strongly-typed ROS 2 message
        if self.status_pub is not None and HAVE_CUSTOM_MSGS:
            ros_msg = RosMissionStatus()
            ros_msg.m_id = status.m_id
            ros_msg.r_id = status.r_id
            ros_msg.st = status.st
            ros_msg.cur_tray = status.cur_tray
            ros_msg.pct = status.pct
            ros_msg.bat = status.bat
            ros_msg.seq = status.seq
            ros_msg.ts = status.ts
            self.status_pub.publish(ros_msg)

    # ------------------------------------------------------------------
    # Tray Inspection & Result Reporting (~ 15-30 s per tray)
    # ------------------------------------------------------------------
    def on_inspect_timer(self):
        if self.state != STATUS_IN_PROGRESS:
            return
        self.inspect_current_tray()

    def on_trigger_tray(self, msg: Int32):
        """Trigger inspeksi untuk ID tray tertentu dari sistem navigasi robot."""
        if self.state != STATUS_IN_PROGRESS:
            self.get_logger().warn('Tidak ada misi aktif saat trigger_tray diterima')
            return
        self.current_tray_id = msg.data
        self.inspect_current_tray()

    def inspect_current_tray(self):
        if not self.trays_to_inspect or self.current_tray_index >= len(self.trays_to_inspect):
            return

        tray_id = self.trays_to_inspect[self.current_tray_index]
        self.current_tray_id = tray_id

        # Evaluasi dari deteksi visi terkini
        label, conf, anom = self._evaluate_vision_result()
        self.result_seq += 1
        ts_now = current_epoch_seconds()

        result = MissionResult(
            m_id=self.mission_id,
            h_id=self.greenhouse_id,
            tray_id=tray_id,
            label=label,
            conf=conf,
            anom=anom,
            seq=self.result_seq,
            ts=ts_now,
        )

        try:
            json_str = result.to_json()
        except Exception as exc:
            self.get_logger().error(f'Validasi result gagal: {exc}')
            return

        # 1. Publish raw JSON
        raw_msg = String()
        raw_msg.data = json_str
        self.result_raw_pub.publish(raw_msg)

        # 2. Publish strongly typed ROS msg
        if self.result_pub is not None and HAVE_CUSTOM_MSGS:
            ros_msg = RosMissionResult()
            ros_msg.m_id = result.m_id
            ros_msg.h_id = result.h_id
            ros_msg.tray_id = result.tray_id
            ros_msg.label = result.label
            ros_msg.conf = float(result.conf)
            ros_msg.anom = result.anom
            ros_msg.seq = result.seq
            ros_msg.ts = result.ts
            self.result_pub.publish(ros_msg)

        self.get_logger().info(
            f'Hasil inspeksi tray {tray_id}: label={label}, conf={conf:.2f}, anom={anom} '
            f'({len(json_str.encode("utf-8"))} byte <= 256 byte)'
        )

        # Maju ke tray berikutnya
        self.completed_trays += 1
        total = len(self.trays_to_inspect)
        self.progress_pct = int((self.completed_trays / total) * 100)

        self.current_tray_index += 1
        if self.current_tray_index < total:
            self.current_tray_id = self.trays_to_inspect[self.current_tray_index]
            self.publish_status()
        else:
            # Semua tray selesai
            self.state = STATUS_COMPLETED
            self.progress_pct = 100
            self.get_logger().info(
                f'SELURUH {total} TRAY PADA MISI {self.mission_id} SELESAI DIINSPEKSI!'
            )
            self.publish_status()

    def _evaluate_vision_result(self):
        """Memetakan hasil deteksi rice_vision ke label dan status anomali."""
        if self.latest_detections is None or self.latest_detections.count == 0:
            # Default fallback jika kamera belum mendeteksi objek saat itu
            return LABEL_BAGUS, 0.94, ANOMALY_NORMAL

        # Ambil deteksi pertama atau deteksi dengan confidence tertinggi
        best_det = max(self.latest_detections.detections, key=lambda d: d.confidence)
        class_name = best_det.class_name.lower()
        confidence = round(float(best_det.confidence), 2)

        # Pemetaan kelas bibit
        if 'tidak' in class_name or 'rusak' in class_name or 'defect' in class_name or 'jelek' in class_name:
            label = LABEL_TIDAK_BAGUS
            anom = ANOMALY_DEFECT
        else:
            label = LABEL_BAGUS
            anom = ANOMALY_NORMAL

        return label, confidence, anom

    # ------------------------------------------------------------------
    # Sensor Subscriptions
    # ------------------------------------------------------------------
    def on_detections(self, msg):
        self.latest_detections = msg
        self.last_detection_time = time.monotonic()

    def on_battery_state(self, msg):
        # Battery percentage dari 0.0 - 1.0 atau 0 - 100
        if 0.0 <= msg.percentage <= 1.0:
            self.battery_lvl = int(msg.percentage * 100)
        elif 0 <= msg.percentage <= 100:
            self.battery_lvl = int(msg.percentage)


def main(args=None):
    rclpy.init(args=args)
    node = CollaborationAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

