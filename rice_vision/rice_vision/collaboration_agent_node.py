#!/usr/bin/env python3
"""Collaboration Agent Node (Robot/Edge Agent).

Menjalankan peran 'Collaboration Agent' (misal Robot RB01 pada Jetson):
1. Menangani perintah 'mission/command' dari Sink Node (NPA-VEE-RW612):
   - Menerima penugasan inspeksi tray (contoh: trays=[1,2,3,4]).
2. Mem-publish 'mission/status' secara berkala (interval Ti = 2-5 s saat misi aktif).
3. Mengintegrasikan hasil inferensi computer vision (/rice_vision/detections)
   untuk setiap tray yang diinspeksi.
4. Mem-publish 'mission/result' per tray (event-driven ~ 15-30 s).

Alur inspeksi per tray:
- Saat tray mulai diinspeksi, buffer sampel deteksi dikosongkan, lalu agent
  mengumpulkan deteksi selama `tray_inspection_interval_sec`.
- Label tray = voting mayoritas sampel yang masuk SELAMA tray itu diinspeksi
  (bukan frame lama dari tray sebelumnya); conf = rata-rata conf label pemenang.
- Bila tidak ada sampel sama sekali, agent menunggu tambahan
  `detection_grace_sec`. Bila tetap kosong, hasil dikirim sebagai
  label=UNKNOWN, conf=0.0, anom=1 (perlu dicek manual) -- agent tidak pernah
  mengarang hasil "BAGUS".

Mode navigasi:
- auto_inspect=true : tray diproses berurutan sesuai daftar pada command.
- auto_inspect=false: agent menunggu /mission/trigger_tray (Int32 = tray_id)
  dari sistem navigasi saat robot tiba di tray tersebut.
- /mission/step_tray (Empty): paksa evaluasi tray saat ini sekarang juga.

Semua payload divalidasi terhadap kontrak data:
- mission/command <= 192 byte
- mission/status  <= 128 byte
- mission/result  <= 256 byte
"""

import math
import time
from collections import deque
from typing import Any, Deque, List, Optional, Tuple

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
    LABEL_UNKNOWN,
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

# Kata kunci nama kelas model yang dianggap bibit tidak bagus / anomali.
DEFECT_KEYWORDS = ('tidak', 'rusak', 'defect', 'jelek', 'bad', 'buruk')

# Berapa command terakhir yang diingat untuk membuang duplikat
# (mis. QoS-1 redelivery dari broker atau command yang terkirim ulang).
COMMAND_DEDUP_HISTORY = 16


def classify_label(class_name: str) -> Tuple[str, int]:
    """Petakan nama kelas model ke (label kontrak, anom)."""
    name = class_name.lower()
    if any(k in name for k in DEFECT_KEYWORDS):
        return LABEL_TIDAK_BAGUS, ANOMALY_DEFECT
    return LABEL_BAGUS, ANOMALY_NORMAL


def aggregate_samples(samples) -> Optional[Tuple[str, float, int]]:
    """Voting mayoritas dari sampel (label, conf, anom).

    Return (label, conf_rata2_pemenang, anom) atau None bila kosong.
    Bila seri, label TIDAK BAGUS yang menang (lebih aman untuk inspeksi).
    """
    if not samples:
        return None
    buckets = {}
    for label, conf, anom in samples:
        b = buckets.setdefault(label, [0, 0.0, anom])
        b[0] += 1
        b[1] += conf
    def rank(item):
        label, (count, _, _) = item
        return (count, 1 if label == LABEL_TIDAK_BAGUS else 0)
    label, (count, conf_sum, anom) = max(buckets.items(), key=rank)
    conf = round(min(max(conf_sum / count, 0.0), 1.0), 2)
    return label, conf, anom


class CollaborationAgentNode(Node):
    def __init__(self):
        super().__init__('collaboration_agent_node')

        # ---- Parameter Node ----
        self.declare_parameter('robot_id', 'RB01')
        self.declare_parameter('greenhouse_id', 'GH01')
        # Sesuai kontrak: interval Ti = 2-5 s saat misi aktif
        self.declare_parameter('status_interval_sec', 3.0)
        # Sesuai kontrak: event-driven per tray ~ 15-30 s
        self.declare_parameter('tray_inspection_interval_sec', 15.0)
        # Waktu tunggu tambahan bila belum ada deteksi sama sekali di tray ini
        self.declare_parameter('detection_grace_sec', 10.0)
        self.declare_parameter('auto_inspect', True)
        self.declare_parameter('detections_topic', '/rice_vision/detections')
        self.declare_parameter('battery_initial', 84)
        self.declare_parameter('simulate_battery_drain', True)
        self.declare_parameter('min_samples_for_finish', 0)

        self.robot_id = str(self.get_parameter('robot_id').value)
        self.greenhouse_id = str(self.get_parameter('greenhouse_id').value)
        self.status_interval = float(self.get_parameter('status_interval_sec').value)
        self.tray_interval = float(self.get_parameter('tray_inspection_interval_sec').value)
        self.detection_grace = float(self.get_parameter('detection_grace_sec').value)
        self.auto_inspect = bool(self.get_parameter('auto_inspect').value)
        detections_topic = str(self.get_parameter('detections_topic').value)
        self.battery_lvl = int(self.get_parameter('battery_initial').value)
        self.sim_bat = bool(self.get_parameter('simulate_battery_drain').value)
        self.min_samples_for_finish = int(self.get_parameter('min_samples_for_finish').value)

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
        self.trays_to_inspect: List[int] = []   # urutan asli dari command
        self.pending_trays: List[int] = []      # belum diinspeksi
        self.current_tray_id = 0
        self.completed_trays = 0
        self.progress_pct = 0
        self.status_seq = 0
        self.result_seq = 0

        # Inspeksi tray yang sedang berjalan
        self.tray_active = False
        self.tray_started_at = 0.0
        self.tray_samples: List[Tuple[str, float, int]] = []

        self._recent_commands: Deque[Tuple[str, str, int]] = deque(maxlen=COMMAND_DEDUP_HISTORY)
        self.latest_detections: Optional[Any] = None
        self.last_detection_time = 0.0

        # ---- ROS Subscriptions ----
        # sink_bridge_node mem-publish command ke topic typed DAN raw. Agent
        # hanya berlangganan SATU jalur supaya tiap command diproses sekali.
        if HAVE_CUSTOM_MSGS:
            self.create_subscription(
                RosMissionCommand, '/mission/command', self.on_mission_command, 10
            )
        else:
            self.create_subscription(
                String, '/mission/command/raw', self.on_mission_command_raw, 10
            )

        if HAVE_CUSTOM_MSGS and DetectionArray is not None:
            self.create_subscription(
                DetectionArray, detections_topic, self.on_detections, 10
            )
        else:
            self.get_logger().warn(
                'rice_vision_interfaces tidak tersedia: deteksi visi tidak bisa '
                'diterima, semua tray akan dilaporkan UNKNOWN.'
            )

        if HAVE_BATTERY_MSG and BatteryState is not None:
            self.create_subscription(
                BatteryState, '/battery_state', self.on_battery_state, 10
            )

        # Trigger dari sistem navigasi saat robot tiba di tray tertentu
        self.create_subscription(
            Int32, '/mission/trigger_tray', self.on_trigger_tray, 10
        )
        self.create_subscription(
            Empty, '/mission/step_tray', self.on_step_tray, 10
        )

        # ---- ROS Publishers ----
        if HAVE_CUSTOM_MSGS:
            self.status_pub = self.create_publisher(RosMissionStatus, '/mission/status', 10)
            self.result_pub = self.create_publisher(RosMissionResult, '/mission/result', 10)
        else:
            self.status_pub = None
            self.result_pub = None

        # Raw JSON selalu di-publish (dipakai viewer_node & debugging)
        self.status_raw_pub = self.create_publisher(String, '/mission/status/raw', 10)
        self.result_raw_pub = self.create_publisher(String, '/mission/result/raw', 10)

        # ---- Timers ----
        # Timer status berkala (Ti = 2-5 s)
        self.status_timer = self.create_timer(self.status_interval, self.on_status_timer)
        # Timer pengecekan inspeksi tray (resolusi 0.5 s)
        self.inspect_timer = self.create_timer(0.5, self.on_inspect_timer)

        self.get_logger().info(
            f'CollaborationAgentNode aktif [robot_id={self.robot_id}, gh_id={self.greenhouse_id}, '
            f'auto_inspect={self.auto_inspect}]. Interval status: {self.status_interval}s | '
            f'Interval inspeksi tray: {self.tray_interval}s'
        )

    # ------------------------------------------------------------------
    # Utilitas waktu (bisa di-override saat test)
    # ------------------------------------------------------------------
    def _now(self) -> float:
        return time.monotonic()

    # ------------------------------------------------------------------
    # Command Handling
    # ------------------------------------------------------------------
    def on_mission_command(self, msg):
        try:
            cmd = MissionCommand(
                m_id=msg.m_id,
                h_id=msg.h_id,
                cmd=msg.cmd,
                trays=[int(t) for t in msg.trays],
                crit_lvl=int(msg.crit_lvl),
                ts=int(msg.ts),
            )
            cmd.validate()
        except Exception as exc:
            self.get_logger().error(f'mission/command tidak valid: {exc}')
            return
        self._process_command(cmd)

    def on_mission_command_raw(self, msg: String):
        try:
            cmd = MissionCommand.from_json(msg.data)
        except Exception as exc:
            self.get_logger().error(f'Gagal parse mission/command raw JSON: {exc}')
            return
        self._process_command(cmd)

    def _process_command(self, cmd: MissionCommand):
        action = cmd.cmd.upper()
        key = (cmd.m_id, action, int(cmd.ts))
        if key in self._recent_commands:
            self.get_logger().info(
                f'Duplikat command diabaikan: {action} m_id={cmd.m_id} ts={cmd.ts}'
            )
            return
        self._recent_commands.append(key)

        self.get_logger().info(
            f'Collaboration Agent memproses perintah: {action} untuk misi {cmd.m_id}'
        )

        if action in (CMD_INSPECT, CMD_START):
            self._start_mission(cmd)
            return

        # Perintah kontrol hanya berlaku untuk misi yang sedang dijalankan
        if not self.mission_id or cmd.m_id != self.mission_id:
            self.get_logger().warn(
                f'Perintah {action} untuk misi {cmd.m_id} diabaikan '
                f'(misi aktif: {self.mission_id or "-"})'
            )
            return

        if action in (CMD_STOP, CMD_ABORT):
            if self.state in (STATUS_IN_PROGRESS, STATUS_PAUSED):
                self.state = STATUS_ABORTED
                self.tray_active = False
                self.pending_trays = []
                self.get_logger().warn(f'Misi {self.mission_id} DIBATALKAN.')
                self.publish_status()

        elif action == CMD_PAUSE:
            if self.state == STATUS_IN_PROGRESS:
                self.state = STATUS_PAUSED
                self.tray_active = False  # inspeksi tray diulang setelah resume
                self.get_logger().info(f'Misi {self.mission_id} DIJEDA.')
                self.publish_status()

        elif action == CMD_RESUME:
            if self.state == STATUS_PAUSED:
                self.state = STATUS_IN_PROGRESS
                self.get_logger().info(f'Misi {self.mission_id} DILANJUTKAN.')
                if self.auto_inspect and self.current_tray_id in self.pending_trays:
                    self._begin_tray(self.current_tray_id)
                self.publish_status()

        else:
            self.get_logger().warn(f'Perintah tidak dikenal: {action}')

    def _start_mission(self, cmd: MissionCommand):
        if self.state in (STATUS_IN_PROGRESS, STATUS_PAUSED) and cmd.m_id != self.mission_id:
            self.get_logger().warn(
                f'Misi {self.mission_id} masih aktif; diganti dengan misi baru {cmd.m_id}'
            )

        # Hilangkan duplikat tray dengan urutan tetap
        trays: List[int] = []
        for t in cmd.trays:
            if t not in trays:
                trays.append(int(t))

        self.mission_id = cmd.m_id
        self.greenhouse_id = cmd.h_id or self.greenhouse_id
        self.trays_to_inspect = trays
        self.pending_trays = list(trays)
        self.completed_trays = 0
        self.progress_pct = 0
        self.status_seq = 0
        self.result_seq = 0
        self.tray_active = False
        self.tray_samples = []
        self.current_tray_id = trays[0] if trays else 0

        if not trays:
            self.state = STATUS_COMPLETED
            self.progress_pct = 100
            self.get_logger().warn(f'Misi {self.mission_id} tidak berisi tray; langsung COMPLETED.')
            self.publish_status()
            return

        self.state = STATUS_IN_PROGRESS
        self.get_logger().info(
            f'Misi {self.mission_id} DIMULAI! Trays: {self.trays_to_inspect}, '
            f'mode: {"auto" if self.auto_inspect else "menunggu /mission/trigger_tray"}'
        )
        if self.auto_inspect:
            self._begin_tray(self.current_tray_id)
        # Segera publish status awal
        self.publish_status()

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

        status = MissionStatus(
            m_id=self.mission_id or 'NONE',
            r_id=self.robot_id,
            st=self.state,
            cur_tray=int(self.current_tray_id),
            pct=int(self.progress_pct),
            bat=int(min(max(self.battery_lvl, 0), 100)),
            seq=int(self.status_seq),
            ts=current_epoch_seconds(),
        )

        try:
            json_str = status.to_json()
        except Exception as exc:
            self.get_logger().error(f'Validasi status gagal: {exc}')
            return

        # 1. Publish raw JSON (viewer/debug)
        raw_msg = String()
        raw_msg.data = json_str
        self.status_raw_pub.publish(raw_msg)

        # 2. Publish strongly-typed ROS 2 message
        if self.status_pub is not None:
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
    def _begin_tray(self, tray_id: int):
        self.current_tray_id = tray_id
        self.tray_active = True
        self.tray_started_at = self._now()
        self.tray_samples = []
        self.get_logger().info(
            f'Mulai inspeksi tray {tray_id} (mengumpulkan deteksi {self.tray_interval:.0f}s)'
        )

    def on_inspect_timer(self):
        if self.state != STATUS_IN_PROGRESS or not self.tray_active:
            return
        elapsed = self._now() - self.tray_started_at
        can_finish = (elapsed >= self.tray_interval) or (
            self.min_samples_for_finish > 0 and len(self.tray_samples) >= self.min_samples_for_finish
        )
        if not can_finish:
            return
        if not self.tray_samples and elapsed < self.tray_interval + self.detection_grace:
            self.get_logger().warn(
                f'Belum ada deteksi visi untuk tray {self.current_tray_id}; menunggu...',
                throttle_duration_sec=5.0,
            )
            return
        self._finish_tray()

    def on_trigger_tray(self, msg: Int32):
        """Robot tiba di tray tertentu (dari sistem navigasi): mulai inspeksi tray itu."""
        if self.state != STATUS_IN_PROGRESS:
            self.get_logger().warn('Tidak ada misi aktif saat trigger_tray diterima')
            return
        tray_id = int(msg.data)
        if tray_id not in self.pending_trays:
            self.get_logger().warn(
                f'trigger_tray {tray_id} diabaikan: bukan tray yang tersisa pada misi '
                f'(sisa: {self.pending_trays})'
            )
            return
        self._begin_tray(tray_id)
        self.publish_status()

    def on_step_tray(self, _msg=None):
        """Paksa evaluasi tray saat ini sekarang juga."""
        if self.state != STATUS_IN_PROGRESS:
            self.get_logger().warn('Tidak ada misi aktif saat step_tray diterima')
            return
        if not self.tray_active:
            if self.current_tray_id not in self.pending_trays:
                return
            self._begin_tray(self.current_tray_id)
        self._finish_tray()

    def _finish_tray(self):
        tray_id = self.current_tray_id
        self.tray_active = False

        agg = aggregate_samples(self.tray_samples)
        n_samples = len(self.tray_samples)
        if agg is None:
            label, conf, anom = LABEL_UNKNOWN, 0.0, ANOMALY_DEFECT
            self.get_logger().warn(
                f'Tray {tray_id}: tidak ada deteksi visi -> dilaporkan {LABEL_UNKNOWN} '
                '(anom=1, perlu dicek manual)'
            )
        else:
            label, conf, anom = agg

        self.result_seq += 1
        result = MissionResult(
            m_id=self.mission_id,
            h_id=self.greenhouse_id,
            tray_id=int(tray_id),
            label=label,
            conf=conf,
            anom=anom,
            seq=self.result_seq,
            ts=current_epoch_seconds(),
        )

        try:
            json_str = result.to_json()
        except Exception as exc:
            self.get_logger().error(f'Validasi result gagal: {exc}')
            return

        raw_msg = String()
        raw_msg.data = json_str
        self.result_raw_pub.publish(raw_msg)

        if self.result_pub is not None:
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
            f'Hasil inspeksi tray {tray_id}: label={label}, conf={conf:.2f}, anom={anom}, '
            f'sampel={n_samples} ({len(json_str.encode("utf-8"))} byte <= 256 byte)'
        )

        # Update progres
        if tray_id in self.pending_trays:
            self.pending_trays.remove(tray_id)
        total = len(self.trays_to_inspect)
        self.completed_trays = total - len(self.pending_trays)
        self.progress_pct = int((self.completed_trays / total) * 100) if total else 100

        if self.pending_trays:
            self.current_tray_id = self.pending_trays[0]
            if self.auto_inspect:
                self._begin_tray(self.current_tray_id)
            self.publish_status()
        else:
            self.state = STATUS_COMPLETED
            self.progress_pct = 100
            self.get_logger().info(
                f'SELURUH {total} TRAY PADA MISI {self.mission_id} SELESAI DIINSPEKSI!'
            )
            self.publish_status()

    # ------------------------------------------------------------------
    # Sensor Subscriptions
    # ------------------------------------------------------------------
    def on_detections(self, msg):
        self.latest_detections = msg
        self.last_detection_time = self._now()
        if not self.tray_active or self.state != STATUS_IN_PROGRESS:
            return
        if msg.count == 0 or not msg.detections:
            return
        best = max(msg.detections, key=lambda d: d.confidence)
        label, anom = classify_label(best.class_name)
        self.tray_samples.append((label, float(best.confidence), anom))
        self.get_logger().info(
            f'Tray {self.current_tray_id}: sampel ke-{len(self.tray_samples)} deteksi visi '
            f'(label={label}, conf={float(best.confidence):.2f})',
            throttle_duration_sec=3.0,
        )

    def on_battery_state(self, msg):
        pct = float(msg.percentage)
        if math.isnan(pct):
            return  # baterai tidak diketahui (sesuai spesifikasi BatteryState)
        # Spesifikasi ROS: 0.0 - 1.0; sebagian driver mengirim 0 - 100
        if 0.0 <= pct <= 1.0:
            self.battery_lvl = int(round(pct * 100))
        elif 1.0 < pct <= 100.0:
            self.battery_lvl = int(round(pct))


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
