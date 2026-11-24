#!/usr/bin/env python3
"""Viewer node - interface tampilan hasil deteksi.

Menampilkan feed kamera beranotasi (dari detector_node) di sebuah jendela
OpenCV, lengkap dengan panel informasi: jumlah objek terdeteksi, FPS,
waktu inferensi model, dan rincian tiap objek (kelas + confidence).

Ini "interface" mandiri seperti Camera Panel pada paper Rezakhani & Kim
(2025) -- tanpa perlu rqt/rviz. Tekan 'q' pada jendela untuk keluar.

Alur data:
  detector_node  ->  /rice_vision/annotated_image  (sensor_msgs/Image)
                 ->  /rice_vision/detections        (DetectionArray)
                                     |
                                     v
                                viewer_node  ->  jendela OpenCV
"""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from rice_vision import image_codec
from rice_vision_interfaces.msg import DetectionArray


class ViewerNode(Node):
    def __init__(self):
        super().__init__('viewer_node')

        # ---- Parameter ----
        self.declare_parameter('annotated_topic', '/rice_vision/annotated_image')
        self.declare_parameter('detections_topic', '/rice_vision/detections')
        self.declare_parameter('window_name', 'Rice Vision - Deteksi Objek')
        # Lebar panel info (px) di sisi kanan jendela
        self.declare_parameter('panel_width', 320)
        # Jika True, tampilkan juga FPS tampilan (bukan hanya FPS inferensi)
        self.declare_parameter('show_display_fps', True)

        annotated_topic = self.get_parameter('annotated_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        self.window_name = self.get_parameter('window_name').value
        self.panel_width = int(self.get_parameter('panel_width').value)
        self.show_display_fps = self.get_parameter('show_display_fps').value

        # ---- State ----
        self.latest_frame = None          # frame beranotasi terakhir (BGR)
        self.latest_detections = None     # DetectionArray terakhir
        self.latest_mission_status = None # status misi terakhir
        self._last_display_time = time.monotonic()
        self._display_fps = 0.0

        # ---- Subscriptions ----
        self.create_subscription(
            Image, annotated_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/image_raw', self.on_raw_image, qos_profile_sensor_data)
        self.create_subscription(
            DetectionArray, detections_topic, self.on_detections, 10)
        self.create_subscription(
            String, '/mission/status/raw', self.on_mission_status_raw, 10)

        # ---- Jendela tampilan ----
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # Timer untuk merefresh jendela (30 Hz) -- terpisah dari laju kamera
        self.create_timer(1.0 / 30.0, self.render)

        self.get_logger().info(
            f'Viewer aktif. Menampilkan "{annotated_topic}". '
            "Tekan 'q' pada jendela untuk keluar.")

    # ------------------------------------------------------------------
    def on_image(self, msg: Image):
        try:
            self.latest_frame = image_codec.imgmsg_to_cv2(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Gagal decode gambar: {exc}',
                                   throttle_duration_sec=5.0)

    def on_raw_image(self, msg: Image):
        if self.latest_frame is None:
            try:
                self.latest_frame = image_codec.imgmsg_to_cv2(msg)
            except Exception:
                pass

    def on_detections(self, msg: DetectionArray):
        self.latest_detections = msg

    def on_mission_status_raw(self, msg: String):
        try:
            self.latest_mission_status = json.loads(msg.data)
        except Exception:
            pass

    # ------------------------------------------------------------------
    def render(self):
        if self.latest_frame is None:
            # Belum ada frame -- tampilkan layar tunggu
            placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(placeholder, 'Menunggu frame dari detector...',
                        (40, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (200, 200, 200), 2, cv2.LINE_AA)
            self._show(placeholder)
            return

        frame = self.latest_frame.copy()

        # Hitung FPS tampilan
        now = time.monotonic()
        dt = now - self._last_display_time
        self._last_display_time = now
        if dt > 0:
            # smoothing sederhana
            self._display_fps = 0.9 * self._display_fps + 0.1 * (1.0 / dt)

        canvas = self._compose(frame)
        self._show(canvas)

    # ------------------------------------------------------------------
    def _compose(self, frame):
        """Gabungkan frame video dengan panel info di sisi kanan."""
        h, w = frame.shape[:2]
        panel = np.full((h, self.panel_width, 3), 30, dtype=np.uint8)

        x = 16
        y = 40
        line = 28

        def put(text, color=(230, 230, 230), scale=0.6, thick=1):
            nonlocal y
            cv2.putText(panel, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                        scale, color, thick, cv2.LINE_AA)
            y += line

        # Judul
        put('RICE VISION', (0, 200, 255), 0.8, 2)
        y += 6

        det = self.latest_detections
        count = det.count if det is not None else 0
        infer_ms = det.inference_time_ms if det is not None else 0.0

        # Sorot jumlah objek
        put(f'Objek terdeteksi: {count}', (0, 255, 120), 0.7, 2)
        y += 4
        put(f'Inferensi : {infer_ms:5.1f} ms', (200, 200, 200), 0.55, 1)
        if self.show_display_fps:
            put(f'Display   : {self._display_fps:5.1f} FPS',
                (200, 200, 200), 0.55, 1)
        y += 8

        # Informasi Misi & Sink Node RW612
        if self.latest_mission_status is not None:
            cv2.line(panel, (x, y), (self.panel_width - x, y), (80, 80, 80), 1)
            y += line
            put('Misi Sink RW612:', (0, 200, 255), 0.6, 1)
            m_id = self.latest_mission_status.get('m_id', '-')
            st = self.latest_mission_status.get('st', '-')
            cur_t = self.latest_mission_status.get('cur_tray', 0)
            pct = self.latest_mission_status.get('pct', 0)
            bat = self.latest_mission_status.get('bat', 0)
            put(f'ID   : {m_id}', (220, 220, 220), 0.5, 1)
            st_color = (0, 255, 0) if st == 'IN_PROGRESS' else ((255, 200, 0) if st == 'COMPLETED' else (200, 200, 200))
            put(f'Stat : {st}', st_color, 0.5, 1)
            put(f'Tray : {cur_t} ({pct}%)', (220, 220, 220), 0.5, 1)
            put(f'Bat  : {bat}%', (220, 220, 220), 0.5, 1)
            y += 4

        # Garis pemisah
        cv2.line(panel, (x, y), (self.panel_width - x, y), (80, 80, 80), 1)
        y += line

        put('Rincian:', (180, 180, 255), 0.6, 1)

        if det is not None and det.count > 0:
            # Batasi baris agar tidak melebihi panel
            max_rows = max(0, (frame.shape[0] - y - 16) // line)
            for i, d in enumerate(det.detections[:max_rows]):
                put(f'- {d.class_name} ({d.confidence:.2f})',
                    (220, 220, 220), 0.5, 1)
            sisa = det.count - max_rows
            if sisa > 0:
                put(f'  (+{sisa} lainnya...)', (150, 150, 150), 0.5, 1)
        else:
            put('- (tidak ada)', (150, 150, 150), 0.5, 1)

        # Footer
        cv2.putText(panel, "Tekan 'q' untuk keluar",
                    (x, frame.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (120, 120, 120), 1, cv2.LINE_AA)

        return np.hstack([frame, panel])

    # ------------------------------------------------------------------
    def _show(self, image):
        cv2.imshow(self.window_name, image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("Tombol 'q' ditekan, menutup viewer.")
            raise KeyboardInterrupt

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
