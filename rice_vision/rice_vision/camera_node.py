#!/usr/bin/env python3
"""Camera publisher node.

Membaca frame dari webcam/USB camera menggunakan OpenCV dan mem-publish
ke topic ROS 2 sebagai sensor_msgs/Image (encoding bgr8).

Terinspirasi arsitektur pada paper Rezakhani & Kim (2025): kamera robot
mem-publish ke topic "/camera", lalu modul computer vision men-subscribe
topic tersebut sebagai input model deteksi objek.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String

from rice_vision import image_codec


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # ---- Parameter ----
        self.declare_parameter('camera_id', 0)          # /dev/video0
        self.declare_parameter('frame_rate', 15.0)      # fps publikasi
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        # Simulasi tanpa webcam: path file video (mp4/avi) atau pola gambar
        # OpenCV (mis. /data/tray_%03d.jpg). Kosong = pakai camera_id.
        # Sumber file diputar ulang (loop) saat habis.
        self.declare_parameter('video_source', '')
        self.declare_parameter('dummy_fallback', True)
        self.declare_parameter('sync_with_scanner', True)

        camera_id = self.get_parameter('camera_id').value
        frame_rate = float(self.get_parameter('frame_rate').value)
        self.width = int(self.get_parameter('frame_width').value)
        self.height = int(self.get_parameter('frame_height').value)
        image_topic = self.get_parameter('image_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        video_source = str(self.get_parameter('video_source').value)
        dummy_fallback = bool(self.get_parameter('dummy_fallback').value)
        self.sync_with_scanner = bool(self.get_parameter('sync_with_scanner').value)

        self.loop_source = bool(video_source)
        source = video_source if video_source else camera_id

        # Normalisasi source jika numerik (mis. string "0" -> integer 0)
        if isinstance(source, str) and source.isdigit():
            source = int(source)

        self.use_dummy = False
        self.cap = None
        self.grass_bed = None
        self.standby_frame = None
        self.frame_idx = 0
        self.scan_phase = 'UPRIGHT'

        # Sinkronisasi sudut pandang kamera dengan fase gerakan lengan robot
        self.create_subscription(String, '/scan_phase', self._on_scan_phase, 10)

        if str(source).lower() == 'dummy':
            self.use_dummy = True
            self.get_logger().info('Mode kamera: DUMMY (gambar sintetis bibit padi)')
        else:
            try:
                # Buka kamera fisik via V4L2 backend (NVIDIA Jetson / Linux)
                if isinstance(source, int) or (isinstance(source, str) and (source.startswith('/dev/video') or source.isdigit())):
                    cam_idx = int(source.replace('/dev/video', '')) if isinstance(source, str) else source
                    self.cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
                    if not self.cap.isOpened():
                        self.cap = cv2.VideoCapture(cam_idx)
                else:
                    self.cap = cv2.VideoCapture(source)

                if not self.cap.isOpened():
                    if dummy_fallback:
                        self.get_logger().warn(
                            f'Tidak bisa membuka sumber kamera "{source}". '
                            'Menggunakan mode dummy camera (gambar bibit padi sintetis).'
                        )
                        self.use_dummy = True
                        if self.cap is not None:
                            self.cap.release()
                        self.cap = None
                    else:
                        raise RuntimeError(f'Cannot open camera {source}')
                else:
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            except Exception as exc:
                if dummy_fallback:
                    self.get_logger().warn(
                        f'Error membuka kamera "{source}": {exc}. '
                        'Menggunakan mode dummy camera.'
                    )
                    self.use_dummy = True
                    self.cap = None
                else:
                    raise

        self.publisher = self.create_publisher(
            Image, image_topic, qos_profile_sensor_data)

        period = 1.0 / max(frame_rate, 1.0)
        self.timer = self.create_timer(period, self.capture_and_publish)

        active_info = 'DUMMY' if self.use_dummy else f'kamera={source}'
        self.get_logger().info(
            f'Camera node aktif: {active_info}, {self.width}x{self.height} '
            f'@{frame_rate}fps -> topic "{image_topic}"')

    def _on_scan_phase(self, msg: String):
        self.scan_phase = msg.data.strip()

    def _create_standby_frame(self, w: int, h: int) -> np.ndarray:
        """Membuat frame pandangan kamera saat robot tegak lurus (mengarah ke langit-langit)."""
        frame = np.full((h, w, 3), (225, 232, 228), dtype=np.uint8)

        # Rangka atap / truss greenhouse
        cv2.line(frame, (0, int(h * 0.17)), (w, int(h * 0.21)), (170, 175, 172), 3, cv2.LINE_AA)
        cv2.line(frame, (0, int(h * 0.50)), (w, int(h * 0.56)), (160, 165, 162), 4, cv2.LINE_AA)
        cv2.line(frame, (int(w * 0.28), 0), (int(w * 0.34), h), (175, 180, 178), 2, cv2.LINE_AA)
        cv2.line(frame, (int(w * 0.72), 0), (int(w * 0.66), h), (175, 180, 178), 2, cv2.LINE_AA)

        # Noise sensor ambient
        rng = np.random.default_rng(101)
        noise = rng.normal(0, 2, frame.shape).astype(np.int16)
        frame = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        # HUD Panel Status Kamera
        banner_h = 72
        cv2.rectangle(frame, (10, 10), (w - 10, 10 + banner_h), (35, 38, 45), -1)
        cv2.rectangle(frame, (10, 10), (w - 10, 10 + banner_h), (70, 78, 90), 2)

        cv2.putText(frame, '[KAMERA UJUNG LENGAN] STANDBY / UPRIGHT', (24, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'Arah: Menghadap ke Atas (Langit-langit Greenhouse)', (24, 56),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(frame, 'Menunggu perintah misi MQTT (cmd: INSPECT) untuk memindai baki...', (24, 74),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (140, 255, 140), 1, cv2.LINE_AA)

        return frame

    def _create_seedling_bed(self, w: int, h: int) -> np.ndarray:
        """Membuat kanvas bibit padi (rerumputan) realistis dengan tanah organik dan helai daun bertingkat."""
        rng = np.random.default_rng(2026)
        soil = np.zeros((h, w, 3), dtype=np.uint8)
        soil[:, :] = (24, 38, 28)  # BGR coklat tanah organik gelap
        noise = rng.integers(-18, 18, (h, w, 3), dtype=np.int16)
        soil = np.clip(soil.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        # Butiran media tanam / perlit
        for _ in range(800):
            gx = int(rng.integers(0, w))
            gy = int(rng.integers(0, h))
            gr = int(rng.integers(1, 3))
            gc = int(rng.integers(15, 60))
            cv2.circle(soil, (gx, gy), gr, (gc, gc + 5, gc + 10), -1)

        canvas = soil.copy()
        # Rerumputan / bilah bibit padi hijau bertingkat
        layers = [
            (900, 40, 110, (18, 110, 30), 1),    # Layer belakang (hijau tua)
            (1000, 50, 140, (25, 160, 45), 2),   # Layer tengah (hijau subur)
            (800, 60, 160, (35, 205, 55), 2),    # Layer depan (hijau cerah)
            (400, 40, 120, (50, 230, 95), 1),    # Pucuk tunas muda segar
        ]

        for count, h_min, h_max, (b, g, r), thick in layers:
            for _ in range(count):
                bx = int(rng.integers(-20, w + 20))
                by = int(rng.integers(h // 5, h + 40))
                bh = int(rng.integers(h_min, h_max))
                bend = int(rng.integers(-30, 30))

                p0 = (bx, by)
                p1 = (bx + bend // 2, by - bh // 2)
                p2 = (bx + bend, by - bh)

                cb = int(np.clip(b + rng.integers(-12, 12), 10, 80))
                cg = int(np.clip(g + rng.integers(-20, 25), 90, 255))
                cr = int(np.clip(r + rng.integers(-15, 20), 20, 120))

                pts = np.array([p0, p1, p2], dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(canvas, [pts], isClosed=False, color=(cb, cg, cr),
                              thickness=thick, lineType=cv2.LINE_AA)

        return canvas

    def _generate_dummy_frame(self) -> np.ndarray:
        """Membuat frame dinamis bibit padi saat memindai, atau frame standby saat tegak."""
        if self.sync_with_scanner and self.scan_phase != 'FORWARD_SCANNING':
            if self.standby_frame is None:
                self.standby_frame = self._create_standby_frame(self.width, self.height)
            return self.standby_frame.copy()

        if self.grass_bed is None:
            self.grass_bed = self._create_seedling_bed(self.width + 160, self.height + 120)

        self.frame_idx += 1
        t = self.frame_idx * 0.08
        max_xo = max(0, self.grass_bed.shape[1] - self.width)
        max_yo = max(0, self.grass_bed.shape[0] - self.height)

        # Panning halus mensimulasikan pergerakan kamera di atas hamparan baki semai
        xo = int((max_xo / 2) + (max_xo / 2 - 10) * np.sin(t * 0.6)) if max_xo > 0 else 0
        yo = int((max_yo / 2) + (max_yo / 2 - 10) * np.cos(t * 0.4)) if max_yo > 0 else 0

        crop = self.grass_bed[yo:yo + self.height, xo:xo + self.width].copy()

        # Tambahkan sedikit sensor noise (tekstur autentik sensor CMOS)
        rng = np.random.default_rng()
        noise = rng.normal(0, 2.5, crop.shape).astype(np.int16)
        return np.clip(crop.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    def capture_and_publish(self):
        if self.use_dummy:
            frame = self._generate_dummy_frame()
        else:
            ret, frame = self.cap.read()
            if not ret and self.loop_source:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Gagal membaca frame dari kamera.',
                                       throttle_duration_sec=5.0)
                return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        msg = image_codec.cv2_to_imgmsg(frame, header=header)
        self.publisher.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
