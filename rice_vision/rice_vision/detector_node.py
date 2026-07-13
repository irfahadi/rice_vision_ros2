#!/usr/bin/env python3
"""YOLOv8 object detection node.

Men-subscribe topic gambar dari kamera, menjalankan inferensi model YOLOv8
custom (misalnya model deteksi kualitas bibit padi), lalu mem-publish:

  1. /rice_vision/detections        -> rice_vision_interfaces/DetectionArray
     (data terstruktur: kelas, confidence, bounding box, jumlah objek)
  2. /rice_vision/annotated_image   -> sensor_msgs/Image
     (frame dengan bounding box tergambar, untuk visualisasi rqt/rviz)

Mengikuti pola pipeline pada paper Rezakhani & Kim (2025): ROS sebagai
middleware, node CV memproses topic "/camera" secara real-time dengan
YOLOv8, dan hasilnya dibagikan kembali lewat jaringan/topic ROS.
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rice_vision_interfaces.msg import Detection, DetectionArray


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        # ---- Parameter ----
        # Path ke bobot model custom kamu (.pt hasil training YOLOv8).
        # Default 'yolov8n.pt' akan otomatis diunduh Ultralytics --
        # berguna untuk menguji pipeline sebelum model custom siap.
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/rice_vision/detections')
        self.declare_parameter('annotated_topic', '/rice_vision/annotated_image')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'cpu')      # 'cpu', 'cuda:0', dst.
        self.declare_parameter('publish_annotated', True)
        # Proses maksimal N frame/detik agar CPU tidak kewalahan (0 = tanpa batas)
        self.declare_parameter('max_detection_rate', 10.0)

        model_path = self.get_parameter('model_path').value
        image_topic = self.get_parameter('image_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        annotated_topic = self.get_parameter('annotated_topic').value
        self.conf = float(self.get_parameter('confidence_threshold').value)
        self.iou = float(self.get_parameter('iou_threshold').value)
        self.device = self.get_parameter('device').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        max_rate = float(self.get_parameter('max_detection_rate').value)
        self.min_interval = (1.0 / max_rate) if max_rate > 0 else 0.0
        self.last_inference_time = 0.0

        # ---- Muat model YOLOv8 ----
        # Import di sini agar error dependency muncul dengan pesan jelas.
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().fatal(
                'Package "ultralytics" belum terpasang. '
                'Jalankan: pip3 install -r requirements.txt')
            raise

        model_path = os.path.expanduser(model_path)
        self.get_logger().info(f'Memuat model YOLOv8: {model_path} '
                               f'(device={self.device}) ...')
        self.model = YOLO(model_path)
        self.class_names = self.model.names
        self.get_logger().info(
            f'Model siap. Kelas: {self.class_names}')

        # ---- ROS I/O ----
        self.bridge = CvBridge()
        self.detections_pub = self.create_publisher(
            DetectionArray, detections_topic, 10)
        self.annotated_pub = self.create_publisher(
            Image, annotated_topic, qos_profile_sensor_data)
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback,
            qos_profile_sensor_data)

        self.get_logger().info(
            f'Detector aktif: subscribe "{image_topic}" -> '
            f'publish "{detections_topic}" & "{annotated_topic}"')

    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        # Batasi laju inferensi (frame yang datang terlalu cepat dilewati)
        now = time.monotonic()
        if self.min_interval > 0 and (now - self.last_inference_time) < self.min_interval:
            return
        self.last_inference_time = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        t0 = time.monotonic()
        results = self.model.predict(
            source=frame,
            conf=self.conf,
            iou=self.iou,
            device=self.device,
            verbose=False,
        )
        inference_ms = (time.monotonic() - t0) * 1000.0

        result = results[0]

        # ---- Susun pesan DetectionArray ----
        det_array = DetectionArray()
        det_array.header = msg.header
        det_array.inference_time_ms = float(inference_ms)

        boxes = result.boxes
        if boxes is not None:
            for box in boxes:
                det = Detection()
                class_id = int(box.cls[0])
                det.class_id = class_id
                det.class_name = str(self.class_names.get(class_id, class_id)) \
                    if isinstance(self.class_names, dict) \
                    else str(self.class_names[class_id])
                det.confidence = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                det.x_min = int(x1)
                det.y_min = int(y1)
                det.x_max = int(x2)
                det.y_max = int(y2)
                det_array.detections.append(det)

        det_array.count = len(det_array.detections)
        self.detections_pub.publish(det_array)

        # ---- Publish frame beranotasi (untuk rqt_image_view / rviz2) ----
        if self.publish_annotated:
            annotated = result.plot()  # BGR ndarray dengan bbox + label
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)

        if det_array.count > 0:
            self.get_logger().info(
                f'{det_array.count} objek terdeteksi '
                f'({inference_ms:.1f} ms)',
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
