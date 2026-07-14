#!/usr/bin/env python3
"""Keras/TensorFlow (.h5) image classification node.

Untuk model CNN klasifikasi hasil training Keras -- seperti model bibit padi
biner (Bagus / Tidak Bagus) yang dilatih dengan:

    ImageDataGenerator(rescale=1./255)
    target_size=(224, 224), class_mode='binary'
    output: Dense(1, activation='sigmoid')

Node ini men-subscribe frame kamera, melakukan preprocessing PERSIS seperti
saat training (resize 224x224, konversi BGR->RGB, normalisasi /255), lalu
mem-publish hasil klasifikasi ke topic yang SAMA dengan detector_node:

  1. /rice_vision/detections       -> DetectionArray berisi 1 "deteksi"
     yang mencakup seluruh frame (klasifikasi tidak punya bounding box,
     jadi bbox = seluruh frame). Dengan begitu viewer_node dan node
     subscriber lain tetap bekerja tanpa perubahan.
  2. /rice_vision/annotated_image  -> frame dengan label + border berwarna
     (hijau = kelas 0 / Bagus, merah = kelas 1 / Tidak Bagus).

Catatan urutan kelas: flow_from_directory memberi indeks kelas berdasarkan
urutan alfabet nama folder. Pada dataset kamu:
  indeks 0 = "Bibit Padi (Bagus)"        -> sigmoid < threshold
  indeks 1 = "Bibit padi (tidak bagus)"  -> sigmoid > threshold
(sesuai kode inferensi di script training: prediction > 0.5 -> "Tidak Bagus")
Urutan ini bisa diubah lewat parameter `class_names`.
"""

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rice_vision import image_codec
from rice_vision_interfaces.msg import Detection, DetectionArray


class ClassifierNode(Node):
    def __init__(self):
        super().__init__('classifier_node')

        # ---- Parameter ----
        self.declare_parameter('model_path', '')          # WAJIB diisi (.h5)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('results_topic', '/rice_vision/detections')
        self.declare_parameter('annotated_topic', '/rice_vision/annotated_image')
        # Urutan HARUS sama dengan indeks kelas flow_from_directory
        # (alfabetis nama folder dataset).
        self.declare_parameter('class_names',
                               ['Bibit Padi (Bagus)',
                                'Bibit Padi (Tidak Bagus)'])
        self.declare_parameter('input_width', 224)
        self.declare_parameter('input_height', 224)
        self.declare_parameter('threshold', 0.5)          # utk output sigmoid
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('max_classification_rate', 10.0)
        # TensorFlow default dipaksa CPU agar tidak bentrok CUDA dengan
        # torch/YOLO (TF 2.13 butuh CUDA 11.8, tidak cocok dgn CUDA 12.x).
        self.declare_parameter('use_gpu', False)

        model_path = str(self.get_parameter('model_path').value)
        image_topic = self.get_parameter('image_topic').value
        results_topic = self.get_parameter('results_topic').value
        annotated_topic = self.get_parameter('annotated_topic').value
        self.class_names = list(self.get_parameter('class_names').value)
        self.in_w = int(self.get_parameter('input_width').value)
        self.in_h = int(self.get_parameter('input_height').value)
        self.threshold = float(self.get_parameter('threshold').value)
        self.publish_annotated = self.get_parameter('publish_annotated').value
        max_rate = float(self.get_parameter('max_classification_rate').value)
        use_gpu = bool(self.get_parameter('use_gpu').value)
        self.min_interval = (1.0 / max_rate) if max_rate > 0 else 0.0
        self.last_inference_time = 0.0

        if not model_path:
            self.get_logger().fatal(
                'Parameter "model_path" kosong. Jalankan dengan:\n'
                '  ros2 launch rice_vision classification.launch.py '
                'model_path:=/path/ke/model_bibit_padi.h5')
            raise RuntimeError('model_path belum diisi')

        if not use_gpu:
            # Sembunyikan GPU dari TensorFlow SEBELUM import tf.
            os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
        os.environ.setdefault('TF_CPP_MIN_LOG_LEVEL', '2')  # kurangi log tf

        # ---- Muat model Keras ----
        try:
            import tensorflow as tf  # noqa: F401
            from tensorflow.keras.models import load_model
        except ImportError:
            self.get_logger().fatal(
                'Package "tensorflow" belum terpasang di venv. Jalankan:\n'
                '  pip install tensorflow==2.13.1\n'
                '(2.13 adalah versi terakhir yang mendukung Python 3.8/Foxy)')
            raise

        model_path = os.path.expanduser(model_path)
        self.get_logger().info(
            f'Memuat model Keras (.h5): {model_path} '
            f'({"GPU" if use_gpu else "CPU"}) ...')
        self.model = load_model(model_path, compile=False)

        # Deteksi bentuk output: 1 neuron sigmoid (biner) atau N neuron
        # softmax (multi-kelas).
        out_shape = self.model.output_shape
        self.n_outputs = int(out_shape[-1]) if out_shape[-1] else 1
        self.get_logger().info(
            f'Model siap. Input {self.model.input_shape} -> output '
            f'{out_shape} ({"sigmoid biner" if self.n_outputs == 1 else "softmax " + str(self.n_outputs) + " kelas"}). '
            f'Kelas: {self.class_names}')

        # Warmup sekali agar inferensi pertama tidak lambat
        dummy = np.zeros((1, self.in_h, self.in_w, 3), dtype=np.float32)
        self.model.predict(dummy, verbose=0)

        # ---- ROS I/O ----
        self.results_pub = self.create_publisher(
            DetectionArray, results_topic, 10)
        self.annotated_pub = self.create_publisher(
            Image, annotated_topic, qos_profile_sensor_data)
        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'Classifier aktif: subscribe "{image_topic}" -> '
            f'publish "{results_topic}" & "{annotated_topic}"')

    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        now = time.monotonic()
        if self.min_interval > 0 and (now - self.last_inference_time) < self.min_interval:
            return
        self.last_inference_time = now

        frame = image_codec.imgmsg_to_cv2(msg)
        h, w = frame.shape[:2]

        # ---- Preprocessing: PERSIS seperti training ----
        # Training pakai keras load_img (RGB) + resize target_size + /255.
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (self.in_w, self.in_h),
                             interpolation=cv2.INTER_AREA)
        batch = np.expand_dims(resized.astype(np.float32) / 255.0, axis=0)

        # ---- Inferensi ----
        t0 = time.monotonic()
        pred = self.model.predict(batch, verbose=0)
        inference_ms = (time.monotonic() - t0) * 1000.0

        # ---- Interpretasi output ----
        if self.n_outputs == 1:
            # Sigmoid biner: p = probabilitas kelas indeks 1
            p = float(pred[0][0])
            class_id = 1 if p > self.threshold else 0
            confidence = p if class_id == 1 else 1.0 - p
        else:
            # Softmax multi-kelas
            class_id = int(np.argmax(pred[0]))
            confidence = float(pred[0][class_id])

        class_name = (self.class_names[class_id]
                      if class_id < len(self.class_names)
                      else f'kelas_{class_id}')

        # ---- Publish hasil (format DetectionArray, bbox = seluruh frame) ----
        det = Detection()
        det.class_id = class_id
        det.class_name = class_name
        det.confidence = confidence
        det.x_min, det.y_min = 0, 0
        det.x_max, det.y_max = w, h

        result = DetectionArray()
        result.header = msg.header
        result.detections.append(det)
        result.count = 1
        result.inference_time_ms = float(inference_ms)
        self.results_pub.publish(result)

        # ---- Frame beranotasi: border + label ----
        if self.publish_annotated:
            annotated = frame.copy()
            # hijau untuk kelas 0 (Bagus), merah untuk kelas lain
            color = (0, 200, 0) if class_id == 0 else (0, 0, 220)
            cv2.rectangle(annotated, (0, 0), (w - 1, h - 1), color, 6)
            label = f'{class_name}  {confidence:.2f}'
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
            cv2.rectangle(annotated, (10, 10), (18 + tw, 24 + th), color, -1)
            cv2.putText(annotated, label, (14, 16 + th),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (255, 255, 255), 2, cv2.LINE_AA)
            annotated_msg = image_codec.cv2_to_imgmsg(
                annotated, header=msg.header)
            self.annotated_pub.publish(annotated_msg)

        self.get_logger().info(
            f'{class_name} ({confidence:.2f}) [{inference_ms:.1f} ms]',
            throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
