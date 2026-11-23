#!/usr/bin/env python3
"""ONNX Runtime (.onnx) image classification node -- CPU/laptop/simulasi.

Kembaran classifier_trt_node tanpa TensorRT/pycuda: model .onnx yang sama
(rice_vision/models/models_bibit_padi.onnx) dijalankan dengan onnxruntime,
sehingga pipeline misi bisa diuji di laptop atau PC tanpa GPU NVIDIA sebelum
dipasang di Jetson.

Preprocessing & interpretasi output PERSIS sama dengan classifier_trt_node:
  RGB -> resize (INTER_AREA) ke ukuran input model -> /255 -> NHWC/NCHW otomatis
  sigmoid > threshold -> kelas indeks 1, selain itu indeks 0 (softmax -> argmax)

Topic output sama (/rice_vision/detections & /rice_vision/annotated_image),
jadi collaboration_agent_node dan viewer_node tidak perlu diubah.

Butuh: pip install onnxruntime   (atau onnxruntime-gpu)
"""

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from rice_vision import image_codec
from rice_vision_interfaces.msg import Detection, DetectionArray


def preprocess(frame_bgr, in_w, in_h, layout):
    """Preprocessing identik classifier_trt_node (dipakai juga oleh test)."""
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (in_w, in_h), interpolation=cv2.INTER_AREA)
    norm = resized.astype(np.float32) / 255.0
    if layout == 'nchw':
        norm = np.transpose(norm, (2, 0, 1))
    return np.ascontiguousarray(np.expand_dims(norm, axis=0))


def interpret(output, threshold):
    """Return (class_id, confidence) dari output model."""
    flat = np.asarray(output).ravel()
    if flat.size == 1:
        p = float(flat[0])
        class_id = 1 if p > threshold else 0
        confidence = p if class_id == 1 else 1.0 - p
    else:
        class_id = int(np.argmax(flat))
        confidence = float(flat[class_id])
    return class_id, min(max(confidence, 0.0), 1.0)


def resolve_layout(shape, layout, in_w, in_h):
    """Tentukan (layout, w, h) dari shape input ONNX (dim dinamis = None/str)."""
    dims = [d if isinstance(d, int) and d > 0 else 0 for d in shape]
    if layout == 'auto':
        if len(dims) == 4 and dims[1] == 3:
            layout = 'nchw'
        else:
            layout = 'nhwc'
    if len(dims) == 4:
        h, w = (dims[2], dims[3]) if layout == 'nchw' else (dims[1], dims[2])
    else:
        h = w = 0
    w = in_w if in_w > 0 else (w or 224)
    h = in_h if in_h > 0 else (h or 224)
    return layout, w, h


class ClassifierONNXNode(Node):
    def __init__(self):
        super().__init__('classifier_onnx_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('results_topic', '/rice_vision/detections')
        self.declare_parameter('annotated_topic', '/rice_vision/annotated_image')
        self.declare_parameter('class_names',
                               ['Bibit Padi (Bagus)',
                                'Bibit Padi (Tidak Bagus)'])
        self.declare_parameter('input_width', 0)
        self.declare_parameter('input_height', 0)
        self.declare_parameter('layout', 'auto')
        self.declare_parameter('threshold', 0.5)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('max_classification_rate', 10.0)
        self.declare_parameter('sync_with_scanner', True)

        model_path = os.path.expanduser(str(self.get_parameter('model_path').value))
        image_topic = self.get_parameter('image_topic').value
        results_topic = self.get_parameter('results_topic').value
        annotated_topic = self.get_parameter('annotated_topic').value
        self.class_names = list(self.get_parameter('class_names').value)
        in_w = int(self.get_parameter('input_width').value)
        in_h = int(self.get_parameter('input_height').value)
        layout = str(self.get_parameter('layout').value).lower()
        self.threshold = float(self.get_parameter('threshold').value)
        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)
        self.sync_with_scanner = bool(self.get_parameter('sync_with_scanner').value)
        self.scan_phase = 'UPRIGHT'
        max_rate = float(self.get_parameter('max_classification_rate').value)
        self.min_interval = (1.0 / max_rate) if max_rate > 0 else 0.0
        self.last_inference_time = 0.0

        if not model_path or not os.path.isfile(model_path):
            candidates = [
                model_path,
                os.path.expanduser('~/ros2_ws/src/rice_vision_ros2/rice_vision/models/models_bibit_padi.onnx'),
                os.path.expanduser('~/ros2_ws/install/rice_vision/share/rice_vision/models/models_bibit_padi.onnx'),
                '/home/jetson/ros2_ws/src/rice_vision_ros2/rice_vision/models/models_bibit_padi.onnx',
                '/home/tesis/ros2_ws/src/rice_vision_ros2/rice_vision/models/models_bibit_padi.onnx',
            ]
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg = get_package_share_directory('rice_vision')
                candidates.insert(0, os.path.join(pkg, 'models', 'models_bibit_padi.onnx'))
            except Exception:
                pass
            for c in candidates:
                if c and os.path.isfile(c):
                    model_path = c
                    break

        if not model_path or not os.path.isfile(model_path):
            self.get_logger().fatal(
                f'model_path tidak valid: "{model_path}". Contoh:\n'
                '  model_path:=.../rice_vision/models/models_bibit_padi.onnx')
            raise RuntimeError('model_path belum diisi / tidak ditemukan')

        try:
            import onnxruntime as ort
        except ImportError:
            self.get_logger().fatal('onnxruntime belum terpasang: pip install onnxruntime')
            raise

        # GPU (CUDA) bila onnxruntime-gpu terpasang, selain itu CPU
        available = ort.get_available_providers()
        providers = [p for p in ('CUDAExecutionProvider', 'CPUExecutionProvider') if p in available]
        self.session = ort.InferenceSession(model_path, providers=providers or available)
        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        self.layout, self.in_w, self.in_h = resolve_layout(list(inp.shape), layout, in_w, in_h)

        self.results_pub = self.create_publisher(DetectionArray, results_topic, 10)
        self.annotated_pub = self.create_publisher(Image, annotated_topic, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/scan_phase', self._on_scan_phase, 10)

        self.get_logger().info(
            f'Classifier ONNX Runtime aktif: {os.path.basename(model_path)} input '
            f'{self.in_w}x{self.in_h} ({self.layout.upper()}), provider '
            f'{self.session.get_providers()[0]}. "{image_topic}" -> "{results_topic}"')

    def _on_scan_phase(self, msg: String):
        self.scan_phase = msg.data.strip()

    def image_callback(self, msg: Image):
        # Jika sinkronisasi scanner aktif dan robot tidak sedang memindai baki (FORWARD_SCANNING):
        # Lewatkan frame langsung ke annotated_image (agar RViz menampilkan kamera standby),
        # tanpa menjalankan inferensi / mem-publish deteksi bibit palsu.
        if self.sync_with_scanner and self.scan_phase != 'FORWARD_SCANNING':
            if self.publish_annotated:
                self.annotated_pub.publish(msg)
            return

        now = time.monotonic()
        if self.min_interval > 0 and (now - self.last_inference_time) < self.min_interval:
            return
        self.last_inference_time = now

        frame = image_codec.imgmsg_to_cv2(msg)
        h, w = frame.shape[:2]
        batch = preprocess(frame, self.in_w, self.in_h, self.layout)

        t0 = time.monotonic()
        out = self.session.run(None, {self.input_name: batch})[0]
        inference_ms = (time.monotonic() - t0) * 1000.0

        class_id, confidence = interpret(out, self.threshold)
        class_name = (self.class_names[class_id]
                      if class_id < len(self.class_names) else f'kelas_{class_id}')

        det = Detection()
        det.class_id = class_id
        det.class_name = class_name
        det.confidence = float(confidence)
        det.x_min, det.y_min = 0, 0
        det.x_max, det.y_max = w, h

        result = DetectionArray()
        result.header = msg.header
        result.detections.append(det)
        result.count = 1
        result.inference_time_ms = float(inference_ms)
        self.results_pub.publish(result)

        if self.publish_annotated:
            annotated = frame.copy()
            color = (0, 200, 0) if class_id == 0 else (0, 0, 220)
            cv2.rectangle(annotated, (0, 0), (w - 1, h - 1), color, 6)
            label = f'{class_name}  {confidence:.2f}  [ONNX]'
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
            cv2.rectangle(annotated, (10, 10), (18 + tw, 24 + th), color, -1)
            cv2.putText(annotated, label, (14, 16 + th), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 255, 255), 2, cv2.LINE_AA)
            self.annotated_pub.publish(image_codec.cv2_to_imgmsg(annotated, header=msg.header))

        self.get_logger().info(
            f'{class_name} ({confidence:.2f}) [{inference_ms:.1f} ms, ONNX]',
            throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ClassifierONNXNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
