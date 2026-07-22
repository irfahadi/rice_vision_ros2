#!/usr/bin/env python3
"""TensorRT (.onnx / .engine) image classification node.

Versi ber-akselerasi GPU dari classifier_node, memakai NVIDIA TensorRT
untuk inferensi tercepat di Ubuntu 20.04. Cocok untuk model CNN
klasifikasi bibit padi (Bagus / Tidak Bagus) yang sudah di-export ke ONNX
dari model Keras .h5.

Alur:
  .h5 (Keras)  --tf2onnx-->  model.onnx  --TensorRT-->  engine (GPU)

Node ini:
  - Menerima .onnx (dibangun jadi engine TensorRT, lalu di-cache sebagai
    .engine agar run berikutnya tidak perlu build ulang), ATAU .engine/.plan
    yang sudah jadi (langsung dimuat).
  - Preprocessing PERSIS seperti training Keras: RGB, resize 224x224, /255.
    Layout (NHWC/NCHW) dideteksi otomatis dari bentuk input engine.
  - Mem-publish ke topic yang SAMA dengan classifier_node/detector_node:
      /rice_vision/detections       (DetectionArray, 1 deteksi full-frame)
      /rice_vision/annotated_image  (frame + label + border)
    sehingga viewer_node dan subscriber lain bekerja tanpa perubahan.

Butuh: tensorrt (8.x) + pycuda. Lihat docs/TENSORRT_ONNX.md untuk instalasi
dan cara convert .h5 -> .onnx.
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


class ClassifierTRTNode(Node):
    def __init__(self):
        super().__init__('classifier_trt_node')

        # ---- Parameter ----
        self.declare_parameter('model_path', '')          # .onnx / .engine
        self.declare_parameter('engine_cache', '')        # path simpan .engine
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('results_topic', '/rice_vision/detections')
        self.declare_parameter('annotated_topic', '/rice_vision/annotated_image')
        self.declare_parameter('class_names',
                               ['Bibit Padi (Bagus)',
                                'Bibit Padi (Tidak Bagus)'])
        # 0 = auto (dibaca dari engine). Isi manual bila auto gagal.
        self.declare_parameter('input_width', 0)
        self.declare_parameter('input_height', 0)
        self.declare_parameter('layout', 'auto')          # auto|nhwc|nchw
        self.declare_parameter('threshold', 0.5)
        self.declare_parameter('fp16', True)              # build FP16 (lbh cepat)
        self.declare_parameter('workspace_mb', 2048)      # memori build engine
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('max_classification_rate', 30.0)

        model_path = str(self.get_parameter('model_path').value)
        engine_cache = str(self.get_parameter('engine_cache').value)
        image_topic = self.get_parameter('image_topic').value
        results_topic = self.get_parameter('results_topic').value
        annotated_topic = self.get_parameter('annotated_topic').value
        self.class_names = list(self.get_parameter('class_names').value)
        self.in_w = int(self.get_parameter('input_width').value)
        self.in_h = int(self.get_parameter('input_height').value)
        self.layout = str(self.get_parameter('layout').value).lower()
        self.threshold = float(self.get_parameter('threshold').value)
        self.fp16 = bool(self.get_parameter('fp16').value)
        self.workspace_mb = int(self.get_parameter('workspace_mb').value)
        self.publish_annotated = self.get_parameter('publish_annotated').value
        max_rate = float(self.get_parameter('max_classification_rate').value)
        self.min_interval = (1.0 / max_rate) if max_rate > 0 else 0.0
        self.last_inference_time = 0.0

        if not model_path:
            self.get_logger().fatal(
                'Parameter "model_path" kosong. Jalankan dengan:\n'
                '  ros2 launch rice_vision classification_trt.launch.py '
                'model_path:=/path/ke/model.onnx')
            raise RuntimeError('model_path belum diisi')

        # ---- Import TensorRT + pycuda (pesan jelas bila belum ada) ----
        try:
            import tensorrt as trt
            import pycuda.driver as cuda
            import pycuda.autoinit  # noqa: F401  (buat konteks CUDA)
        except ImportError as exc:
            self.get_logger().fatal(
                f'TensorRT/pycuda belum terpasang ({exc}).\n'
                'Lihat docs/TENSORRT_ONNX.md. Ringkas:\n'
                '  pip install pycuda\n'
                '  (tensorrt biasanya dari paket NVIDIA / apt, bukan pip)')
            raise

        self.trt = trt
        self.cuda = cuda
        self.TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

        model_path = os.path.expanduser(model_path)
        engine = self._load_or_build_engine(model_path, engine_cache)
        self.engine = engine
        self.context = engine.create_execution_context()

        self._setup_bindings()

        # ---- ROS I/O ----
        self.results_pub = self.create_publisher(
            DetectionArray, results_topic, 10)
        self.annotated_pub = self.create_publisher(
            Image, annotated_topic, qos_profile_sensor_data)
        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'Classifier TensorRT aktif: input {self.in_w}x{self.in_h} '
            f'({self.layout.upper()}), {self.n_outputs} output. '
            f'subscribe "{image_topic}" -> "{results_topic}".')

    # ------------------------------------------------------------------
    def _load_or_build_engine(self, model_path, engine_cache):
        trt = self.trt
        runtime = trt.Runtime(self.TRT_LOGGER)

        ext = os.path.splitext(model_path)[1].lower()
        if ext in ('.engine', '.plan', '.trt'):
            self.get_logger().info(f'Memuat engine TensorRT: {model_path}')
            with open(model_path, 'rb') as f:
                return runtime.deserialize_cuda_engine(f.read())

        # .onnx -> cek cache dulu
        if not engine_cache:
            engine_cache = model_path + '.engine'
        engine_cache = os.path.expanduser(engine_cache)

        if os.path.isfile(engine_cache) and \
                os.path.getmtime(engine_cache) >= os.path.getmtime(model_path):
            self.get_logger().info(
                f'Memuat engine dari cache: {engine_cache}')
            with open(engine_cache, 'rb') as f:
                return runtime.deserialize_cuda_engine(f.read())

        # Build dari ONNX
        self.get_logger().info(
            f'Membangun engine TensorRT dari ONNX: {model_path} '
            f'(FP16={self.fp16}) ... ini bisa memakan waktu 1-3 menit.')
        serialized = self._build_from_onnx(model_path)
        if serialized is None:
            raise RuntimeError('Gagal membangun engine dari ONNX')

        try:
            with open(engine_cache, 'wb') as f:
                f.write(serialized)
            self.get_logger().info(f'Engine di-cache ke: {engine_cache}')
        except OSError as exc:
            self.get_logger().warn(f'Tidak bisa menyimpan cache engine: {exc}')

        return runtime.deserialize_cuda_engine(serialized)

    def _build_from_onnx(self, onnx_path):
        trt = self.trt
        builder = trt.Builder(self.TRT_LOGGER)
        flag = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
        network = builder.create_network(flag)
        parser = trt.OnnxParser(network, self.TRT_LOGGER)

        with open(onnx_path, 'rb') as f:
            if not parser.parse(f.read()):
                for i in range(parser.num_errors):
                    self.get_logger().error(
                        f'ONNX parse error: {parser.get_error(i)}')
                return None

        config = builder.create_builder_config()
        # Batas workspace (API beda antar versi TRT)
        ws_bytes = self.workspace_mb * 1024 * 1024
        try:
            config.set_memory_pool_limit(
                trt.MemoryPoolType.WORKSPACE, ws_bytes)
        except AttributeError:
            config.max_workspace_size = ws_bytes  # TRT lama

        if self.fp16 and builder.platform_has_fast_fp16:
            config.set_flag(trt.BuilderFlag.FP16)

        # Tangani batch dinamis: paksa batch = 1 lewat optimization profile
        inp = network.get_input(0)
        in_shape = list(inp.shape)
        if in_shape and in_shape[0] in (-1, None):
            profile = builder.create_optimization_profile()
            fixed = [1] + [max(1, d) for d in in_shape[1:]]
            profile.set_shape(inp.name, fixed, fixed, fixed)
            config.add_optimization_profile(profile)

        # API build beda antar versi TRT
        if hasattr(builder, 'build_serialized_network'):
            return builder.build_serialized_network(network, config)
        engine = builder.build_engine(network, config)  # TRT lama
        return engine.serialize() if engine is not None else None

    # ------------------------------------------------------------------
    def _setup_bindings(self):
        trt = self.trt
        cuda = self.cuda
        engine = self.engine

        self.inputs = []
        self.outputs = []
        self.bindings = []
        self.stream = cuda.Stream()
        self.n_outputs = 1

        for binding in engine:
            idx = engine.get_binding_index(binding)
            shape = list(engine.get_binding_shape(binding))
            # Ganti dimensi dinamis (-1) dengan 1 (batch)
            shape = [1 if d < 0 else d for d in shape]
            if engine.binding_is_input(binding):
                self.context.set_binding_shape(idx, tuple(shape))

            dtype = trt.nptype(engine.get_binding_dtype(binding))
            size = int(np.prod(shape))
            host_mem = cuda.pagelocked_empty(size, dtype)
            dev_mem = cuda.mem_alloc(host_mem.nbytes)
            self.bindings.append(int(dev_mem))

            entry = {'host': host_mem, 'device': dev_mem,
                     'shape': shape, 'dtype': dtype}
            if engine.binding_is_input(binding):
                self.inputs.append(entry)
                self._resolve_input_layout(shape)
            else:
                self.outputs.append(entry)

        # Jumlah kelas = ukuran output (tanpa batch)
        if self.outputs:
            out_shape = self.outputs[0]['shape']
            self.n_outputs = int(np.prod(out_shape[1:])) if len(out_shape) > 1 \
                else int(np.prod(out_shape))

    def _resolve_input_layout(self, shape):
        """Tentukan H, W, dan layout dari bentuk input engine."""
        # shape 4D: [N, C, H, W] (nchw) atau [N, H, W, C] (nhwc)
        if self.layout == 'auto':
            if len(shape) == 4 and shape[1] == 3:
                self.layout = 'nchw'
            elif len(shape) == 4 and shape[3] == 3:
                self.layout = 'nhwc'
            else:
                self.layout = 'nchw'
                self.get_logger().warn(
                    f'Layout input tidak jelas dari shape {shape}, '
                    'asumsi NCHW. Set param "layout" bila salah.')

        if self.layout == 'nchw':
            _, _, h, w = (shape + [0, 0, 0, 0])[:4]
        else:  # nhwc
            _, h, w, _ = (shape + [0, 0, 0, 0])[:4]

        # Param manual menimpa auto bila diisi (>0)
        if self.in_h <= 0:
            self.in_h = int(h) if h > 0 else 224
        if self.in_w <= 0:
            self.in_w = int(w) if w > 0 else 224

    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        now = time.monotonic()
        if self.min_interval > 0 and (now - self.last_inference_time) < self.min_interval:
            return
        self.last_inference_time = now

        frame = image_codec.imgmsg_to_cv2(msg)
        h, w = frame.shape[:2]

        # ---- Preprocessing sama seperti training Keras ----
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (self.in_w, self.in_h),
                             interpolation=cv2.INTER_AREA)
        norm = resized.astype(np.float32) / 255.0
        if self.layout == 'nchw':
            norm = np.transpose(norm, (2, 0, 1))  # HWC -> CHW
        batch = np.ascontiguousarray(np.expand_dims(norm, axis=0))

        # ---- Inferensi TensorRT ----
        t0 = time.monotonic()
        out = self._infer(batch)
        inference_ms = (time.monotonic() - t0) * 1000.0

        # ---- Interpretasi output ----
        flat = np.asarray(out).ravel()
        if self.n_outputs == 1:
            p = float(flat[0])
            class_id = 1 if p > self.threshold else 0
            confidence = p if class_id == 1 else 1.0 - p
        else:
            class_id = int(np.argmax(flat))
            confidence = float(flat[class_id])

        class_name = (self.class_names[class_id]
                      if class_id < len(self.class_names)
                      else f'kelas_{class_id}')

        # ---- Publish (DetectionArray full-frame) ----
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

        # ---- Frame beranotasi ----
        if self.publish_annotated:
            annotated = frame.copy()
            color = (0, 200, 0) if class_id == 0 else (0, 0, 220)
            cv2.rectangle(annotated, (0, 0), (w - 1, h - 1), color, 6)
            label = f'{class_name}  {confidence:.2f}  [TRT]'
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
            cv2.rectangle(annotated, (10, 10), (18 + tw, 24 + th), color, -1)
            cv2.putText(annotated, label, (14, 16 + th),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (255, 255, 255), 2, cv2.LINE_AA)
            self.annotated_pub.publish(
                image_codec.cv2_to_imgmsg(annotated, header=msg.header))

        self.get_logger().info(
            f'{class_name} ({confidence:.2f}) [{inference_ms:.1f} ms, TRT]',
            throttle_duration_sec=2.0)

    def _infer(self, batch):
        cuda = self.cuda
        inp = self.inputs[0]
        # Salin input ke host buffer lalu ke device
        np.copyto(inp['host'], batch.ravel().astype(inp['dtype'], copy=False))
        cuda.memcpy_htod_async(inp['device'], inp['host'], self.stream)

        # Jalankan
        self.context.execute_async_v2(
            bindings=self.bindings, stream_handle=self.stream.handle)

        # Ambil output
        for out in self.outputs:
            cuda.memcpy_dtoh_async(out['host'], out['device'], self.stream)
        self.stream.synchronize()

        return self.outputs[0]['host']


def main(args=None):
    rclpy.init(args=args)
    node = ClassifierTRTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
