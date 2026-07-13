#!/usr/bin/env python3
"""Konversi manual antara array OpenCV (numpy) dan sensor_msgs/Image.

Kenapa tidak pakai cv_bridge? cv_bridge adalah ekstensi C++ yang di-compile
melawan versi OpenCV & numpy tertentu (biasanya bawaan apt/ROS). Kalau di
virtual environment kamu memakai opencv-python/numpy versi lain (dibutuhkan
oleh ultralytics/torch), cv_bridge sering gagal dengan error seperti
`KeyError: 16` di cv2_to_imgmsg/imgmsg_to_cv2 karena tabel tipe internalnya
tidak lagi cocok dengan OpenCV yang sedang berjalan.

Modul ini menggantikan cv_bridge dengan konversi manual yang sangat
sederhana untuk encoding 'bgr8' (uint8, 3 channel) -- satu-satunya encoding
yang dipakai di pipeline ini -- sehingga node tidak bergantung sama sekali
pada versi OpenCV/numpy yang dipakai cv_bridge.
"""

import numpy as np
from sensor_msgs.msg import Image


def cv2_to_imgmsg(frame: np.ndarray, header=None) -> Image:
    """Ubah array BGR uint8 (H, W, 3) jadi sensor_msgs/Image encoding bgr8."""
    if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError(
            f'Frame harus uint8 HxWx3 (BGR), didapat dtype={frame.dtype} '
            f'shape={frame.shape}')

    msg = Image()
    if header is not None:
        msg.header = header
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = int(frame.strides[0])
    msg.data = np.ascontiguousarray(frame).tobytes()
    return msg


def imgmsg_to_cv2(msg: Image) -> np.ndarray:
    """Ubah sensor_msgs/Image encoding bgr8 jadi array numpy BGR uint8."""
    if msg.encoding != 'bgr8':
        raise ValueError(
            f'Encoding "{msg.encoding}" belum didukung oleh image_codec '
            f'(hanya "bgr8"). Pesan datang dengan encoding "{msg.encoding}".')
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    return arr.reshape(msg.height, msg.width, 3)
