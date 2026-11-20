import os
import cv2
import numpy as np
import pytest

from rice_vision.camera_node import CameraNode
import rclpy


@pytest.fixture(scope="module")
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_camera_dummy_frame_generation(rclpy_init):
    node = CameraNode()
    
    # 1. Saat fase STANDBY / UPRIGHT (kamera menghadap langit-langit)
    node.scan_phase = 'UPRIGHT'
    frame_standby = node._generate_dummy_frame()
    assert frame_standby is not None
    assert frame_standby.shape == (node.height, node.width, 3)
    assert frame_standby.dtype == np.uint8

    # 2. Saat fase memindai FORWARD_SCANNING (kamera menghadap baki rumput)
    node.scan_phase = 'FORWARD_SCANNING'
    frame_grass = node._generate_dummy_frame()
    assert frame_grass is not None
    assert frame_grass.shape == (node.height, node.width, 3)
    assert frame_grass.dtype == np.uint8

    # Frame saat scanning harus berbeda secara signifikan dari frame standby
    diff = np.mean(np.abs(frame_grass.astype(float) - frame_standby.astype(float)))
    assert diff > 20.0, f"Frame standby dan scanning terlalu identik (diff={diff})"

    node.destroy_node()
