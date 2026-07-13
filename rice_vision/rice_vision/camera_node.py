#!/usr/bin/env python3
"""Camera publisher node.

Membaca frame dari webcam/USB camera menggunakan OpenCV dan mem-publish
ke topic ROS 2 sebagai sensor_msgs/Image (encoding bgr8).

Terinspirasi arsitektur pada paper Rezakhani & Kim (2025): kamera robot
mem-publish ke topic "/camera", lalu modul computer vision men-subscribe
topic tersebut sebagai input model deteksi objek.
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header

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

        camera_id = self.get_parameter('camera_id').value
        frame_rate = self.get_parameter('frame_rate').value
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value
        image_topic = self.get_parameter('image_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # ---- OpenCV capture ----
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Tidak bisa membuka kamera id={camera_id}. '
                'Cek apakah kamera terpasang (ls /dev/video*).')
            raise RuntimeError(f'Cannot open camera {camera_id}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.publisher = self.create_publisher(
            Image, image_topic, qos_profile_sensor_data)

        period = 1.0 / max(frame_rate, 1.0)
        self.timer = self.create_timer(period, self.capture_and_publish)

        self.get_logger().info(
            f'Camera node aktif: kamera={camera_id}, {width}x{height} '
            f'@{frame_rate}fps -> topic "{image_topic}"')

    def capture_and_publish(self):
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
