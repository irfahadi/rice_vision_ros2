"""Launch file untuk Mission Collaboration Agent + Sink Node NPA-VEE-RW612 Bridge.

Menjalankan pipeline terintegrasi:
1. camera_node: stream kamera USB/webcam
2. vision node (classifier_trt_node / classifier_node / detector_node)
3. sink_bridge_node: bridge komunikasi MQTT/Serial ke Sink Node RW612
4. collaboration_agent_node: pengelola siklus misi, interval status, & evaluasi tray
5. viewer_node (opsional): display GUI OpenCV dengan panel status misi

Contoh penggunaan:
  # Mode default (TensorRT + MQTT bridge):
  ros2 launch rice_vision mission_collaboration.launch.py

  # Menyesuaikan broker Sink Node RW612:
  ros2 launch rice_vision mission_collaboration.launch.py \
      mqtt_broker:=168.110.214.70 robot_id:=RB01

  # Menggunakan model Keras .h5:
  ros2 launch rice_vision mission_collaboration.launch.py \
      vision_type:=keras model_path:=$HOME/models/bibit_padi.h5

  # Menggunakan koneksi Serial UART ke hardware board RW612:
  ros2 launch rice_vision mission_collaboration.launch.py \
      transport:=serial serial_port:=/dev/ttyACM0

  # Mode headless (tanpa GUI viewer):
  ros2 launch rice_vision mission_collaboration.launch.py viewer:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rice_vision')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        # ---- Argumen Launch ----
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Path file konfigurasi YAML parameter'),
        DeclareLaunchArgument(
            'vision_type', default_value='trt',
            description='Tipe inferensi visi: trt (TensorRT), keras (.h5), atau yolo (.pt)'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='Path custom model inferensi (kosong = ikuti params.yaml)'),
        DeclareLaunchArgument(
            'camera_id', default_value='0',
            description='Index perangkat kamera (/dev/video0)'),
        DeclareLaunchArgument(
            'viewer', default_value='true',
            description='Tampilkan visualizer GUI OpenCV (true/false)'),
        DeclareLaunchArgument(
            'transport', default_value='mqtt',
            description='Transport ke Sink Node: mqtt atau serial'),
        DeclareLaunchArgument(
            'mqtt_broker', default_value='168.110.214.70',
            description='Alamat IP/Host MQTT Broker Sink Node RW612'),
        DeclareLaunchArgument(
            'robot_id', default_value='RB01',
            description='ID Collaboration Agent Robot'),
        DeclareLaunchArgument(
            'greenhouse_id', default_value='SH01',
            description='ID Smart Greenhouse'),

        # ---- Node Kamera ----
        Node(
            package='rice_vision',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'camera_id': LaunchConfiguration('camera_id')},
            ],
        ),

        # ---- Node Visi: TensorRT (.onnx/.engine) ----
        Node(
            package='rice_vision',
            executable='classifier_trt_node',
            name='classifier_trt_node',
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('vision_type'), "' == 'trt'"])
            ),
            parameters=[
                LaunchConfiguration('params_file'),
                {'model_path': LaunchConfiguration('model_path')},
            ],
        ),

        # ---- Node Visi: Keras (.h5) ----
        Node(
            package='rice_vision',
            executable='classifier_node',
            name='classifier_node',
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('vision_type'), "' == 'keras'"])
            ),
            parameters=[
                LaunchConfiguration('params_file'),
                {'model_path': LaunchConfiguration('model_path')},
            ],
        ),

        # ---- Node Visi: YOLOv8 (.pt) ----
        Node(
            package='rice_vision',
            executable='detector_node',
            name='detector_node',
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('vision_type'), "' == 'yolo'"])
            ),
            parameters=[
                LaunchConfiguration('params_file'),
                {'model_path': LaunchConfiguration('model_path')},
            ],
        ),

        # ---- Node Bridge Sink Node NPA-VEE-RW612 ----
        Node(
            package='rice_vision',
            executable='sink_bridge_node',
            name='sink_bridge_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'transport': LaunchConfiguration('transport'),
                    'mqtt_broker': LaunchConfiguration('mqtt_broker'),
                },
            ],
        ),

        # ---- Node Collaboration Agent ----
        Node(
            package='rice_vision',
            executable='collaboration_agent_node',
            name='collaboration_agent_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'robot_id': LaunchConfiguration('robot_id'),
                    'greenhouse_id': LaunchConfiguration('greenhouse_id'),
                },
            ],
        ),

        # ---- Node Viewer GUI (Opsional) ----
        Node(
            package='rice_vision',
            executable='viewer_node',
            name='viewer_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('viewer')),
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
