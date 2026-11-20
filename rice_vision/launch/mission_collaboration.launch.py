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

  # SIMULASI tanpa Jetson/GPU (onnxruntime CPU + broker lokal + video file):
  ros2 launch rice_vision mission_collaboration.launch.py \
      vision_type:=onnx mqtt_broker:=127.0.0.1 video_source:=/path/tray.mp4

  # Menggunakan koneksi Serial UART (butuh firmware RW612 yang mendukung UART):
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
    default_onnx = os.path.join(pkg_share, 'models', 'models_bibit_padi.onnx')

    return LaunchDescription([
        # ---- Argumen Launch ----
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Path file konfigurasi YAML parameter'),
        DeclareLaunchArgument(
            'vision_type', default_value='trt',
            description='Tipe inferensi visi: trt (TensorRT), onnx (onnxruntime CPU), '
                        'keras (.h5), atau yolo (.pt)'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='Path custom model inferensi (kosong = ikuti params.yaml)'),
        DeclareLaunchArgument(
            'camera_id', default_value='0',
            description='Index perangkat kamera (/dev/video0)'),
        DeclareLaunchArgument(
            'video_source', default_value='',
            description='Simulasi: path video/pola gambar pengganti webcam (kosong = webcam)'),
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
            'mqtt_port', default_value='1883',
            description='Port MQTT Broker'),
        DeclareLaunchArgument(
            'robot_id', default_value='RB01',
            description='ID Collaboration Agent Robot'),
        DeclareLaunchArgument(
            'greenhouse_id', default_value='GH01',
            description='ID Smart Greenhouse'),
        DeclareLaunchArgument(
            'tray_inspection_interval_sec', default_value='15.0',
            description='Durasi pengumpulan sampel per tray (detik)'),
        DeclareLaunchArgument(
            'auto_inspect', default_value='true',
            description='Inspeksi tray berurutan secara otomatis (true/false)'),

        # ---- Node Kamera ----
        Node(
            package='rice_vision',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'camera_id': LaunchConfiguration('camera_id'),
                 'video_source': LaunchConfiguration('video_source')},
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
                {'model_path': PythonExpression([
                    "'", LaunchConfiguration('model_path'), "' or '",
                    default_onnx, "'"])},
            ],
        ),

        # ---- Node Visi: ONNX Runtime (CPU, simulasi/laptop) ----
        Node(
            package='rice_vision',
            executable='classifier_onnx_node',
            name='classifier_onnx_node',
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('vision_type'), "' == 'onnx'"])
            ),
            parameters=[
                LaunchConfiguration('params_file'),
                {'model_path': PythonExpression([
                    "'", LaunchConfiguration('model_path'), "' or '",
                    default_onnx, "'"])},
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
                    'mqtt_port': LaunchConfiguration('mqtt_port'),
                    'greenhouse_id': LaunchConfiguration('greenhouse_id'),
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
                    'tray_inspection_interval_sec': LaunchConfiguration('tray_inspection_interval_sec'),
                    'auto_inspect': LaunchConfiguration('auto_inspect'),
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

