"""Launch file utama: menjalankan camera_node + detector_node sekaligus.

Contoh:
  ros2 launch rice_vision detection.launch.py
  ros2 launch rice_vision detection.launch.py model_path:=/path/ke/best.pt
  ros2 launch rice_vision detection.launch.py camera_id:=1 device:=cuda:0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rice_vision')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        # ---- Argumen yang bisa dioverride dari terminal ----
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Path file YAML parameter'),
        DeclareLaunchArgument(
            'model_path', default_value='yolov8n.pt',
            description='Path bobot YOLOv8 custom (.pt). '
                        'Default yolov8n.pt (auto-download, untuk uji coba)'),
        DeclareLaunchArgument(
            'camera_id', default_value='0',
            description='Index kamera (0 = /dev/video0)'),
        DeclareLaunchArgument(
            'device', default_value='cpu',
            description="Device inferensi: 'cpu' atau 'cuda:0'"),
        DeclareLaunchArgument(
            'confidence_threshold', default_value='0.5',
            description='Ambang confidence deteksi'),

        # ---- Node kamera ----
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

        # ---- Node deteksi YOLOv8 ----
        Node(
            package='rice_vision',
            executable='detector_node',
            name='detector_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'model_path': LaunchConfiguration('model_path'),
                    'device': LaunchConfiguration('device'),
                    'confidence_threshold':
                        LaunchConfiguration('confidence_threshold'),
                },
            ],
        ),
    ])
