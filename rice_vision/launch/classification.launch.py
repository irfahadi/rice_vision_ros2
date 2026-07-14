"""Launch file untuk model klasifikasi Keras .h5 (bibit padi Bagus/Tidak).

Menjalankan camera_node + classifier_node (+ viewer). Hasil klasifikasi
dipublish ke topic yang sama dengan pipeline YOLO, jadi viewer dan
subscriber lain bekerja tanpa perubahan.

Contoh:
  ros2 launch rice_vision classification.launch.py \
      model_path:=$HOME/models/model_bibit_padi.h5
  ros2 launch rice_vision classification.launch.py \
      model_path:=/path/model.h5 camera_id:=1 viewer:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rice_vision')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Path file YAML parameter'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='Path model Keras .h5 (WAJIB diisi)'),
        DeclareLaunchArgument(
            'camera_id', default_value='0',
            description='Index kamera (0 = /dev/video0)'),
        DeclareLaunchArgument(
            'threshold', default_value='0.5',
            description='Ambang sigmoid (>threshold = kelas indeks 1)'),
        DeclareLaunchArgument(
            'viewer', default_value='true',
            description='Tampilkan jendela viewer (true/false)'),

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

        # ---- Node klasifikasi Keras .h5 ----
        Node(
            package='rice_vision',
            executable='classifier_node',
            name='classifier_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'model_path': LaunchConfiguration('model_path'),
                    'threshold': LaunchConfiguration('threshold'),
                },
            ],
        ),

        # ---- Node viewer ----
        Node(
            package='rice_vision',
            executable='viewer_node',
            name='viewer_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('viewer')),
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
