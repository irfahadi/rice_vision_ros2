#!/usr/bin/env python3
"""Integrated Launch File: myCobot 280 Pi/M5 Scanner + Rice Vision Collaboration Pipeline.

Menjalankan sistem kolaborasi robot inspeksi baki semai secara terintegrasi:
1. Robot State Publisher & Kinematika (myCobot 280 M5 / Pi)
2. Scanner Node (Simulasi RViz2 atau Hardware Nyata)
3. Camera Node (Webcam USB atau generator frame bibit padi tersinkronisasi)
4. Vision Node (Classifier ONNX Runtime CPU atau TensorRT)
5. Sink Bridge Node (Koneksi MQTT Broker ke SmartGreenhouse / Sink Node)
6. Collaboration Agent Node (Siklus misi, status berkala, evaluasi baki & hasil inspeksi)
7. GUI Viewers opsional (RViz2 & OpenCV Window)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def evaluate_nodes(context, *args, **kwargs):
    mode = context.perform_substitution(LaunchConfiguration('mode')).lower()
    model = context.perform_substitution(LaunchConfiguration('model'))
    port = context.perform_substitution(LaunchConfiguration('port'))
    baud = int(context.perform_substitution(LaunchConfiguration('baud')))
    speed = int(context.perform_substitution(LaunchConfiguration('speed')))
    wait_for_mission = context.perform_substitution(LaunchConfiguration('wait_for_mission')).lower() == 'true'
    scan_duration = float(context.perform_substitution(LaunchConfiguration('scan_duration')))
    transition_duration = float(context.perform_substitution(LaunchConfiguration('transition_duration')))
    upright_duration = float(context.perform_substitution(LaunchConfiguration('upright_duration')))
    pitch_sign = float(context.perform_substitution(LaunchConfiguration('pitch_sign')))
    pitch_total_deg = float(context.perform_substitution(LaunchConfiguration('pitch_total_deg')))
    keep_tool_yaw = context.perform_substitution(LaunchConfiguration('keep_tool_yaw')).lower() == 'true'
    with_rviz = context.perform_substitution(LaunchConfiguration('rviz')).lower() == 'true'
    with_viewer = context.perform_substitution(LaunchConfiguration('viewer')).lower() == 'true'
    vision_type = context.perform_substitution(LaunchConfiguration('vision_type')).lower()
    model_path = context.perform_substitution(LaunchConfiguration('model_path'))
    mqtt_broker = context.perform_substitution(LaunchConfiguration('mqtt_broker'))
    mqtt_port = int(context.perform_substitution(LaunchConfiguration('mqtt_port')))
    robot_id = context.perform_substitution(LaunchConfiguration('robot_id'))
    greenhouse_id = context.perform_substitution(LaunchConfiguration('greenhouse_id'))
    tray_interval = float(context.perform_substitution(LaunchConfiguration('tray_inspection_interval_sec')))
    camera_id = context.perform_substitution(LaunchConfiguration('camera_id'))
    video_source = context.perform_substitution(LaunchConfiguration('video_source'))

    pkg_mycobot = get_package_share_directory('mycobot_280_pi_scanner')
    pkg_vision = get_package_share_directory('rice_vision')

    urdf_filename = f'{model}.urdf'
    urdf_path = os.path.join(pkg_mycobot, 'urdf', urdf_filename)
    if not os.path.exists(urdf_path):
        urdf_path = os.path.join(pkg_mycobot, 'urdf', 'mycobot_280_m5.urdf')

    rviz_filename = f'{model}.rviz'
    rviz_path = os.path.join(pkg_mycobot, 'rviz', rviz_filename)
    if not os.path.exists(rviz_path):
        rviz_path = os.path.join(pkg_mycobot, 'rviz', 'mycobot_280_m5.rviz')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    default_params = os.path.join(pkg_vision, 'config', 'params.yaml')
    default_onnx = os.path.join(pkg_vision, 'models', 'models_bibit_padi.onnx')
    actual_model_path = model_path if model_path else default_onnx

    nodes = []

    # 1. Robot State Publisher
    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        )
    )

    # 2. Scanner Node (Simulasi RViz atau Hardware Nyata)
    if mode == 'device':
        nodes.append(
            Node(
                package='mycobot_280_pi_scanner',
                executable='device_scanner_node',
                name='device_scanner_node',
                output='screen',
                parameters=[{
                    'port': port,
                    'baud': baud,
                    'speed': speed,
                    'upright_duration': upright_duration,
                    'transition_duration': transition_duration,
                    'scan_duration': scan_duration,
                    'pitch_sign': pitch_sign,
                    'pitch_total_deg': pitch_total_deg,
                    'keep_tool_yaw': keep_tool_yaw,
                    'wait_for_mission': wait_for_mission,
                }],
            )
        )
    else:
        nodes.append(
            Node(
                package='mycobot_280_pi_scanner',
                executable='rviz_scanner_node',
                name='rviz_scanner_node',
                output='screen',
                parameters=[{
                    'wait_for_mission': wait_for_mission,
                    'upright_duration': upright_duration,
                    'transition_duration': transition_duration,
                    'scan_duration': scan_duration,
                    'pitch_sign': pitch_sign,
                    'pitch_total_deg': pitch_total_deg,
                    'keep_tool_yaw': keep_tool_yaw,
                }],
            )
        )

    # 3. RViz2 (Opsional)
    if with_rviz:
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_path],
            )
        )

    # 4. Camera Node
    nodes.append(
        Node(
            package='rice_vision',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                default_params,
                {
                    'camera_id': int(camera_id) if camera_id.isdigit() else 0,
                    'video_source': video_source,
                    'dummy_fallback': True,
                    'sync_with_scanner': True,
                },
            ],
        )
    )

    # 5. Vision Classifier Node (ONNX CPU atau TensorRT)
    if vision_type == 'trt':
        nodes.append(
            Node(
                package='rice_vision',
                executable='classifier_trt_node',
                name='classifier_trt_node',
                output='screen',
                parameters=[
                    default_params,
                    {'model_path': actual_model_path},
                ],
            )
        )
    else:
        nodes.append(
            Node(
                package='rice_vision',
                executable='classifier_onnx_node',
                name='classifier_onnx_node',
                output='screen',
                parameters=[
                    default_params,
                    {'model_path': actual_model_path},
                ],
            )
        )

    # 6. Sink Bridge Node (MQTT Connection)
    nodes.append(
        Node(
            package='rice_vision',
            executable='sink_bridge_node',
            name='sink_bridge_node',
            output='screen',
            parameters=[
                default_params,
                {
                    'transport': 'mqtt',
                    'mqtt_broker': mqtt_broker,
                    'mqtt_port': mqtt_port,
                    'greenhouse_id': greenhouse_id,
                    'publish_direct_cloud': True,
                },
            ],
        )
    )

    # 7. Collaboration Agent Node
    nodes.append(
        Node(
            package='rice_vision',
            executable='collaboration_agent_node',
            name='collaboration_agent_node',
            output='screen',
            parameters=[
                default_params,
                {
                    'robot_id': robot_id,
                    'greenhouse_id': greenhouse_id,
                    'tray_inspection_interval_sec': tray_interval,
                    'auto_inspect': True,
                },
            ],
        )
    )

    # 8. Viewer GUI Node (Opsional)
    if with_viewer:
        nodes.append(
            Node(
                package='rice_vision',
                executable='viewer_node',
                name='viewer_node',
                output='screen',
                parameters=[default_params],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim',
                              description='Mode eksekusi: sim (RViz2) atau device (Hardware fisik)'),
        DeclareLaunchArgument('model', default_value='mycobot_280_m5',
                              description='Model robot: mycobot_280_m5 atau mycobot_280_pi'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='Serial port untuk hardware (/dev/ttyUSB0 atau /dev/ttyAMA0)'),
        DeclareLaunchArgument('baud', default_value='115200',
                              description='Baud rate serial (115200 untuk M5, 1000000 untuk Pi)'),
        DeclareLaunchArgument('speed', default_value='35',
                              description='Kecepatan servo robot (1-100)'),
        DeclareLaunchArgument('wait_for_mission', default_value='true',
                              description='Tunggu perintah MQTT INSPECT sebelum bergerak memindai'),
        DeclareLaunchArgument('scan_duration', default_value='30.0',
                              description='Durasi gerakan memindai (detik)'),
        DeclareLaunchArgument('transition_duration', default_value='3.0',
                              description='Durasi transisi pose ke/dari pose pemindaian (detik)'),
        DeclareLaunchArgument('upright_duration', default_value='5.0',
                              description='Durasi posisi tegak standby saat non-mission (detik)'),
        DeclareLaunchArgument('pitch_sign', default_value='1.0',
                              description='Faktor kalibrasi tanda pitch'),
        DeclareLaunchArgument('pitch_total_deg', default_value='-45.0',
                              description='Batas sudut total pitch ujung lengan'),
        DeclareLaunchArgument('keep_tool_yaw', default_value='false',
                              description='Kunci yaw end-effector selama scanning'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Buka jendela 3D GUI RViz2 (true/false)'),
        DeclareLaunchArgument('viewer', default_value='false',
                              description='Buka jendela OpenCV GUI Viewer (true/false)'),
        DeclareLaunchArgument('vision_type', default_value='onnx',
                              description='Tipe inferensi: onnx (CPU/laptop/sim) atau trt (TensorRT Jetson)'),
        DeclareLaunchArgument('model_path', default_value='',
                              description='Path custom model deep learning'),
        DeclareLaunchArgument('mqtt_broker', default_value='127.0.0.1',
                              description='IP/Host MQTT Broker Sink Node / Cloud'),
        DeclareLaunchArgument('mqtt_port', default_value='1883',
                              description='Port MQTT Broker'),
        DeclareLaunchArgument('robot_id', default_value='RB01',
                              description='ID Robot Collaboration Agent'),
        DeclareLaunchArgument('greenhouse_id', default_value='GH01',
                              description='ID Screenhouse (GH01 -> Screenhouse 90)'),
        DeclareLaunchArgument('tray_inspection_interval_sec', default_value='15.0',
                              description='Durasi sampling pengumpulan deteksi per tray (detik)'),
        DeclareLaunchArgument('camera_id', default_value='0',
                              description='Index kamera (/dev/video0)'),
        DeclareLaunchArgument('video_source', default_value='',
                              description='Path video pengganti webcam (kosong = webcam/dummy fallback)'),

        OpaqueFunction(function=evaluate_nodes),
    ])
