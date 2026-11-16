#!/usr/bin/env python3
"""Bridge node antara Sink Node (NPA-VEE-RW612) dan Collaboration Agent (ROS 2).

Mendukung transportasi MQTT (default) dan Serial UART:
1. Menerima 'mission/command' dari Sink Node -> mem-publish ke ROS 2
2. Menerima 'mission/status' dari ROS 2 -> mengirim ke Sink Node (interval 2-5 s)
3. Menerima 'mission/result' dari ROS 2 -> mengirim ke Sink Node (event-driven per tray)

Memvalidasi semua payload terhadap batas ukuran byte dan struktur schema JSON:
- mission/command <= 192 byte
- mission/status  <= 128 byte
- mission/result  <= 256 byte
"""

import json
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rice_vision.contracts import (
    MAX_BYTES_MISSION_COMMAND,
    MAX_BYTES_MISSION_RESULT,
    MAX_BYTES_MISSION_STATUS,
    TOPIC_MISSION_COMMAND,
    TOPIC_MISSION_RESULT,
    TOPIC_MISSION_STATUS,
    ContractValidationError,
    MissionCommand,
    MissionResult,
    MissionStatus,
    PayloadSizeExceededError,
)

# Coba import custom ROS 2 messages bila sudah di-build
try:
    from rice_vision_interfaces.msg import (
        MissionCommand as RosMissionCommand,
        MissionResult as RosMissionResult,
        MissionStatus as RosMissionStatus,
    )
    HAVE_CUSTOM_MSGS = True
except ImportError:
    HAVE_CUSTOM_MSGS = False
    RosMissionCommand = None
    RosMissionStatus = None
    RosMissionResult = None

# Coba import paho-mqtt
try:
    import paho.mqtt.client as mqtt
    HAVE_PAHO = True
except ImportError:
    HAVE_PAHO = False
    mqtt = None

# Coba import serial
try:
    import serial
    HAVE_SERIAL = True
except ImportError:
    HAVE_SERIAL = False
    serial = None


class SinkBridgeNode(Node):
    def __init__(self):
        super().__init__('sink_bridge_node')

        # ---- Parameter Transport & Koneksi ----
        self.declare_parameter('transport', 'mqtt')              # 'mqtt' atau 'serial'
        self.declare_parameter('mqtt_broker', '168.110.214.70')  # Sink Node / Cloud MQTT Broker
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_client_id', 'rice_vision_collab_agent')
        self.declare_parameter('mqtt_username', '')
        self.declare_parameter('mqtt_password', '')
        self.declare_parameter('mqtt_keepalive', 60)
        self.declare_parameter('topic_prefix', '')               # contoh: 'gh01/' bila diperlukan
        self.declare_parameter('topic_command', TOPIC_MISSION_COMMAND)
        self.declare_parameter('topic_status', TOPIC_MISSION_STATUS)
        self.declare_parameter('topic_result', TOPIC_MISSION_RESULT)

        self.declare_parameter('serial_port', '/dev/ttyACM0')    # port hardware Sink Node RW612
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('serial_timeout', 1.0)

        # Ambil nilai parameter
        self.transport = str(self.get_parameter('transport').value).lower()
        self.mqtt_broker = str(self.get_parameter('mqtt_broker').value)
        self.mqtt_port = int(self.get_parameter('mqtt_port').value)
        self.mqtt_client_id = str(self.get_parameter('mqtt_client_id').value)
        self.mqtt_username = str(self.get_parameter('mqtt_username').value)
        self.mqtt_password = str(self.get_parameter('mqtt_password').value)
        self.mqtt_keepalive = int(self.get_parameter('mqtt_keepalive').value)
        self.topic_prefix = str(self.get_parameter('topic_prefix').value)

        self.t_cmd = self.topic_prefix + str(self.get_parameter('topic_command').value)
        self.t_status = self.topic_prefix + str(self.get_parameter('topic_status').value)
        self.t_result = self.topic_prefix + str(self.get_parameter('topic_result').value)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.serial_baudrate = int(self.get_parameter('serial_baudrate').value)
        self.serial_timeout = float(self.get_parameter('serial_timeout').value)

        # ---- State Transport ----
        self.mqtt_client: Optional[Any] = None
        self.serial_conn = None
        self.is_connected = False
        self._running = True
        self._serial_thread: Optional[threading.Thread] = None

        # ---- ROS 2 Publishers (ke Collaboration Agent) ----
        if HAVE_CUSTOM_MSGS:
            self.cmd_pub = self.create_publisher(RosMissionCommand, '/mission/command', 10)
        else:
            self.cmd_pub = None
            self.get_logger().warn(
                'rice_vision_interfaces belum terinstall di ROS environment; '
                'hanya topic raw JSON (/mission/command/raw) yang akan aktif.'
            )
        self.cmd_raw_pub = self.create_publisher(String, '/mission/command/raw', 10)

        # ---- ROS 2 Subscriptions (dari Collaboration Agent) ----
        if HAVE_CUSTOM_MSGS:
            self.create_subscription(
                RosMissionStatus, '/mission/status', self.on_ros_status, 10
            )
            self.create_subscription(
                RosMissionResult, '/mission/result', self.on_ros_result, 10
            )
        self.create_subscription(
            String, '/mission/status/raw', self.on_ros_status_raw, 10
        )
        self.create_subscription(
            String, '/mission/result/raw', self.on_ros_result_raw, 10
        )

        # Inisialisasi Transport
        if self.transport == 'mqtt':
            self._init_mqtt()
        elif self.transport == 'serial':
            self._init_serial()
        else:
            self.get_logger().error(f"Transport tidak dikenal: '{self.transport}'. Gunakan 'mqtt' atau 'serial'.")

        self.get_logger().info(
            f'SinkBridgeNode aktif [transport={self.transport}]. '
            f'Sub: {self.t_cmd} | Pub: {self.t_status}, {self.t_result}'
        )

    # ------------------------------------------------------------------
    # MQTT Transport
    # ------------------------------------------------------------------
    def _init_mqtt(self):
        if not HAVE_PAHO:
            self.get_logger().error(
                'Package paho-mqtt tidak ditemukan! Jalankan: pip install paho-mqtt'
            )
            return

        try:
            self.mqtt_client = mqtt.Client(client_id=self.mqtt_client_id)
            if self.mqtt_username:
                self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)

            self.mqtt_client.on_connect = self._on_mqtt_connect
            self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
            self.mqtt_client.on_message = self._on_mqtt_message

            self.get_logger().info(
                f'Menghubungkan ke MQTT Broker Sink Node: {self.mqtt_broker}:{self.mqtt_port} ...'
            )
            self.mqtt_client.connect_async(
                self.mqtt_broker, self.mqtt_port, keepalive=self.mqtt_keepalive
            )
            self.mqtt_client.loop_start()
        except Exception as exc:
            self.get_logger().error(f'Gagal inisialisasi MQTT: {exc}')

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.is_connected = True
            self.get_logger().info(f'Terhubung ke Sink Node MQTT Broker! Subscribing {self.t_cmd} ...')
            client.subscribe(self.t_cmd, qos=1)
        else:
            self.is_connected = False
            self.get_logger().warn(f'Koneksi MQTT gagal dengan kode hasil: {rc}')

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.is_connected = False
        self.get_logger().warn(f'Terputus dari Sink Node MQTT Broker (rc={rc}). Menunggu reconnect...')

    def _on_mqtt_message(self, client, userdata, msg):
        payload_bytes = msg.payload
        topic = msg.topic
        self.get_logger().info(
            f'Menerima pesan MQTT [{topic}], ukuran: {len(payload_bytes)} byte'
        )

        if topic == self.t_cmd or topic.endswith(TOPIC_MISSION_COMMAND):
            self._handle_incoming_command(payload_bytes)

    # ------------------------------------------------------------------
    # Serial Transport
    # ------------------------------------------------------------------
    def _init_serial(self):
        if not HAVE_SERIAL:
            self.get_logger().error('Package pyserial tidak ditemukan! Jalankan: pip install pyserial')
            return

        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baudrate,
                timeout=self.serial_timeout,
            )
            self.is_connected = True
            self.get_logger().info(
                f'Terhubung ke serial Sink Node: {self.serial_port} @ {self.serial_baudrate} bps'
            )
            self._serial_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
            self._serial_thread.start()
        except Exception as exc:
            self.get_logger().error(f'Gagal membuka serial port {self.serial_port}: {exc}')

    def _serial_reader_loop(self):
        buffer = ''
        while self._running and self.serial_conn and self.serial_conn.is_open:
            try:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                # Format serial bisa: "mission/command:{...}" atau raw JSON "{...}"
                if line.startswith(f'{TOPIC_MISSION_COMMAND}:'):
                    json_str = line.split(':', 1)[1]
                elif line.startswith('{'):
                    json_str = line
                else:
                    continue

                self._handle_incoming_command(json_str.encode('utf-8'))
            except Exception as exc:
                self.get_logger().warn(f'Error membaca serial Sink Node: {exc}')
                time.sleep(1.0)

    # ------------------------------------------------------------------
    # Penerimaan Command (Cloud -> Sink Node -> Collaboration Agent)
    # ------------------------------------------------------------------
    def _handle_incoming_command(self, payload_bytes: bytes):
        size = len(payload_bytes)
        if size > MAX_BYTES_MISSION_COMMAND:
            self.get_logger().error(
                f'REJECT: Ukuran mission/command {size} byte melebihi batas {MAX_BYTES_MISSION_COMMAND} byte'
            )
            return

        try:
            cmd = MissionCommand.from_json(payload_bytes)
        except (ContractValidationError, PayloadSizeExceededError) as exc:
            self.get_logger().error(f'REJECT: Payload mission/command tidak valid: {exc}')
            return

        self.get_logger().info(
            f'VALID mission/command diterima: m_id={cmd.m_id}, cmd={cmd.cmd}, '
            f'trays={cmd.trays}, crit_lvl={cmd.crit_lvl} ({size} byte <= 192 byte)'
        )

        # 1. Publish ke raw JSON topic
        raw_msg = String()
        raw_msg.data = cmd.to_json()
        self.cmd_raw_pub.publish(raw_msg)

        # 2. Publish ke strongly-typed topic bila tersedia
        if self.cmd_pub is not None and HAVE_CUSTOM_MSGS:
            ros_msg = RosMissionCommand()
            ros_msg.m_id = cmd.m_id
            ros_msg.h_id = cmd.h_id
            ros_msg.cmd = cmd.cmd
            ros_msg.trays = cmd.trays
            ros_msg.crit_lvl = cmd.crit_lvl
            ros_msg.ts = int(cmd.ts)
            self.cmd_pub.publish(ros_msg)

    # ------------------------------------------------------------------
    # Pengiriman Status (Collaboration Agent -> Sink Node -> Cloud)
    # ------------------------------------------------------------------
    def on_ros_status(self, msg):
        status = MissionStatus(
            m_id=msg.m_id,
            r_id=msg.r_id,
            st=msg.st,
            cur_tray=msg.cur_tray,
            pct=msg.pct,
            bat=msg.bat,
            seq=msg.seq,
            ts=msg.ts,
        )
        self._transmit_status(status)

    def on_ros_status_raw(self, msg: String):
        try:
            status = MissionStatus.from_json(msg.data)
            self._transmit_status(status)
        except Exception as exc:
            self.get_logger().error(f'Format status raw JSON tidak valid: {exc}')

    def _transmit_status(self, status: MissionStatus):
        try:
            json_str = status.to_json()
            payload_bytes = json_str.encode('utf-8')
            size = len(payload_bytes)
            if size > MAX_BYTES_MISSION_STATUS:
                raise PayloadSizeExceededError(
                    f'Ukuran status {size} byte > batas {MAX_BYTES_MISSION_STATUS} byte'
                )

            if self.transport == 'mqtt' and self.mqtt_client is not None:
                self.mqtt_client.publish(self.t_status, payload_bytes, qos=0)
            elif self.transport == 'serial' and self.serial_conn is not None:
                packet = f'{TOPIC_MISSION_STATUS}:{json_str}\n'.encode('utf-8')
                self.serial_conn.write(packet)

            self.get_logger().info(
                f'Transmitted mission/status: m_id={status.m_id}, st={status.st}, '
                f'tray={status.cur_tray}, pct={status.pct}%, bat={status.bat}%, '
                f'seq={status.seq} ({size} byte <= 128 byte)'
            )
        except Exception as exc:
            self.get_logger().error(f'Gagal transmit mission/status: {exc}')

    # ------------------------------------------------------------------
    # Pengiriman Result (Collaboration Agent -> Sink Node -> Cloud)
    # ------------------------------------------------------------------
    def on_ros_result(self, msg):
        result = MissionResult(
            m_id=msg.m_id,
            h_id=msg.h_id,
            tray_id=msg.tray_id,
            label=msg.label,
            conf=float(msg.conf),
            anom=int(msg.anom),
            seq=int(msg.seq),
            ts=int(msg.ts),
        )
        self._transmit_result(result)

    def on_ros_result_raw(self, msg: String):
        try:
            result = MissionResult.from_json(msg.data)
            self._transmit_result(result)
        except Exception as exc:
            self.get_logger().error(f'Format result raw JSON tidak valid: {exc}')

    def _transmit_result(self, result: MissionResult):
        try:
            json_str = result.to_json()
            payload_bytes = json_str.encode('utf-8')
            size = len(payload_bytes)
            if size > MAX_BYTES_MISSION_RESULT:
                raise PayloadSizeExceededError(
                    f'Ukuran result {size} byte > batas {MAX_BYTES_MISSION_RESULT} byte'
                )

            if self.transport == 'mqtt' and self.mqtt_client is not None:
                self.mqtt_client.publish(self.t_result, payload_bytes, qos=1)
            elif self.transport == 'serial' and self.serial_conn is not None:
                packet = f'{TOPIC_MISSION_RESULT}:{json_str}\n'.encode('utf-8')
                self.serial_conn.write(packet)

            self.get_logger().info(
                f'Transmitted mission/result: m_id={result.m_id}, tray={result.tray_id}, '
                f'label={result.label}, conf={result.conf:.2f}, anom={result.anom}, '
                f'seq={result.seq} ({size} byte <= 256 byte)'
            )
        except Exception as exc:
            self.get_logger().error(f'Gagal transmit mission/result: {exc}')

    def destroy_node(self):
        self._running = False
        if self.mqtt_client is not None:
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except Exception:
                pass
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SinkBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
