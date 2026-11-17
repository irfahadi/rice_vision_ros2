# rice_vision_ros2

Integrasi **computer vision (YOLOv8)** dengan **ROS 2 Foxy** untuk deteksi objek
real-time dari webcam/USB camera — misalnya deteksi kualitas bibit padi dengan
model custom hasil training sendiri.

Arsitektur mengikuti pola pada paper *"Real-time inspection in construction:
Integrating ROS and APS for computer vision applications"* (Rezakhani & Kim,
2025): kamera mem-publish frame ke topic ROS, node computer vision men-subscribe
topic tersebut, menjalankan YOLOv8, lalu mem-publish hasil deteksi + frame
beranotasi kembali ke jaringan ROS.

```
┌──────────────┐  /camera/image_raw   ┌───────────────┐  /rice_vision/detections
│ camera_node  │ ───────────────────► │ detector_node │ ───────────────────────►
│ (OpenCV/USB) │  sensor_msgs/Image   │   (YOLOv8)    │  DetectionArray (custom)
└──────────────┘                      └───────┬───────┘
                                              │ /rice_vision/annotated_image
                                              ▼ sensor_msgs/Image
                                      ┌───────────────┐
                                      │  viewer_node  │  jendela + panel info
                                      │  (interface)  │  (bisa juga rqt/rviz2)
                                      └───────────────┘
```

## Isi Repo

| Package | Tipe | Fungsi |
|---|---|---|
| `rice_vision_interfaces` | ament_cmake (msg) | Pesan custom `Detection`, `DetectionArray`, `MissionCommand`, `MissionStatus`, & `MissionResult` |
| `rice_vision` | ament_python | `camera_node` (webcam), `detector_node` (YOLOv8 .pt), `classifier_node` (Keras .h5), `classifier_trt_node` (TensorRT .onnx), `classifier_onnx_node` (onnxruntime CPU, untuk simulasi/laptop), `viewer_node` (jendela tampilan), `sink_bridge_node` (bridge Sink Node RW612), & `collaboration_agent_node` (pengelola misi inspeksi) |

## Prasyarat

- Ubuntu 20.04 + [ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation.html) (desktop install)
- Python 3.8 (bawaan Ubuntu 20.04)
- Webcam / USB camera (cek dengan `ls /dev/video*`)

## Instalasi

Dependency Python untuk YOLOv8 (`ultralytics`, `torch`, dll.) butuh versi
`numpy` yang lebih baru daripada bawaan Ubuntu 20.04. **Jangan install
langsung ke Python sistem** — pakai virtual environment dengan
`--system-site-packages` supaya tetap bisa mengakses `rclpy` dkk dari ROS:

```bash
# 1. Dependency ROS (perhatikan: python3-opencv, BUKAN cv-bridge --
#    lihat catatan di bagian Troubleshooting)
sudo apt update
sudo apt install -y python3-opencv python3-pip python3-venv \
                    python3-colcon-common-extensions

# 2. Buat workspace & extract repo ini ke src/
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# extract zip di sini, sehingga strukturnya: ~/ros2_ws/src/rice_vision_ros2/

# 3. Buat virtual environment (sekali saja)
cd ~/ros2_ws
python3 -m venv .venv --system-site-packages
echo 'export PYTHONNOUSERSITE=1' >> .venv/bin/activate   # cegah bentrok dgn ~/.local
source .venv/bin/activate

# 4. Dependency Python (YOLOv8). Kalau sudah punya torch yang cocok dengan
#    GPU/CUDA kamu, install ultralytics tanpa menimpanya:
#      pip install --no-deps ultralytics==8.0.196
#      pip install numpy==1.23.5 opencv-python
#    Kalau belum punya torch sama sekali, pakai requirements.txt penuh
#    (baca komentar torch di dalamnya untuk pilihan CPU vs CUDA):
pip install -r rice_vision_ros2/requirements.txt

# 5. Build (WAJIB dengan venv aktif, supaya node memakai Python venv ini)
source /opt/ros/foxy/setup.bash
colcon build --symlink-install

# 6. Source hasil build (lakukan di setiap terminal baru, urutan penting:
#    venv dulu baru install/setup.bash)
source .venv/bin/activate
source ~/ros2_ws/install/setup.bash
```

> **Tips:** jangan tambahkan baris `source` ke `~/.bashrc` untuk repo ini,
> karena venv-nya sebaiknya diaktifkan manual per sesi kerja (agar tidak
> tertukar dengan proyek ROS lain yang tidak butuh venv ini).

## Menjalankan

### Uji cepat (model pretrained COCO, auto-download)

```bash
ros2 launch rice_vision detection.launch.py
```

`yolov8n.pt` akan diunduh otomatis oleh Ultralytics saat pertama kali jalan —
berguna untuk memastikan pipeline kamera → deteksi bekerja sebelum model
custom kamu siap.

### Dengan model custom (mis. deteksi bibit padi)

Letakkan bobot hasil training di `rice_vision/models/` lalu:

```bash
ros2 launch rice_vision detection.launch.py \
  model_path:=$HOME/ros2_ws/src/rice_vision_ros2/rice_vision/models/bibit_padi_best.pt
```

Argumen launch lain yang tersedia:

```bash
ros2 launch rice_vision detection.launch.py \
  camera_id:=1 \                 # pakai /dev/video1
  device:=cuda:0 \               # inferensi di GPU NVIDIA
  confidence_threshold:=0.6
```

Konfigurasi permanen bisa diedit di `rice_vision/config/params.yaml`.

### Mode klasifikasi: model Keras .h5 (CNN bibit padi)

Selain YOLO (.pt), repo ini juga mendukung **model klasifikasi Keras/
TensorFlow (.h5)** lewat `classifier_node` — cocok untuk CNN biner
"Bibit Padi Bagus / Tidak Bagus" yang dilatih dengan
`ImageDataGenerator(rescale=1./255)`, `target_size=(224,224)`,
`class_mode='binary'`, dan output `Dense(1, activation='sigmoid')`.

Bedanya dengan YOLO: model klasifikasi menilai **keseluruhan frame**
(tidak ada bounding box per objek), jadi hasilnya satu label + confidence
per frame. Node ini melakukan preprocessing persis seperti saat training
(RGB, resize 224×224, ÷255) lalu mem-publish ke topic yang sama dengan
pipeline YOLO, sehingga viewer bekerja tanpa perubahan.

```bash
# 1. Install TensorFlow (sekali saja, di dalam venv)
pip install tensorflow==2.13.1

# 2. Jalankan dengan model .h5 kamu
ros2 launch rice_vision classification.launch.py \
  model_path:=$HOME/models/model_bibit_padi.h5
```

Interpretasi output mengikuti script training: sigmoid > `threshold`
(default 0.5) → kelas indeks 1 ("Tidak Bagus"), selain itu kelas indeks 0
("Bagus"). Urutan nama kelas bisa diubah di `params.yaml`
(`classifier_node.class_names`) — urutannya harus sama dengan indeks
`flow_from_directory` (alfabetis nama folder dataset). Model dengan output
softmax multi-kelas juga otomatis didukung (argmax).

> **Catatan:** TF 2.13 (versi terakhir untuk Python 3.8) butuh CUDA 11.8
> untuk GPU — dengan CUDA 12.x biarkan default `use_gpu: false` (CPU);
> CNN 3-layer seperti ini tetap cepat di CPU. Kalau `.h5` kamu disimpan
> dari Keras 3 (Colab terbaru) dan gagal dimuat, simpan ulang di Colab
> dengan `model.save('model.h5', save_format='h5')`.

### Mode klasifikasi cepat: TensorRT + model .onnx (GPU)

Untuk inferensi **tercepat di GPU**, pakai `classifier_trt_node` dengan
model ONNX (hasil convert dari `.h5`) yang dijalankan lewat **NVIDIA
TensorRT**. Node ini membangun engine TensorRT dari `.onnx` (lalu meng-cache
sebagai `.engine`) atau langsung memuat `.engine` yang sudah jadi, dan
mem-publish ke topic yang sama sehingga viewer tetap bekerja.

```bash
# 1. Install TensorRT + pycuda (CUDA 12.x -> TensorRT 8.6.x), di dalam venv
pip install tensorrt==8.6.1 pycuda

# 2. Convert model .h5 -> .onnx (sekali; lihat docs/TENSORRT_ONNX.md)
pip install tf2onnx onnx
#    lalu jalankan script convert di dokumen tersebut

# 3. Jalankan (build engine otomatis saat pertama kali, ~1-3 menit)
ros2 launch rice_vision classification_trt.launch.py \
  model_path:=$HOME/models/model_bibit_padi.onnx
```

Panduan lengkap (instalasi TensorRT, convert `.h5`→`.onnx`, parameter,
troubleshooting) ada di **[docs/TENSORRT_ONNX.md](docs/TENSORRT_ONNX.md)**.

Ringkasan tiga cara klasifikasi/deteksi:

| Node | Model | Runtime | Kecepatan |
|---|---|---|---|
| `detector_node` | YOLOv8 `.pt` | Ultralytics/torch | deteksi objek + bbox |
| `classifier_node` | Keras `.h5` | TensorFlow (CPU) | klasifikasi, tanpa convert |
| `classifier_trt_node` | `.onnx`/`.engine` | TensorRT (GPU) | klasifikasi tercepat |

### Melihat hasil (interface)

Ada tiga cara menampilkan hasil deteksi:

**1. Viewer bawaan (jendela khusus, direkomendasikan).** `detection.launch.py`
sudah otomatis membuka jendela viewer (`viewer_node`) yang menampilkan feed
kamera beranotasi + panel info (jumlah objek, FPS, waktu inferensi, rincian
tiap objek). Tidak perlu perintah tambahan -- cukup:

```bash
ros2 launch rice_vision detection.launch.py
```

Tekan `q` pada jendela untuk menutup. Untuk mode headless (tanpa GUI, mis.
di server/SSH tanpa X), matikan viewer:

```bash
ros2 launch rice_vision detection.launch.py viewer:=false
```

Viewer juga bisa dijalankan sendiri di terminal terpisah (mis. kalau kamu
menjalankan camera_node + detector_node di mesin lain):

```bash
ros2 run rice_vision viewer_node
```

**2. rqt_image_view (bawaan ROS).**

```bash
ros2 run rqt_image_view rqt_image_view /rice_vision/annotated_image
```

**3. rviz2.** Add → Image → topic `/rice_vision/annotated_image`.

Data deteksi terstruktur (kelas, confidence, bbox, jumlah objek):

```bash
ros2 topic echo /rice_vision/detections
```

Contoh output:

```yaml
header:
  frame_id: camera_optical_frame
detections:
- class_name: bibit_sehat
  class_id: 0
  confidence: 0.91
  x_min: 120
  y_min: 84
  x_max: 260
  y_max: 233
count: 1
inference_time_ms: 42.7
```

## Topics & Parameter

### Topics

| Topic | Tipe | Arah | Keterangan |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | camera_node → | Frame mentah webcam (bgr8) |
| `/rice_vision/detections` | `rice_vision_interfaces/DetectionArray` | detector_node / classifier → | Hasil deteksi terstruktur |
| `/rice_vision/annotated_image` | `sensor_msgs/Image` | detector_node / classifier → | Frame dengan label / bbox |
| `/mission/command` | `rice_vision_interfaces/MissionCommand` | sink_bridge_node → | Perintah misi dari Cloud/Sink Node (≤ 192 byte) |
| `/mission/status` | `rice_vision_interfaces/MissionStatus` | collaboration_agent_node → | Status berkala robot ke Sink Node (≤ 128 byte, Ti=2-5s) |
| `/mission/result` | `rice_vision_interfaces/MissionResult` | collaboration_agent_node → | Hasil inspeksi tray ke Sink Node (≤ 256 byte) |

### Parameter penting (`config/params.yaml`)

| Parameter | Node | Default | Keterangan |
|---|---|---|---|
| `camera_id` | camera_node | `0` | Index kamera (`/dev/video0`) |
| `frame_rate` | camera_node | `15.0` | FPS publikasi kamera |
| `model_path` | detector_node | `yolov8n.pt` | Path bobot YOLOv8 custom |
| `confidence_threshold` | detector_node | `0.5` | Ambang confidence |
| `device` | detector_node | `cpu` | `cpu` atau `cuda:0` |
| `max_detection_rate` | detector_node | `10.0` | Batas frame/detik yang diinferensi |

## Menghubungkan dengan node lain (mis. kontrol robot)

Node lain tinggal subscribe ke `/rice_vision/detections`:

```python
from rice_vision_interfaces.msg import DetectionArray

self.create_subscription(
    DetectionArray, '/rice_vision/detections', self.on_detections, 10)

def on_detections(self, msg):
    for det in msg.detections:
        self.get_logger().info(
            f'{det.class_name} ({det.confidence:.2f}) '
            f'di [{det.x_min},{det.y_min},{det.x_max},{det.y_max}]')
```

## Integrasi Sink Node NPA-VEE-RW612 & Kontrak Data Misi

Modul integrasi ini menghubungkan **Collaboration Agent (Robot/Jetson)** dengan **Sink Node (`npa-vee-rw612`)** dan **Cloud** menggunakan spesifikasi kontrak data yang terstandardisasi:

```
┌─────────────┐                      ┌──────────────────────┐                     ┌─────────────────────┐
│    Cloud    │ ── mission/command ─►│      Sink Node       │ ─ mission/command ─►│ Collaboration Agent │
│  (Backend)  │ ◄── mission/status ──│   (NPA-VEE-RW612)    │ ◄── mission/status ─│    (ROS 2 RB01)     │
│             │ ◄── mission/result ──│                      │ ◄── mission/result ─│ (rice_vision_ros2)  │
└─────────────┘                      └──────────────────────┘                     └─────────────────────┘
```

### Tabel Kontrak Data Komunikasi

| Topik / Arah | Format Payload (JSON) | Batasan & Karakteristik |
|---|---|---|
| `mission/command`<br>`Cloud → Sink Node → Collaboration Agent` | `{"m_id":"M20261023-01", "h_id":"GH01", "cmd":"INSPECT", "trays":[1,2,3,4], "crit_lvl":1, "ts":1761200010}` | **≤ 192 byte**<br>*(event-driven saat penugasan)* |
| `mission/status`<br>`Collaboration Agent → Sink Node → Cloud` | `{"m_id":"M20261023-01", "r_id":"RB01", "st":"IN_PROGRESS", "cur_tray":3, "pct":50, "bat":84, "seq":12, "ts":1761200045}` | **≤ 128 byte**<br>*(interval Ti = 2–5 s saat misi aktif)* |
| `mission/result`<br>`Collaboration Agent → Sink Node → Cloud` | `{"m_id":"M20261023-01", "h_id":"GH01", "tray_id":3, "label":"BAGUS", "conf":0.94, "anom":0, "seq":3, "ts":1761200050}` | **≤ 256 byte**<br>*(event-driven per tray, ≈ 15–30 s)* |

### Topik MQTT (harus cocok dengan firmware Sink Node)

Firmware `apps/SmartGreenhouse/.../MqttSubscribeService.java` di repo
`npa-vee-rw612` **merelay** pesan misi antara sisi Cloud dan sisi agent.
Agent hanya boleh memakai topik `gh01/agent/...`:

| Arah | Cloud | Sink Node merelay ke/dari | Collaboration Agent (`sink_bridge_node`) |
|---|---|---|---|
| command | publish `gh01/mission/command` | → `gh01/agent/mission/command` | **subscribe** `gh01/agent/mission/command` |
| status | subscribe `gh01/mission/status` | ← `gh01/agent/mission/status` | **publish** `gh01/agent/mission/status` |
| result | subscribe `gh01/mission/result` | ← `gh01/agent/mission/result` | **publish** `gh01/agent/mission/result` |

Default `sink_bridge_node` sudah `topic_prefix: "gh01/agent/"` dan
`greenhouse_id: "GH01"` (sama dengan `HOUSE_ID` firmware). Sink Node membuang
pesan yang melebihi batas byte atau tidak punya `m_id` (+ `cmd`/`st`/`tray_id`),
memakai QoS 0, dan tidak mem-buffer saat offline. Firmware saat ini hanya
merelay misi lewat MQTT/WiFi; `transport:=serial` baru berguna bila firmware
RW612 ditambah pembaca UART `mission/<x>:{json}`.

### Node Pendukung

1. **`sink_bridge_node`**:
   - Menjembatani topic ROS 2 dengan Sink Node via MQTT (default) atau Serial UART.
   - Memvalidasi schema JSON dan batas ukuran byte sesuai kontrak data.
   - Kompatibel paho-mqtt 1.6.x dan 2.x.
   - ROS 2: subscribe `/mission/status` & `/mission/result` (typed; fallback ke
     `/mission/*/raw` hanya bila `rice_vision_interfaces` tidak ada), publish
     `/mission/command` (typed) dan `/mission/command/raw` (debug).

2. **`collaboration_agent_node`**:
   - Siklus misi `IN_PROGRESS` / `PAUSED` / `COMPLETED` / `ABORTED`; perintah
     PAUSE/RESUME/STOP/ABORT hanya berlaku untuk `m_id` misi aktif; command
     duplikat (kirim ulang/redelivery) diabaikan.
   - Status berkala `mission/status` (2–5 s).
   - Per tray: mengumpulkan deteksi `/rice_vision/detections` **selama tray itu
     diinspeksi** (`tray_inspection_interval_sec`), lalu voting mayoritas →
     `BAGUS` / `TIDAK BAGUS`, conf = rata-rata conf label pemenang.
   - Bila tidak ada deteksi sama sekali, menunggu `detection_grace_sec`, lalu
     mengirim `label="UNKNOWN", conf=0.0, anom=1` (perlu dicek manual) — tidak
     pernah mengarang hasil "BAGUS".
   - `auto_inspect: true` → tray berurutan. `auto_inspect: false` → menunggu
     `/mission/trigger_tray` (`std_msgs/Int32`, tray_id) dari sistem navigasi saat
     robot tiba di tray. `/mission/step_tray` (`std_msgs/Empty`) memaksa evaluasi.

### Menjalankan Mission Collaboration

```bash
# Hardware: Jetson + TensorRT, broker Sink Node RW612 (default 168.110.214.70)
ros2 launch rice_vision mission_collaboration.launch.py

# Kustomisasi broker dan robot ID:
ros2 launch rice_vision mission_collaboration.launch.py \
  mqtt_broker:=168.110.214.70 robot_id:=RB01 greenhouse_id:=GH01

# Tanpa GPU (laptop): model .onnx yang sama via onnxruntime, video sebagai kamera
pip install onnxruntime
ros2 launch rice_vision mission_collaboration.launch.py \
  vision_type:=onnx mqtt_broker:=127.0.0.1 video_source:=$HOME/tray_test.mp4
```

Uji manual dari sisi Cloud (dengan `mosquitto-clients`):

```bash
mosquitto_sub -h 168.110.214.70 -t 'gh01/mission/#' -v
```

```bash
mosquitto_pub -h 168.110.214.70 -t gh01/mission/command \
  -m '{"m_id":"M20261023-01","h_id":"GH01","cmd":"INSPECT","trays":[1,2],"crit_lvl":1,"ts":1761200010}'
```

### Simulasi integrasi sebelum ke hardware (`sim/`)

`sim/run_integration_sim.py` menjalankan **Cloud palsu ↔ firmware Sink Node
RW612 (kode Java asli, di JVM) ↔ node rice_vision asli** lewat broker
Mosquitto lokal, memakai model `models_bibit_padi.onnx`. ROS 2 diganti shim
in-process (`sim/fake_ros`), kamera diganti gambar sintetis per tray.

```bash
sudo apt install mosquitto default-jdk zip
pip install paho-mqtt onnxruntime opencv-python numpy
# build simulator Sink Node dari repo firmware (sekali)
sim/rw612_sink/build.sh ../npa-vee-rw612
python3 sim/run_integration_sim.py          # 5 skenario, exit code 0 = lulus
```

Skenario: alur normal 3 tray, kamera mati (UNKNOWN), PAUSE/RESUME/ABORT,
mode trigger navigasi, payload tidak valid & command duplikat. Waktu dipercepat
(tray 3 s, status 2 s). Yang **tidak** diuji simulasi: ROS 2/DDS asli, launch
file, TensorRT di Jetson, WiFi/broker produksi, dan **akurasi model** (gambar
sintetis bukan foto bibit; uji akurasi dengan foto/video tray asli via
`video_source`).

Unit test cepat (tanpa broker/ROS): `python3 -m pytest tests`.

## Troubleshooting

- **`Cannot open camera 0`** — cek `ls /dev/video*`, ganti `camera_id`, dan
  pastikan user masuk grup video: `sudo usermod -aG video $USER` (lalu logout).
- **`ImportError: Matplotlib requires numpy>=1.20; you have 1.17.4` atau
  node memuat `ultralytics` dari `/usr/local/lib/.../dist-packages` bukan
  dari venv** — ini artinya `ros2 launch` dijalankan **tanpa venv aktif**
  (atau `colcon build` dilakukan tanpa venv aktif sehingga script hasil
  build "terkunci" ke Python sistem). Perbaikan: hapus build lama lalu
  build ulang dengan venv aktif:
  ```bash
  source ~/ros2_ws/.venv/bin/activate
  cd ~/ros2_ws
  rm -rf build install log
  colcon build --symlink-install
  source install/setup.bash
  ros2 launch rice_vision detection.launch.py
  ```
- **`KeyError: 16` (atau error aneh lain) dari `cv_bridge`** — repo ini
  **tidak lagi memakai `cv_bridge`** (lihat `rice_vision/image_codec.py`)
  justru karena bug ini: `cv_bridge` adalah binary C++ yang di-compile
  melawan versi OpenCV/numpy tertentu, dan gampang bentrok begitu venv
  memakai `opencv-python`/`numpy` versi lain untuk ultralytics. Kalau kamu
  masih menemukan pesan ini, pastikan kamu memakai versi node terbaru dari
  repo ini (cek tidak ada lagi `from cv_bridge import CvBridge` di
  `camera_node.py` / `detector_node.py`).
- **Error terkait `nvidia-*-cu12` / `libnvJitLink.so` saat `import torch`**
  — biasanya ada beberapa versi paket `nvidia-*-cu12` tercampur antara
  `~/.local` dan venv. Install ulang torch dengan `--force-reinstall` di
  dalam venv yang aktif (lihat komentar di `requirements.txt` untuk index
  CUDA yang sesuai), dan pastikan `PYTHONNOUSERSITE=1` sudah ter-set
  (baris ini otomatis ditambahkan ke `.venv/bin/activate` pada langkah
  instalasi di atas) supaya `~/.local` tidak ikut ke-load.
- **Import error `ultralytics`** — jalankan lagi
  `pip3 install -r requirements.txt` **di dalam venv yang aktif**.
- **FPS rendah di CPU** — turunkan `max_detection_rate`/resolusi kamera, atau
  pakai model lebih kecil (`yolov8n`), atau set `device: cuda:0` jika ada GPU.
- **Node tidak ditemukan setelah build** — pastikan sudah
  `source ~/ros2_ws/install/setup.bash` di terminal tersebut (setelah
  venv diaktifkan).

## Roadmap (mengikuti paper)

- [ ] Tracking + counting objek unik dengan BoT-SORT (`model.track(...)`)
- [ ] Streaming hasil ke web via `web_video_server` / WebSocket
- [ ] Integrasi dengan simulasi Gazebo (topic kamera dari robot virtual)

## Lisensi

MIT — lihat [LICENSE](LICENSE).
