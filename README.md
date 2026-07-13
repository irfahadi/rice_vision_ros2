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
| `rice_vision_interfaces` | ament_cmake (msg) | Pesan custom `Detection` & `DetectionArray` |
| `rice_vision` | ament_python | `camera_node` (publisher webcam), `detector_node` (inferensi YOLOv8) & `viewer_node` (jendela tampilan) |

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
| `/rice_vision/detections` | `rice_vision_interfaces/DetectionArray` | detector_node → | Hasil deteksi terstruktur |
| `/rice_vision/annotated_image` | `sensor_msgs/Image` | detector_node → | Frame dengan bbox tergambar |

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
