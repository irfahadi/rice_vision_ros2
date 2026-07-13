# Folder Model

Letakkan file bobot YOLOv8 custom kamu di sini, misalnya:

```
models/bibit_padi_best.pt
```

Lalu jalankan dengan:

```bash
ros2 launch rice_vision detection.launch.py \
  model_path:=$HOME/ros2_ws/src/rice_vision_ros2/rice_vision/models/bibit_padi_best.pt
```

atau edit `model_path` di `config/params.yaml`.

Catatan: file `.pt` sengaja di-ignore oleh git (lihat `.gitignore` di root repo)
agar repo tetap ringan. Simpan bobot model di release/Google Drive/HuggingFace.
