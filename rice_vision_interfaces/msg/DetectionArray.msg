# Kumpulan hasil deteksi untuk satu frame gambar
std_msgs/Header header
rice_vision_interfaces/Detection[] detections
int32 count              # jumlah objek terdeteksi pada frame ini
float32 inference_time_ms  # waktu inferensi model (milidetik)
