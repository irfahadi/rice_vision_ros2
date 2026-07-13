# Satu hasil deteksi objek dari model YOLOv8
string class_name        # nama kelas objek (misal: "bibit_sehat", "bibit_tidak_sehat")
int32 class_id           # indeks kelas pada model
float32 confidence       # skor kepercayaan deteksi (0.0 - 1.0)

# Bounding box dalam koordinat piksel gambar (format xyxy)
int32 x_min
int32 y_min
int32 x_max
int32 y_max
