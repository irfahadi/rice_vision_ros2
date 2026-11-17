# Kontrak data mission/status (Collaboration Agent -> Sink Node -> Cloud)
# Maksimal <= 128 byte, interval Ti = 2-5 s saat misi aktif
string m_id        # ID Misi aktif (contoh: "M20261023-01")
string r_id        # ID Robot / Collaboration Agent (contoh: "RB01")
string st          # Status agent (contoh: "IN_PROGRESS", "IDLE", "COMPLETED", "PAUSED", "ABORTED", "ERROR")
int32 cur_tray     # Tray yang sedang diinspeksi/dikerjakan (contoh: 3)
int32 pct          # Persentase kemajuan misi 0-100 (contoh: 50)
int32 bat          # Persentase baterai robot 0-100 (contoh: 84)
int32 seq          # Nomor urut sequence (contoh: 12)
int64 ts           # Unix timestamp detik (contoh: 1761200045)

