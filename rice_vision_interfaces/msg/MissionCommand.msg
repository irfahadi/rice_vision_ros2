# Kontrak data mission/command (Cloud -> Sink Node -> Collaboration Agent)
# Maksimal <= 192 byte, event-driven saat penugasan
string m_id        # ID Misi (contoh: "M20261023-01")
string h_id        # ID Greenhouse / House (contoh: "SH01")
string cmd         # Perintah (contoh: "INSPECT", "START", "STOP", "PAUSE", "RESUME", "ABORT")
int32[] trays      # Daftar ID tray yang harus diinspeksi (contoh: [1, 2, 3, 4])
int32 crit_lvl     # Level kritikal/prioritas (contoh: 1)
int64 ts           # Unix timestamp detik (contoh: 1761200010)

