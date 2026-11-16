"""Data contracts and schemas for Sink Node (NPA-VEE-RW612) and Collaboration Agent.

Sesuai spesifikasi kontrak data komunikasi:
1. mission/command (Cloud -> Sink Node -> Collaboration Agent)
   Format: {"m_id":"M20261023-01", "h_id":"SH01", "cmd":"INSPECT", "trays":[1,2,3,4], "crit_lvl":1, "ts":1761200010}
   Batas: <= 192 byte (event-driven saat penugasan)

2. mission/status (Collaboration Agent -> Sink Node -> Cloud)
   Format: {"m_id":"M20261023-01", "r_id":"RB01", "st":"IN_PROGRESS", "cur_tray":3, "pct":50, "bat":84, "seq":12, "ts":1761200045}
   Batas: <= 128 byte (interval Ti = 2-5 s saat misi aktif)

3. mission/result (Collaboration Agent -> Sink Node -> Cloud)
   Format: {"m_id":"M20261023-01", "h_id":"SH01", "tray_id":3, "label":"BAGUS", "conf":0.94, "anom":0, "seq":3, "ts":1761200050}
   Batas: <= 256 byte (event-driven per tray, ~ 15-30 s)
"""

from dataclasses import dataclass
import json
import time
from typing import Any, Dict, List, Union


# Topik MQTT / Protokol
TOPIC_MISSION_COMMAND = 'mission/command'
TOPIC_MISSION_STATUS = 'mission/status'
TOPIC_MISSION_RESULT = 'mission/result'

# Batasan ukuran payload (dalam byte)
MAX_BYTES_MISSION_COMMAND = 192
MAX_BYTES_MISSION_STATUS = 128
MAX_BYTES_MISSION_RESULT = 256

# Perintah baku
CMD_INSPECT = 'INSPECT'
CMD_START = 'START'
CMD_STOP = 'STOP'
CMD_PAUSE = 'PAUSE'
CMD_RESUME = 'RESUME'
CMD_ABORT = 'ABORT'

# Status baku agen kolaborasi
STATUS_IDLE = 'IDLE'
STATUS_IN_PROGRESS = 'IN_PROGRESS'
STATUS_COMPLETED = 'COMPLETED'
STATUS_PAUSED = 'PAUSED'
STATUS_ABORTED = 'ABORTED'
STATUS_ERROR = 'ERROR'

# Label klasifikasi bibit & anomali
LABEL_BAGUS = 'BAGUS'
LABEL_TIDAK_BAGUS = 'TIDAK BAGUS'
ANOMALY_NORMAL = 0
ANOMALY_DEFECT = 1


class ContractValidationError(ValueError):
    """Kesalahan validasi data contract."""
    pass


class PayloadSizeExceededError(ContractValidationError):
    """Ukuran payload melebihi batas byte spesifikasi."""
    pass


def _dump_compact(data: Dict[str, Any]) -> str:
    """Serialisasi JSON tanpa spasi ekstra untuk efisiensi byte over-the-air."""
    return json.dumps(data, separators=(',', ':'), ensure_ascii=True)


@dataclass
class MissionCommand:
    """Kontrak data: mission/command (Cloud -> Sink Node -> Collaboration Agent)."""

    m_id: str
    h_id: str
    cmd: str
    trays: List[int]
    crit_lvl: int
    ts: int

    def validate(self) -> None:
        if not self.m_id or not isinstance(self.m_id, str):
            raise ContractValidationError("Field 'm_id' wajib string non-kosong")
        if not self.h_id or not isinstance(self.h_id, str):
            raise ContractValidationError("Field 'h_id' wajib string non-kosong")
        if not self.cmd or not isinstance(self.cmd, str):
            raise ContractValidationError("Field 'cmd' wajib string non-kosong")
        if not isinstance(self.trays, (list, tuple)):
            raise ContractValidationError("Field 'trays' wajib bertipe list integer")
        for t in self.trays:
            if not isinstance(t, int):
                raise ContractValidationError(f"Elemen tray {t} harus integer")
        if not isinstance(self.crit_lvl, int):
            raise ContractValidationError("Field 'crit_lvl' wajib bertipe integer")
        if not isinstance(self.ts, int) or self.ts <= 0:
            raise ContractValidationError("Field 'ts' wajib bertipe integer unix timestamp > 0")

        encoded_len = len(self.to_bytes())
        if encoded_len > MAX_BYTES_MISSION_COMMAND:
            raise PayloadSizeExceededError(
                f"Ukuran payload mission/command ({encoded_len} byte) melebihi batas {MAX_BYTES_MISSION_COMMAND} byte"
            )

    def to_dict(self) -> Dict[str, Any]:
        return {
            'm_id': str(self.m_id),
            'h_id': str(self.h_id),
            'cmd': str(self.cmd),
            'trays': [int(t) for t in self.trays],
            'crit_lvl': int(self.crit_lvl),
            'ts': int(self.ts),
        }

    def to_json(self) -> str:
        self.validate()
        return _dump_compact(self.to_dict())

    def to_bytes(self) -> bytes:
        data = {
            'm_id': str(self.m_id),
            'h_id': str(self.h_id),
            'cmd': str(self.cmd),
            'trays': [int(t) for t in self.trays],
            'crit_lvl': int(self.crit_lvl),
            'ts': int(self.ts),
        }
        return _dump_compact(data).encode('utf-8')

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'MissionCommand':
        if not isinstance(data, dict):
            raise ContractValidationError('Payload bukan dictionary JSON')
        for key in ('m_id', 'h_id', 'cmd', 'trays', 'crit_lvl', 'ts'):
            if key not in data:
                raise ContractValidationError(f"Field wajib '{key}' tidak ditemukan pada payload")

        trays_raw = data['trays']
        if not isinstance(trays_raw, (list, tuple)):
            raise ContractValidationError("Field 'trays' harus berupa list")

        cmd_obj = cls(
            m_id=str(data['m_id']),
            h_id=str(data['h_id']),
            cmd=str(data['cmd']),
            trays=[int(x) for x in trays_raw],
            crit_lvl=int(data['crit_lvl']),
            ts=int(data['ts']),
        )
        cmd_obj.validate()
        return cmd_obj

    @classmethod
    def from_json(cls, raw: Union[str, bytes]) -> 'MissionCommand':
        if isinstance(raw, bytes):
            if len(raw) > MAX_BYTES_MISSION_COMMAND:
                raise PayloadSizeExceededError(
                    f"Panjang raw bytes {len(raw)} byte melebihi batas {MAX_BYTES_MISSION_COMMAND} byte"
                )
            text = raw.decode('utf-8')
        else:
            text = raw
            if len(text.encode('utf-8')) > MAX_BYTES_MISSION_COMMAND:
                raise PayloadSizeExceededError(
                    f"Panjang text {len(text.encode('utf-8'))} byte melebihi batas {MAX_BYTES_MISSION_COMMAND} byte"
                )

        try:
            parsed = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ContractValidationError(f'Gagal mendecode JSON: {exc}') from exc

        return cls.from_dict(parsed)


@dataclass
class MissionStatus:
    """Kontrak data: mission/status (Collaboration Agent -> Sink Node -> Cloud)."""

    m_id: str
    r_id: str
    st: str
    cur_tray: int
    pct: int
    bat: int
    seq: int
    ts: int

    def validate(self) -> None:
        if not self.m_id or not isinstance(self.m_id, str):
            raise ContractValidationError("Field 'm_id' wajib string non-kosong")
        if not self.r_id or not isinstance(self.r_id, str):
            raise ContractValidationError("Field 'r_id' wajib string non-kosong")
        if not self.st or not isinstance(self.st, str):
            raise ContractValidationError("Field 'st' wajib string non-kosong")
        if not isinstance(self.cur_tray, int) or self.cur_tray < 0:
            raise ContractValidationError("Field 'cur_tray' wajib integer >= 0")
        if not isinstance(self.pct, int) or not (0 <= self.pct <= 100):
            raise ContractValidationError("Field 'pct' wajib integer antara 0 dan 100")
        if not isinstance(self.bat, int) or not (0 <= self.bat <= 100):
            raise ContractValidationError("Field 'bat' wajib integer antara 0 dan 100")
        if not isinstance(self.seq, int) or self.seq < 0:
            raise ContractValidationError("Field 'seq' wajib integer >= 0")
        if not isinstance(self.ts, int) or self.ts <= 0:
            raise ContractValidationError("Field 'ts' wajib integer unix timestamp > 0")

        encoded_len = len(self.to_bytes())
        if encoded_len > MAX_BYTES_MISSION_STATUS:
            raise PayloadSizeExceededError(
                f"Ukuran payload mission/status ({encoded_len} byte) melebihi batas {MAX_BYTES_MISSION_STATUS} byte"
            )

    def to_dict(self) -> Dict[str, Any]:
        return {
            'm_id': str(self.m_id),
            'r_id': str(self.r_id),
            'st': str(self.st),
            'cur_tray': int(self.cur_tray),
            'pct': int(self.pct),
            'bat': int(self.bat),
            'seq': int(self.seq),
            'ts': int(self.ts),
        }

    def to_json(self) -> str:
        self.validate()
        return _dump_compact(self.to_dict())

    def to_bytes(self) -> bytes:
        data = {
            'm_id': str(self.m_id),
            'r_id': str(self.r_id),
            'st': str(self.st),
            'cur_tray': int(self.cur_tray),
            'pct': int(self.pct),
            'bat': int(self.bat),
            'seq': int(self.seq),
            'ts': int(self.ts),
        }
        return _dump_compact(data).encode('utf-8')

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'MissionStatus':
        if not isinstance(data, dict):
            raise ContractValidationError('Payload bukan dictionary JSON')
        for key in ('m_id', 'r_id', 'st', 'cur_tray', 'pct', 'bat', 'seq', 'ts'):
            if key not in data:
                raise ContractValidationError(f"Field wajib '{key}' tidak ditemukan pada payload")

        obj = cls(
            m_id=str(data['m_id']),
            r_id=str(data['r_id']),
            st=str(data['st']),
            cur_tray=int(data['cur_tray']),
            pct=int(data['pct']),
            bat=int(data['bat']),
            seq=int(data['seq']),
            ts=int(data['ts']),
        )
        obj.validate()
        return obj

    @classmethod
    def from_json(cls, raw: Union[str, bytes]) -> 'MissionStatus':
        if isinstance(raw, bytes):
            if len(raw) > MAX_BYTES_MISSION_STATUS:
                raise PayloadSizeExceededError(
                    f"Panjang raw bytes {len(raw)} byte melebihi batas {MAX_BYTES_MISSION_STATUS} byte"
                )
            text = raw.decode('utf-8')
        else:
            text = raw
            if len(text.encode('utf-8')) > MAX_BYTES_MISSION_STATUS:
                raise PayloadSizeExceededError(
                    f"Panjang text {len(text.encode('utf-8'))} byte melebihi batas {MAX_BYTES_MISSION_STATUS} byte"
                )

        try:
            parsed = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ContractValidationError(f'Gagal mendecode JSON: {exc}') from exc

        return cls.from_dict(parsed)


@dataclass
class MissionResult:
    """Kontrak data: mission/result (Collaboration Agent -> Sink Node -> Cloud)."""

    m_id: str
    h_id: str
    tray_id: int
    label: str
    conf: float
    anom: int
    seq: int
    ts: int

    def validate(self) -> None:
        if not self.m_id or not isinstance(self.m_id, str):
            raise ContractValidationError("Field 'm_id' wajib string non-kosong")
        if not self.h_id or not isinstance(self.h_id, str):
            raise ContractValidationError("Field 'h_id' wajib string non-kosong")
        if not isinstance(self.tray_id, int) or self.tray_id < 0:
            raise ContractValidationError("Field 'tray_id' wajib integer >= 0")
        if not self.label or not isinstance(self.label, str):
            raise ContractValidationError("Field 'label' wajib string non-kosong")
        if not isinstance(self.conf, (int, float)) or not (0.0 <= self.conf <= 1.0):
            raise ContractValidationError("Field 'conf' wajib float 0.0 s.d 1.0")
        if self.anom not in (0, 1):
            raise ContractValidationError("Field 'anom' wajib 0 (normal) atau 1 (anomali)")
        if not isinstance(self.seq, int) or self.seq < 0:
            raise ContractValidationError("Field 'seq' wajib integer >= 0")
        if not isinstance(self.ts, int) or self.ts <= 0:
            raise ContractValidationError("Field 'ts' wajib integer unix timestamp > 0")

        encoded_len = len(self.to_bytes())
        if encoded_len > MAX_BYTES_MISSION_RESULT:
            raise PayloadSizeExceededError(
                f"Ukuran payload mission/result ({encoded_len} byte) melebihi batas {MAX_BYTES_MISSION_RESULT} byte"
            )

    def to_dict(self) -> Dict[str, Any]:
        return {
            'm_id': str(self.m_id),
            'h_id': str(self.h_id),
            'tray_id': int(self.tray_id),
            'label': str(self.label),
            'conf': round(float(self.conf), 2),
            'anom': int(self.anom),
            'seq': int(self.seq),
            'ts': int(self.ts),
        }

    def to_json(self) -> str:
        self.validate()
        return _dump_compact(self.to_dict())

    def to_bytes(self) -> bytes:
        data = {
            'm_id': str(self.m_id),
            'h_id': str(self.h_id),
            'tray_id': int(self.tray_id),
            'label': str(self.label),
            'conf': round(float(self.conf), 2),
            'anom': int(self.anom),
            'seq': int(self.seq),
            'ts': int(self.ts),
        }
        return _dump_compact(data).encode('utf-8')

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'MissionResult':
        if not isinstance(data, dict):
            raise ContractValidationError('Payload bukan dictionary JSON')
        for key in ('m_id', 'h_id', 'tray_id', 'label', 'conf', 'anom', 'seq', 'ts'):
            if key not in data:
                raise ContractValidationError(f"Field wajib '{key}' tidak ditemukan pada payload")

        obj = cls(
            m_id=str(data['m_id']),
            h_id=str(data['h_id']),
            tray_id=int(data['tray_id']),
            label=str(data['label']),
            conf=round(float(data['conf']), 2),
            anom=int(data['anom']),
            seq=int(data['seq']),
            ts=int(data['ts']),
        )
        obj.validate()
        return obj

    @classmethod
    def from_json(cls, raw: Union[str, bytes]) -> 'MissionResult':
        if isinstance(raw, bytes):
            if len(raw) > MAX_BYTES_MISSION_RESULT:
                raise PayloadSizeExceededError(
                    f"Panjang raw bytes {len(raw)} byte melebihi batas {MAX_BYTES_MISSION_RESULT} byte"
                )
            text = raw.decode('utf-8')
        else:
            text = raw
            if len(text.encode('utf-8')) > MAX_BYTES_MISSION_RESULT:
                raise PayloadSizeExceededError(
                    f"Panjang text {len(text.encode('utf-8'))} byte melebihi batas {MAX_BYTES_MISSION_RESULT} byte"
                )

        try:
            parsed = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ContractValidationError(f'Gagal mendecode JSON: {exc}') from exc

        return cls.from_dict(parsed)


def current_epoch_seconds() -> int:
    """Mengembalikan timestamp detik epoch saat ini."""
    return int(time.time())

