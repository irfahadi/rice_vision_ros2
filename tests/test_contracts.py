"""Unit tests untuk validasi kontrak data sink node NPA-VEE-RW612."""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../rice_vision')))

from rice_vision.contracts import (
    ContractValidationError,
    MAX_BYTES_MISSION_COMMAND,
    MAX_BYTES_MISSION_RESULT,
    MAX_BYTES_MISSION_STATUS,
    MissionCommand,
    MissionResult,
    MissionStatus,
    PayloadSizeExceededError,
    TOPIC_MISSION_COMMAND,
    TOPIC_MISSION_RESULT,
    TOPIC_MISSION_STATUS,
)


class TestMissionContracts(unittest.TestCase):

    def test_mission_command_spec(self):
        """Validasi payload persis contoh kontrak: mission/command."""
        payload = {
            "m_id": "M20261023-01",
            "h_id": "SH01",
            "cmd": "INSPECT",
            "trays": [1, 2, 3, 4],
            "crit_lvl": 1,
            "ts": 1761200010,
        }
        cmd = MissionCommand.from_dict(payload)
        json_out = cmd.to_json()
        encoded = json_out.encode('utf-8')

        self.assertLessEqual(len(encoded), MAX_BYTES_MISSION_COMMAND)  # <= 192 byte
        self.assertEqual(cmd.m_id, "M20261023-01")
        self.assertEqual(cmd.h_id, "SH01")
        self.assertEqual(cmd.cmd, "INSPECT")
        self.assertEqual(cmd.trays, [1, 2, 3, 4])
        self.assertEqual(cmd.crit_lvl, 1)
        self.assertEqual(cmd.ts, 1761200010)

        # Test roundtrip JSON
        decoded = MissionCommand.from_json(json_out)
        self.assertEqual(decoded, cmd)

    def test_mission_command_size_limit(self):
        """Uji bila command melebihi batas 192 byte harus menolak/raise error."""
        huge_trays = list(range(100))  # List panjang yang melebihi batas 192 byte
        cmd = MissionCommand(
            m_id="M20261023-01",
            h_id="SH01",
            cmd="INSPECT",
            trays=huge_trays,
            crit_lvl=1,
            ts=1761200010,
        )
        with self.assertRaises(PayloadSizeExceededError):
            cmd.validate()

    def test_mission_status_spec(self):
        """Validasi payload persis contoh kontrak: mission/status."""
        payload = {
            "m_id": "M20261023-01",
            "r_id": "RB01",
            "st": "IN_PROGRESS",
            "cur_tray": 3,
            "pct": 50,
            "bat": 84,
            "seq": 12,
            "ts": 1761200045,
        }
        status = MissionStatus.from_dict(payload)
        json_out = status.to_json()
        encoded = json_out.encode('utf-8')

        self.assertLessEqual(len(encoded), MAX_BYTES_MISSION_STATUS)  # <= 128 byte
        self.assertEqual(status.m_id, "M20261023-01")
        self.assertEqual(status.r_id, "RB01")
        self.assertEqual(status.st, "IN_PROGRESS")
        self.assertEqual(status.cur_tray, 3)
        self.assertEqual(status.pct, 50)
        self.assertEqual(status.bat, 84)
        self.assertEqual(status.seq, 12)
        self.assertEqual(status.ts, 1761200045)

        decoded = MissionStatus.from_json(json_out)
        self.assertEqual(decoded, status)

    def test_mission_status_validation(self):
        """Uji validasi persentase dan nilai di luar batas normal."""
        with self.assertRaises(ContractValidationError):
            # pct > 100 tidak valid
            MissionStatus(
                m_id="M01", r_id="RB01", st="OK", cur_tray=1, pct=105, bat=80, seq=1, ts=1761200000
            ).validate()

        with self.assertRaises(ContractValidationError):
            # bat < 0 tidak valid
            MissionStatus(
                m_id="M01", r_id="RB01", st="OK", cur_tray=1, pct=50, bat=-5, seq=1, ts=1761200000
            ).validate()

    def test_mission_result_spec(self):
        """Validasi payload persis contoh kontrak: mission/result."""
        payload = {
            "m_id": "M20261023-01",
            "h_id": "SH01",
            "tray_id": 3,
            "label": "BAGUS",
            "conf": 0.94,
            "anom": 0,
            "seq": 3,
            "ts": 1761200050,
        }
        result = MissionResult.from_dict(payload)
        json_out = result.to_json()
        encoded = json_out.encode('utf-8')

        self.assertLessEqual(len(encoded), MAX_BYTES_MISSION_RESULT)  # <= 256 byte
        self.assertEqual(result.m_id, "M20261023-01")
        self.assertEqual(result.h_id, "SH01")
        self.assertEqual(result.tray_id, 3)
        self.assertEqual(result.label, "BAGUS")
        self.assertEqual(result.conf, 0.94)
        self.assertEqual(result.anom, 0)
        self.assertEqual(result.seq, 3)
        self.assertEqual(result.ts, 1761200050)

        decoded = MissionResult.from_json(json_out)
        self.assertEqual(decoded, result)

    def test_mission_result_validation(self):
        """Uji anomali dan batas confidence."""
        with self.assertRaises(ContractValidationError):
            # anom selain 0 atau 1
            MissionResult(
                m_id="M01", h_id="SH01", tray_id=1, label="BAGUS", conf=0.9, anom=5, seq=1, ts=1761200000
            ).validate()

        with self.assertRaises(ContractValidationError):
            # conf > 1.0
            MissionResult(
                m_id="M01", h_id="SH01", tray_id=1, label="BAGUS", conf=1.5, anom=0, seq=1, ts=1761200000
            ).validate()


if __name__ == '__main__':
    unittest.main()
