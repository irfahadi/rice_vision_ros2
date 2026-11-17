"""Unit tests untuk pengkodean payload dan verifikasi batasan byte Sink Node Bridge."""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../rice_vision')))

from rice_vision.contracts import (
    MAX_BYTES_MISSION_COMMAND,
    MAX_BYTES_MISSION_RESULT,
    MAX_BYTES_MISSION_STATUS,
    MissionCommand,
    MissionResult,
    MissionStatus,
    PayloadSizeExceededError,
)


class TestSinkBridgeEncoding(unittest.TestCase):

    def test_command_byte_limits_and_formats(self):
        # 1. Kasus normal
        cmd = MissionCommand(
            m_id="M20261023-01",
            h_id="SH01",
            cmd="INSPECT",
            trays=[1, 2, 3, 4, 5, 6, 7, 8],
            crit_lvl=2,
            ts=1761200010,
        )
        b = cmd.to_bytes()
        self.assertLessEqual(len(b), MAX_BYTES_MISSION_COMMAND)

        # 2. Kasus payload mendekati batas
        trays_15 = list(range(1, 16))
        cmd_15 = MissionCommand(
            m_id="M20261023-9999",
            h_id="SH01_SECTOR_A",
            cmd="INSPECT",
            trays=trays_15,
            crit_lvl=1,
            ts=1761200010,
        )
        b_15 = cmd_15.to_bytes()
        self.assertLessEqual(len(b_15), MAX_BYTES_MISSION_COMMAND)

    def test_status_byte_limits_and_formats(self):
        # Status maksimum yang realistis
        status = MissionStatus(
            m_id="M20261023-9999",
            r_id="RB01_INSPECT",
            st="IN_PROGRESS",
            cur_tray=99,
            pct=100,
            bat=100,
            seq=99999,
            ts=1761200045,
        )
        b = status.to_bytes()
        self.assertLessEqual(len(b), MAX_BYTES_MISSION_STATUS)

    def test_result_byte_limits_and_formats(self):
        # Result maksimum
        res = MissionResult(
            m_id="M20261023-9999",
            h_id="SH01_SECTOR_A",
            tray_id=99,
            label="TIDAK BAGUS",
            conf=0.9999,
            anom=1,
            seq=99999,
            ts=1761200050,
        )
        b = res.to_bytes()
        self.assertLessEqual(len(b), MAX_BYTES_MISSION_RESULT)
        # Pastikan conf dibulatkan 2 desimal
        d = res.to_dict()
        self.assertEqual(d["conf"], 1.0)


if __name__ == '__main__':
    unittest.main()

