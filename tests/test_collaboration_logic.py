"""Unit tests untuk logika Collaboration Agent dan simulasi alur misi."""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../rice_vision')))

from rice_vision.contracts import (
    ANOMALY_DEFECT,
    ANOMALY_NORMAL,
    LABEL_BAGUS,
    LABEL_TIDAK_BAGUS,
    STATUS_COMPLETED,
    STATUS_IN_PROGRESS,
    MissionCommand,
    MissionResult,
    MissionStatus,
)


class TestCollaborationLogic(unittest.TestCase):

    def test_mission_lifecycle_progression(self):
        """Simulasi eksekusi misi dari 4 tray [1, 2, 3, 4] seperti di dokumen/gambar."""
        cmd_raw = {
            "m_id": "M20261023-01",
            "h_id": "SH01",
            "cmd": "INSPECT",
            "trays": [1, 2, 3, 4],
            "crit_lvl": 1,
            "ts": 1761200010,
        }
        cmd = MissionCommand.from_dict(cmd_raw)

        total_trays = len(cmd.trays)
        status_seq = 0
        result_seq = 0
        results = []

        # Simulasi tray inspections
        for idx, tray_id in enumerate(cmd.trays):
            status_seq += 1
            pct = int((idx / total_trays) * 100)
            status = MissionStatus(
                m_id=cmd.m_id,
                r_id="RB01",
                st=STATUS_IN_PROGRESS,
                cur_tray=tray_id,
                pct=pct,
                bat=84 - idx,
                seq=status_seq,
                ts=cmd.ts + 10 + (idx * 15),
            )
            status.validate()

            # Hasil tray
            result_seq += 1
            is_anomaly = (tray_id == 4)  # Misalnya tray 4 terdeteksi rusak
            label = LABEL_TIDAK_BAGUS if is_anomaly else LABEL_BAGUS
            anom = ANOMALY_DEFECT if is_anomaly else ANOMALY_NORMAL
            conf = 0.88 if is_anomaly else 0.94

            res = MissionResult(
                m_id=cmd.m_id,
                h_id=cmd.h_id,
                tray_id=tray_id,
                label=label,
                conf=conf,
                anom=anom,
                seq=result_seq,
                ts=cmd.ts + 10 + (idx * 15) + 5,
            )
            res.validate()
            results.append(res)

        # Status akhir setelah semua tray selesai
        status_seq += 1
        final_status = MissionStatus(
            m_id=cmd.m_id,
            r_id="RB01",
            st=STATUS_COMPLETED,
            cur_tray=cmd.trays[-1],
            pct=100,
            bat=80,
            seq=status_seq,
            ts=cmd.ts + 70,
        )
        final_status.validate()

        self.assertEqual(final_status.st, STATUS_COMPLETED)
        self.assertEqual(final_status.pct, 100)
        self.assertEqual(len(results), 4)
        self.assertEqual(results[2].tray_id, 3)
        self.assertEqual(results[2].label, "BAGUS")
        self.assertEqual(results[2].conf, 0.94)
        self.assertEqual(results[2].anom, 0)


if __name__ == '__main__':
    unittest.main()
