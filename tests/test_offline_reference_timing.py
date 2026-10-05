"""Verify native scan association and padded PointCloud2 field extraction."""
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from prepareOfflineReferenceBag import lidar_columns, match_scan, point_view


class NativeTimingTest(unittest.TestCase):
    def test_packet_padding_and_frame_id(self):
        raw = bytearray(12609)
        for i in range(16):
            struct.pack_into("<QHH", raw, i * 788, 1000 + i, 32 + i, 17)
        self.assertEqual(lidar_columns(raw)[-1], (1015, 47, 17))
        raw[-1] = 1
        with self.assertRaises(ValueError):
            lidar_columns(raw)

    def test_partial_frame_exact_clock_beats_adjacent_frame(self):
        relative = np.arange(1024, dtype=np.uint32) * 100000
        exact = {i: 10**12 + int(relative[i]) for i in range(752, 1024)}
        wrong = {i: 10**12 + 10**8 + int(relative[i]) + i % 7 for i in range(64)}
        self.assertEqual(match_scan(relative, {81: exact, 82: wrong}), (81, 10**12, 272))

    def test_ambiguous_or_missing_frame_rejected(self):
        relative = np.arange(32, dtype=np.uint32)
        a = {i: 1000 + int(relative[i]) for i in range(32)}
        with self.assertRaises(ValueError):
            match_scan(relative, {1: a, 2: a})
        with self.assertRaises(ValueError):
            match_scan(relative, {1: {0: 1000}})

    def test_row_padding_and_big_endian(self):
        data = bytearray(32)
        for off, value in [(4, 7), (12, 9), (24, 11)]:
            struct.pack_into(">I", data, off, value)
        message = SimpleNamespace(fields=[SimpleNamespace(name="t", offset=4, datatype=6)],
                                  is_bigendian=True, point_step=8, row_step=20,
                                  height=2, width=2, data=data + bytearray(8))
        np.testing.assert_array_equal(point_view(message, "t"), [[7, 9], [11, 0]])


if __name__ == "__main__":
    unittest.main()
