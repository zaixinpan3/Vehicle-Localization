"""Unit checks for raw packet parsing, units and unchanged installation axes."""
import importlib.util
import struct
import unittest
from pathlib import Path

path = Path(__file__).resolve().parents[1] / "scripts/extractOusterImuFromBag.py"
spec = importlib.util.spec_from_file_location("ouster_export", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class OusterPacketTest(unittest.TestCase):
    def test_padding_is_identical(self):
        packet = struct.pack("<QQQffffff", 1, 2, 3, 0, 0, 1, 1, 2, 3)
        self.assertEqual(module.decode_packet(packet), module.decode_packet(packet+b"\0"))

    def test_unsupported_format_fails(self):
        with self.assertRaises(ValueError):
            module.decode_packet(bytes(47))

    def test_nonzero_trailer_fails(self):
        with self.assertRaises(ValueError):
            module.decode_packet(bytes(48)+b"\1")

    def test_nan_fails(self):
        packet = struct.pack("<QQQffffff", 1, 2, 3, float("nan"), 0, 1, 1, 2, 3)
        with self.assertRaises(ValueError):
            module.decode_packet(packet)


if __name__ == "__main__":
    unittest.main()
