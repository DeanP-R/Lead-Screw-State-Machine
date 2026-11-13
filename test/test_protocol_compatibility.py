#!/usr/bin/env python3
"""Wire-level checks matching the existing Python monitor/logger contract."""

import struct
import unittest


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def packet(command: int, sequence: int, value: int, flags: int) -> bytes:
    body = bytes((command, sequence)) + struct.pack("<i", value) + bytes((flags,))
    return b"\xAA" + body + bytes((crc8(body), 0x55))


class ProtocolCompatibilityTests(unittest.TestCase):
    def test_signed_little_endian_frame(self):
        frame = packet(0x03, 42, -1234, 0x5A)
        self.assertEqual(len(frame), 10)
        self.assertEqual(frame[:3], bytes((0xAA, 0x03, 42)))
        self.assertEqual(struct.unpack("<i", frame[3:7])[0], -1234)
        self.assertEqual(frame[8], crc8(frame[1:8]))
        self.assertEqual(frame[9], 0x55)

    def test_logger_telemetry_response_ids(self):
        sequence = 17
        frames = (
            packet(0x86, sequence, 89, 0x80),
            packet(0x87, sequence, -321, 0x00),
            packet(0x88, sequence, 0x01B40000, 0x06),
        )
        self.assertEqual([frame[1] for frame in frames], [0x86, 0x87, 0x88])
        self.assertTrue(all(frame[2] == sequence for frame in frames))

    def test_invalid_distance_is_minus_one(self):
        frame = packet(0x86, 1, -1, 0)
        self.assertEqual(struct.unpack("<i", frame[3:7])[0], -1)
        self.assertFalse(frame[7] & 0x80)


if __name__ == "__main__":
    unittest.main()
