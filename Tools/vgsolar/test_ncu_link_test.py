#!/usr/bin/env python3
"""Host-only tests for the VGSolar NCU serial helper."""
import unittest
import struct

from test_ncu_link import FrameParser, decode_status, frame


class NcuLinkTests(unittest.TestCase):
    def test_frame_checksum_and_layout(self):
        packet = frame(0xAA, 0x02, b"\x01\x00\x00\x00\x00\x00")
        self.assertEqual(packet[:5], b"\xA5\x5A\xAA\x02\x06")
        self.assertEqual(packet[-1], 0xFF)
        self.assertEqual(packet[-2], sum(packet[2:-2]) & 0xFF)

    def test_decode_apm_34_byte_status(self):
        values = [87, 123456789, -987654321, 35999, -12, 13, -14, -123, 234, 0, 0, 0, 2,
                  314, 314, 65535, 65535]
        body = bytearray(34)
        struct.pack_into("<BiiHhhhhhBBHBHHHH", body, 0, *values)
        packet = frame(0xBB, 0x01, body)
        decoded = decode_status(packet)
        self.assertEqual(decoded["battery_pct"], 87)
        self.assertEqual(decoded["longitude_e7"], 123456789)
        self.assertEqual(decoded["latitude_e7"], -987654321)
        self.assertEqual(decoded["heading_cd"], 35999)
        self.assertEqual(decoded["left_out_cm"], 314)
        self.assertEqual(decoded["right_out_cm"], 65535)

    def test_reject_wrong_status_size(self):
        with self.assertRaises(ValueError):
            decode_status(frame(0xBB, 0x01, bytes(15)))

    def test_stream_split_and_bad_frame_resynchronization(self):
        expected = frame(0xBB, 0x01, bytes(34))
        invalid = bytearray(expected)
        invalid[-2] ^= 0x01
        parser = FrameParser()
        self.assertEqual(parser.feed(b"noise" + invalid + expected[:8]), [])
        self.assertEqual(parser.feed(expected[8:]), [expected])

    def test_stream_concatenated_frames(self):
        first = frame(0xBB, 0x02, b"\x02\x02")
        second = frame(0xBB, 0x01, bytes(34))
        parser = FrameParser()
        self.assertEqual(parser.feed(first + second), [first, second])


if __name__ == "__main__":
    unittest.main()
