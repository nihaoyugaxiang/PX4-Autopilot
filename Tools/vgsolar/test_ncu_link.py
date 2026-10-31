#!/usr/bin/env python3
"""PC-side NCU simulator and validator for the VGSolar read-only UART stage."""
import argparse
import struct
import time

HEADER = b"\xA5\x5A"
TAIL = 0xFF


def frame(source, command, body):
    data = bytes((source, command, len(body))) + body
    return HEADER + data + bytes((sum(data) & 0xFF, TAIL))


class FrameParser:
    def __init__(self):
        self.data = bytearray()
        self.ready = []

    def feed(self, chunk):
        packets = []
        for byte in chunk:
            self.data.append(byte)
            while self.data:
                if self.data[0] != HEADER[0]:
                    del self.data[0]
                    continue
                if len(self.data) < 2:
                    break
                if self.data[1] != HEADER[1]:
                    del self.data[0]
                    continue
                if len(self.data) < 5:
                    break
                if self.data[4] > 34:
                    del self.data[0]
                    continue
                total = 7 + self.data[4]
                if len(self.data) < total:
                    break
                packet = bytes(self.data[:total])
                if packet[-1] == TAIL and packet[-2] == (sum(packet[2:-2]) & 0xFF):
                    packets.append(packet)
                    del self.data[:total]
                else:
                    del self.data[0]
        self.ready.extend(packets)
        return packets


def read_frame(port, timeout_s, frame_parser):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if frame_parser.ready:
            return frame_parser.ready.pop(0)
        chunk = port.read(64)
        frame_parser.feed(chunk)
    return None


def decode_status(packet):
    if packet[:5] != HEADER + bytes((0xBB, 0x01, 34)) or len(packet) != 41:
        raise ValueError(f"expected APM 34-byte status, received len={len(packet)} header={packet[:5].hex(' ')}")
    body = packet[5:39]
    base = struct.unpack_from("<BiiHhhhhhBBHBHHHH", body)
    keys = ("battery_pct", "longitude_e7", "latitude_e7", "heading_cd", "velocity_cms",
            "left_track_cms", "right_track_cms", "roll_cd", "pitch_cd", "control_mode",
            "motion_state", "fault_code", "gps_status", "left_out_cm", "left_in_cm", "right_in_cm", "right_out_cm")
    return dict(zip(keys, base))


def main():
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Install pyserial in the active environment: python -m pip install pyserial") from exc

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="COM port on Windows or /dev/ttyUSBx on Linux")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--inject", action="store_true", help="send valid, bad-checksum, fragmented and concatenated frames")
    args = parser.parse_args()
    deadline = time.monotonic() + args.seconds
    with serial.Serial(args.port, args.baud, timeout=0.05) as port:
        frame_parser = FrameParser()
        if args.inject:
            speed = frame(0xAA, 0x01, b"\x02\x00\x00\x00\x00")
            port.write(speed[:3])
            time.sleep(0.25)
            turn = frame(0xAA, 0x02, b"\x01\x00\x00\x00\x00\x00")
            port.write(speed[3:] + speed + turn)
            invalid = bytearray(speed)
            invalid[-2] ^= 0x80
            port.write(invalid)
            print("sent a timed-out fragment, concatenated speed/turn probes, and one bad checksum")
        count = 0
        while time.monotonic() < deadline:
            packet = read_frame(port, min(1.0, deadline - time.monotonic()), frame_parser)
            if packet is None:
                continue
            if packet[2:4] == b"\xBB\x01":
                count += 1
                values = decode_status(packet)
                print(f"{time.strftime('%H:%M:%S')} frame={count} {values}")
            elif packet[2:4] == b"\xBB\x02":
                print(f"rejected command={packet[5]} status={packet[6]} (expected for read-only stage)")
            else:
                print(f"other FCU frame cmd=0x{packet[3]:02X} len={packet[4]}")
        print(f"received {count} status frames in {args.seconds:.1f}s; target is about 10 Hz")


if __name__ == "__main__":
    main()
