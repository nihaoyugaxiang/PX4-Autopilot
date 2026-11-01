#!/usr/bin/env python3
"""RK3588/PC NCU client. Default command mode prints bytes; --send opens serial.
Monitor is read-only. Mode selection/arming is performed separately through PX4.
"""
import argparse
import json
import math
import struct
import time

LENGTHS = {1: 34, 2: 2, 3: 7, 4: 8}
STATUS_KEYS = ('battery_pct','longitude_e7','latitude_e7','heading_cd','velocity_cms',
 'left_track_cms','right_track_cms','roll_cd','pitch_cd','control_mode','motion_state',
 'fault_code','gps_status','left_out_cm','left_in_cm','right_in_cm','right_out_cm')


def frame(command, body):
    data = bytes((0xAA, command, len(body))) + body
    return b'\xa5\x5a' + data + bytes((sum(data) & 255, 255))


def bounded(value, low, high):
    if not low <= value <= high:
        raise ValueError(f'{value} outside [{low}, {high}]')
    return value


def speed(velocity, yaw, heading=False):
    bounded(velocity, -500, 500)
    bounded(yaw, -32768 if heading else -18000, 32767 if heading else 18000)
    return frame(1, struct.pack('<Bhh', 1 if heading else 2, velocity, yaw))


def turn(angle, rate, left=True, moving=False):
    return frame(2, struct.pack('<BBHH', 2 if moving else 1, 1 if left else 2,
        bounded(angle, 1, 36000), bounded(rate, 0, 18000)))


def parameter(index, value=None):
    bounded(index, 0, 65535)
    if value is None:
        return frame(4, struct.pack('<H', index))
    if index not in (0x101, 0x102, 0x103, 0x104):
        raise ValueError('only brush runtime indexes 0x101..0x104 are supported')
    bounded(value, 0, 100 if index == 0x104 else 1)
    return frame(3, struct.pack('<HBI', index, 3, value))


def system(command):
    return frame(5, bytes((bounded(command, 1, 3),)))


def navigation(mode, x=0, y=0, velocity=0, radius=0, arrival=None):
    bounded(mode, 1, 4)
    if mode == 1:
        bounded(x, -1800000000, 1800000000)
        bounded(y, -900000000, 900000000)
    else:
        bounded(x, -2147483648, 2147483647)
        bounded(y, -2147483648, 2147483647)
    bounded(velocity, -500, 500)
    bounded(radius, 0, 50000)
    yaw = 65535 if arrival is None else bounded(arrival, 0, 36000)
    return frame(6, struct.pack('<BiiHHh', mode, x, y, yaw, radius, velocity))


class Parser:
    """Bounded resynchronizing FCU parser with partial-frame expiration."""
    def __init__(self, timeout=.2):
        self.data = bytearray()
        self.started = None
        self.timeout = timeout
        self.bad_frames = 0

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        if self.started is not None and now - self.started > self.timeout:
            self.data.clear()
            self.started = None
            self.bad_frames += 1
        packets = []
        for byte in data:
            self.data.append(byte)
            while self.data:
                if self.data[0] != 0xA5:
                    del self.data[0]
                    continue
                if len(self.data) >= 2 and self.data[1] != 0x5A:
                    del self.data[0]
                    continue
                if len(self.data) < 5:
                    break
                if self.data[2] != 0xBB or LENGTHS.get(self.data[3]) != self.data[4]:
                    self.bad_frames += 1
                    del self.data[0]
                    continue
                length = 7 + self.data[4]
                if len(self.data) < length:
                    break
                packet = bytes(self.data[:length])
                if packet[-1] != 255 or packet[-2] != (sum(packet[2:-2]) & 255):
                    self.bad_frames += 1
                    del self.data[0]
                    continue
                del self.data[:length]
                packets.append(decode(packet))
            self.started = now if self.data and self.started is None else self.started if self.data else None
        return packets


def decode(packet):
    kind = packet[3]
    body = packet[5:-2]
    if kind == 1:
        value = dict(zip(STATUS_KEYS, struct.unpack('<BiiHhhhhhBBHBHHHH', body)))
        value['type'] = 'status'
        return value
    if kind == 2:
        command, status = struct.unpack('<BB', body)
        return {'type':'ack', 'command':command, 'status':status,
                'meaning':'received only' if command in (2, 5) else 'accepted' if status == 1 else 'rejected'}
    if kind == 3:
        index, data_type, value = struct.unpack('<HBI', body)
        return {'type':'parameter', 'index':index, 'data_type':data_type, 'value':value}
    state, coordinate, distance, heading = struct.unpack('<BBIh', body)
    return {'type':'navigation','state':state,'coordinate':coordinate,'distance_cm':distance,'heading_error_cd':heading}


def write_all(port, packet):
    offset = 0
    deadline = time.monotonic() + 1.
    while offset < len(packet):
        if time.monotonic() >= deadline:
            raise TimeoutError('serial write did not complete')
        written = port.write(packet[offset:])
        if written is None or written <= 0:
            raise OSError('serial write failed')
        offset += written


def receive(port, parser):
    return parser.feed(port.read(256))


def run(port, packet, duration, repeat_speed=False, monitor=False, emit=print):
    parser = Parser()
    deadline = time.monotonic() + duration
    next_send = 0.
    try:
        if not monitor and not repeat_speed:
            write_all(port, packet)
        while time.monotonic() < deadline:
            if repeat_speed and time.monotonic() >= next_send:
                write_all(port, packet)
                next_send = time.monotonic() + .1
            for message in receive(port, parser):
                emit(json.dumps(message, ensure_ascii=False))
    finally:
        if repeat_speed:
            # Best effort explicit stop; FCU's 500 ms watchdog is the independent fallback.
            write_all(port, speed(0, 0))
    return parser.bad_frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='RK3588 /dev/ttySx or Windows COMx')
    parser.add_argument('--send', action='store_true', help='transmit command; otherwise print frame only')
    parser.add_argument('--seconds', type=float, default=3.)
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('monitor')
    for name in ('estop','clear','reboot','cancel'):
        sub.add_parser(name)
    sp = sub.add_parser('speed'); sp.add_argument('velocity', type=int); sp.add_argument('yaw', type=int); sp.add_argument('--heading', action='store_true')
    tp = sub.add_parser('turn'); tp.add_argument('angle', type=int); tp.add_argument('rate', type=int); tp.add_argument('--right', action='store_true'); tp.add_argument('--moving', action='store_true')
    pp = sub.add_parser('param'); pp.add_argument('index', type=lambda x:int(x,0)); pp.add_argument('value', type=int, nargs='?')
    np = sub.add_parser('nav'); np.add_argument('mode', choices=['gps','ned','body']); np.add_argument('x', type=int); np.add_argument('y', type=int); np.add_argument('--velocity', type=int, default=0); np.add_argument('--radius', type=int, default=0); np.add_argument('--arrival', type=int)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 60:
        parser.error('--seconds must be finite and in (0,60]')
    try:
        if args.action in ('estop','clear','reboot'):
            packet = system({'estop':1,'clear':2,'reboot':3}[args.action])
        elif args.action == 'cancel': packet = navigation(4)
        elif args.action == 'speed': packet = speed(args.velocity,args.yaw,args.heading)
        elif args.action == 'turn': packet = turn(args.angle,args.rate,not args.right,args.moving)
        elif args.action == 'param': packet = parameter(args.index,args.value)
        elif args.action == 'nav': packet = navigation({'gps':1,'ned':2,'body':3}[args.mode],args.x,args.y,args.velocity,args.radius,args.arrival)
        else: packet = b''
    except (ValueError, struct.error) as exc:
        parser.error(str(exc))
    if args.action != 'monitor' and not args.send:
        print(packet.hex(' ')); return
    if not args.port:
        parser.error('--port required for monitoring/transmission')
    try:
        import serial
    except ImportError:
        parser.error('install pyserial: python3 -m pip install pyserial')
    with serial.Serial(args.port,115200,timeout=.02,write_timeout=1) as port:
        try:
            run(port,packet,args.seconds,args.action=='speed',args.action=='monitor')
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
