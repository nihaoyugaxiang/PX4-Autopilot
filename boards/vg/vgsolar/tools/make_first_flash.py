#!/usr/bin/env python3
"""Package the VGSolar bootloader and bench application for first USB-DFU programming."""
from pathlib import Path
import argparse
import base64
import hashlib
import json
import re
import struct
import subprocess
import zlib


def record(address, kind, data=b''):
    body = bytes([len(data)]) + address.to_bytes(2, 'big') + bytes([kind]) + data
    return ':' + (body + bytes([(-sum(body)) & 255])).hex().upper()


def write_hex(path, segments):
    lines = []
    upper = None
    for start, data in segments:
        for offset in range(0, len(data), 16):
            address = start + offset
            high = address >> 16
            if high != upper:
                lines.append(record(0, 4, high.to_bytes(2, 'big')))
                upper = high
            chunk = data[offset:offset + 16]
            assert (address & 65535) + len(chunk) <= 65536
            lines.append(record(address & 65535, 0, chunk))
    lines.append(record(0, 1))
    path.write_text('\n'.join(lines) + '\n', encoding='ascii')


def elf_at_vaddr(path, address, length):
    data = path.read_bytes()
    assert data[:6] == b'\x7fELF\x01\x01'
    offset = struct.unpack_from('<I', data, 28)[0]
    size, count = struct.unpack_from('<HH', data, 42)
    for i in range(count):
        kind, file_offset, vaddr, _, filesz, _, _, _ = struct.unpack_from('<IIIIIIII', data, offset + i * size)
        if kind == 1 and vaddr <= address and address + length <= vaddr + filesz:
            return data[file_offset + address - vaddr:file_offset + address - vaddr + length]
    raise ValueError('ELF address is not present in a file-backed LOAD segment')


def main():
    root = Path(__file__).resolve().parents[4]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=root / 'build/vgsolar_first_flash')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    app_dir = root / 'build/vg_vgsolar_default'
    bl_dir = root / 'build/vg_vgsolar_bootloader'
    app = (app_dir / 'vg_vgsolar_default.bin').read_bytes()
    bl = (bl_dir / 'vg_vgsolar_bootloader.bin').read_bytes()
    app_package = json.loads((app_dir / 'vg_vgsolar_default.px4').read_text())
    assert zlib.decompress(base64.b64decode(app_package['image'])) == app
    assert len(bl) < 0x20000
    assert len(app) <= 0x1C0000
    assert app_package['board_id'] == 0
    for payload, base, limit in [(bl, 0x08000000, 0x08020000), (app, 0x08020000, 0x081E0000)]:
        stack, reset = struct.unpack_from('<II', payload)
        assert 0x24000000 < stack <= 0x24080000
        assert base <= (reset & ~1) < limit and reset & 1
    bl_elf = bl_dir / 'vg_vgsolar_bootloader.elf'
    symbols = subprocess.check_output(['arm-none-eabi-nm', str(bl_elf)], text=True)
    info_address = int(re.search(r'^([0-9a-fA-F]+) [A-Za-z] board_info$', symbols, re.M).group(1), 16)
    board_id, board_rev, _, cpu_mhz = struct.unpack('<IIII', elf_at_vaddr(bl_elf, info_address, 16))
    assert board_id == app_package['board_id'] and board_rev == 0
    hex_path = args.output / 'VGSolar_Rover_B0_首次烧写.hex'
    write_hex(hex_path, [(0x08000000, bl), (0x08020000, app)])
    # Independent GNU parser/objcopy verifies address interpretation and exact bytes.
    merged_bin = args.output / 'first_flash_roundtrip.bin'
    subprocess.run(['arm-none-eabi-objcopy', '-I', 'ihex', '-O', 'binary', '--gap-fill', '0xFF', str(hex_path), str(merged_bin)], check=True)
    merged = merged_bin.read_bytes()
    assert merged[:len(bl)] == bl
    assert merged[0x20000:0x20000 + len(app)] == app
    assert all(value == 255 for value in merged[len(bl):0x20000])
    merged_bin.unlink()
    names = subprocess.check_output(['arm-none-eabi-nm', '--defined-only', str(app_dir / 'vg_vgsolar_default.elf')], text=True)
    for needed in ['rover_differential_main', 'icm42688p_main', 'mavlink_main', 'cdcacm_autostart_main']:
        assert re.search(r'\b' + needed + r'$', names, re.M), needed
    for absent in ['pwm_out_main', 'dshot_main', 'px4io_main', 'heater_main', 'tone_alarm_main']:
        assert not re.search(r'\b' + absent + r'$', names, re.M), absent
    manifest = {
        'purpose': 'First programming of a blank STM32H743VI VGSolar board through factory ROM USB DFU',
        'board_id': board_id, 'bootloader_board_id_verified_in_elf': True,
        'bootloader_address': '0x08000000', 'bootloader_size': len(bl),
        'application_address': '0x08020000', 'application_size': len(app),
        'cpu_mhz': cpu_mhz, 'chip': 'STM32H743VI', 'hse_hz': 16000000,
        'hex_sha256': hashlib.sha256(hex_path.read_bytes()).hexdigest(),
        'bootloader_sha256': hashlib.sha256(bl).hexdigest(),
        'application_bin_sha256': hashlib.sha256(app).hexdigest(),
        'application_px4_sha256': hashlib.sha256((app_dir / 'vg_vgsolar_default.px4').read_bytes()).hexdigest(),
        'verified': ['Both ARM builds passed', 'Bootloader/application IDs match', 'Both reset vectors and SRAM stack ranges checked', 'HEX GNU objcopy roundtrip matches both binaries at their correct addresses', 'Rover/USB modules present and actuator drivers absent'],
        'hardware_tested': False,
    }
    (args.output / '首次烧写核对记录.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
