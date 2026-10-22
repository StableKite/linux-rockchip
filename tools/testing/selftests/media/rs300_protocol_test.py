#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Fail-closed host CRC / Mini2 source and Device Tree regression gate.

This is a *source consistency* test, not a hardware or I2C protocol test.
All validations execute under both normal Python and ``python3 -O``.

Usage:
    python3 tools/testing/selftests/media/rs300_protocol_test.py
    python3 -O tools/testing/selftests/media/rs300_protocol_test.py
"""

import argparse
from pathlib import Path
import re
import sys


class ProtocolError(ValueError):
    """A failed CRC vector or source/overlay invariant."""


def require(condition, description):
    if not condition:
        raise ProtocolError(description)


def crc_xmodem(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1) & 0xffff
    return crc


# These checks intentionally constrain the supported *fixed* physical mode.
# They are not a substitute for inspecting actual CSI-2 payloads on hardware.
SOURCE_DEFINES = {
    'RS300_WIDTH': '640U',
    'RS300_HEIGHT': '512U',
    'RS300_FPS': '60U',
    'RS300_LANES': '2U',
    'RS300_LINK_FREQ': '200000000LL',
    'RS300_BUS_CODE': 'MEDIA_BUS_FMT_YUYV8_2X8',
    'RS300_CMD_LEN': '18',
    'RS300_STREAM_CMD_LEN': '28',
}
SOURCE_COMMANDS = (
    'rs300_command(rs, 0x10, 0x10, 0x46, rate',
    'rs300_command(rs, 0x10, 0x02, 0x43, NULL',
    'rs300_command(rs, 0x10, 0x02, 0x41, &p',
    'rs300_put_crc(cmd, 16, rs300_crc(cmd, 16));',
)
OVERLAY_PROPERTIES = (
    'compatible = "infisense,mini2-640";',
    'reg = <0x3c>;',
    'vdd-supply = <&vcc5v0_sys>;',
    'data-lanes = <1 2>;',
    'link-frequencies = /bits/ 64 <200000000>;',
)


def check_vectors():
    require(crc_xmodem(b'') == 0x0000, 'CRC16/XMODEM empty vector mismatch')
    require(crc_xmodem(b'123456789') == 0x31c3,
            'CRC16/XMODEM standard test vector mismatch')
    fps60 = bytes.fromhex('10 10 46 00 01 03 3C') + bytes(9)
    require(len(fps60) == 16, 'Vendor 60 Hz command length mismatch')
    require(crc_xmodem(fps60) == 0x7c32,
            'CRC16/XMODEM vendor 60 Hz vector mismatch')


def check_source(driver, overlay):
    for key, value in SOURCE_DEFINES.items():
        pattern = rf'(?m)^\s*#define\s+{re.escape(key)}\s+{re.escape(value)}\s*$'
        require(re.search(pattern, driver) is not None,
                f'Driver constant {key} must be {value}')
    for code in SOURCE_COMMANDS:
        require(code in driver, f'Missing RS300 protocol command/code: {code}')
    for prop in OVERLAY_PROPERTIES:
        require(prop in overlay, f'Missing Mini2 overlay property: {prop}')


def read_source(path):
    try:
        return Path(path).read_text(encoding='utf-8')
    except (OSError, UnicodeError) as exc:
        raise ProtocolError(f'Cannot read input {path}: {exc}') from exc


def main(argv=None):
    root = Path(__file__).resolve().parents[4]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver', type=Path,
                        default=root / 'drivers/media/i2c/rs300.c')
    parser.add_argument('--overlay', type=Path,
                        default=root / 'arch/arm64/boot/dts/rockchip/overlay/orangepi-5-mini2-rs300.dts')
    options = parser.parse_args(argv)
    try:
        check_vectors()
        check_source(read_source(options.driver), read_source(options.overlay))
    except ProtocolError as exc:
        print(f'RS300_PROTOCOL_GATE=FAIL: {exc}', file=sys.stderr)
        return 1
    print('RS300_PROTOCOL_GATE=PASS: CRC16/XMODEM, fixed-mode source, command IDs, CSI overlay')
    return 0


if __name__ == '__main__':
    sys.exit(main())
