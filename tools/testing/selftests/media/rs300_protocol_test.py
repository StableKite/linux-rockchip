#!/usr/bin/env python3
"""Pure, host-side RS300 packet/CRC regression tests against vendor examples."""
from pathlib import Path
import re


def crc_xmodem(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)) & 0xffff
    return crc


fps60 = bytes.fromhex("10 10 46 00 01 03 3C") + bytes(9)
assert len(fps60) == 16
assert crc_xmodem(fps60) == 0x7C32, "CRC must reproduce vendor 60 Hz command"

src = Path(__file__).resolve().parents[4] / "drivers/media/i2c/rs300.c"
text = src.read_text()
for required in (
    "RS300_WIDTH                640U",
    "RS300_HEIGHT               512U",
    "RS300_FPS                  60U",
    "RS300_LANES                2U",
    "RS300_LINK_FREQ            200000000LL",
    "rs300_command(rs, 0x10, 0x10, 0x46, rate",
    "rs300_command(rs, 0x10, 0x02, 0x43, NULL",
    "rs300_command(rs, 0x10, 0x02, 0x41, &p",
):
    assert required in text, f"Source missing expected protocol configuration: {required}"

mode = Path(__file__).resolve().parents[4] / "arch/arm64/boot/dts/rockchip/overlay/orangepi-5-mini2-rs300.dts"
dts = mode.read_text()
assert "link-frequencies = /bits/ 64 <200000000>;" in dts
assert "vdd-supply = <&vcc5v0_sys>;" in dts
assert "<1 2>" in dts
print("PASS: vendor CRC16/XMODEM, module mode, command IDs, native MIPI overlay")
