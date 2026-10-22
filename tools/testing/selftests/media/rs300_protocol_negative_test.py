#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Negative controls for the host-side RS300 source consistency gate."""

import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).with_name('rs300_protocol_test.py').resolve()
SPEC = importlib.util.spec_from_file_location('rs300_protocol_gate', SCRIPT)
GATE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GATE)


def reference_sources():
    driver = '\n'.join(f'#define {k} {v}' for k, v in GATE.SOURCE_DEFINES.items())
    driver += '\n' + '\n'.join(GATE.SOURCE_COMMANDS) + '\n'
    overlay = '\n'.join(GATE.OVERLAY_PROPERTIES) + '\n'
    return driver, overlay


class GateTests(unittest.TestCase):
    def test_crc_reference_vectors(self):
        GATE.check_vectors()
        self.assertEqual(GATE.crc_xmodem(b'123456789'), 0x31c3)

    def test_good_reference_source(self):
        GATE.check_source(*reference_sources())

    def test_reject_driver_definition_change(self):
        driver, dts = reference_sources()
        for old, new in (('RS300_WIDTH 640U', 'RS300_WIDTH 320U'),
                         ('RS300_FPS 60U', 'RS300_FPS 30U'),
                         ('RS300_LINK_FREQ 200000000LL', 'RS300_LINK_FREQ 0LL'),
                         ('RS300_BUS_CODE MEDIA_BUS_FMT_YUYV8_2X8',
                          'RS300_BUS_CODE MEDIA_BUS_FMT_UYVY8_2X8')):
            with self.subTest(old=old):
                with self.assertRaises(GATE.ProtocolError):
                    GATE.check_source(driver.replace(old, new), dts)

    def test_reject_missing_command(self):
        driver, dts = reference_sources()
        with self.assertRaises(GATE.ProtocolError):
            GATE.check_source(driver.replace(GATE.SOURCE_COMMANDS[2], ''), dts)

    def test_reject_broken_overlay(self):
        driver, dts = reference_sources()
        for prop in GATE.OVERLAY_PROPERTIES:
            with self.subTest(prop=prop):
                with self.assertRaises(GATE.ProtocolError):
                    GATE.check_source(driver, dts.replace(prop, ''))

    def test_cli_fail_closed_optimized(self):
        drv, overlay = reference_sources()
        with tempfile.TemporaryDirectory() as temp:
            a, b = Path(temp) / 'driver.c', Path(temp) / 'overlay.dts'
            a.write_text(drv)
            b.write_text(overlay)
            for optimize in (False, True):
                prefix = [sys.executable] + (['-O'] if optimize else [])
                with self.subTest(optimized=optimize, mode='valid'):
                    proc = subprocess.run(prefix + [str(SCRIPT), '--driver', str(a),
                                                   '--overlay', str(b)], capture_output=True, text=True)
                    self.assertEqual(proc.returncode, 0, proc.stderr)
                    self.assertIn('RS300_PROTOCOL_GATE=PASS', proc.stdout)
                b.write_text(overlay.replace('link-frequencies = /bits/ 64 <200000000>;', ''))
                with self.subTest(optimized=optimize, mode='bad-link-freq'):
                    proc = subprocess.run(prefix + [str(SCRIPT), '--driver', str(a),
                                                   '--overlay', str(b)], capture_output=True, text=True)
                    self.assertNotEqual(proc.returncode, 0)
                    self.assertIn('RS300_PROTOCOL_GATE=FAIL', proc.stderr)
                b.write_text(overlay)
                with self.subTest(optimized=optimize, mode='missing-driver'):
                    proc = subprocess.run(prefix + [str(SCRIPT), '--driver', str(a) + '.missing',
                                                   '--overlay', str(b)], capture_output=True, text=True)
                    self.assertNotEqual(proc.returncode, 0)
                    self.assertIn('RS300_PROTOCOL_GATE=FAIL', proc.stderr)
                with self.subTest(optimized=optimize, mode='missing-dts'):
                    proc = subprocess.run(prefix + [str(SCRIPT), '--driver', str(a),
                                                   '--overlay', str(b) + '.missing'], capture_output=True, text=True)
                    self.assertNotEqual(proc.returncode, 0)
                    self.assertIn('RS300_PROTOCOL_GATE=FAIL', proc.stderr)


if __name__ == '__main__':
    unittest.main()
