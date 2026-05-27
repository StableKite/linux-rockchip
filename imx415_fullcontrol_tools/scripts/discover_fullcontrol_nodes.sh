#!/usr/bin/env bash
set -euo pipefail
LC_ALL=C
printf '===== SENSOR =====\n'
find /sys/bus/i2c/devices -maxdepth 2 -name debug_dphy_param -printf '%h\n' 2>/dev/null || true
printf '\n===== CSI2-DPHY BRIDGE =====\n'
find /sys -name debug_rx_state -printf '%h\n' 2>/dev/null || true
printf '\n===== SAMSUNG DCPHY =====\n'
find /sys -name debug_dcphy_state -printf '%h\n' 2>/dev/null || true
printf '\n===== CSI HOST =====\n'
find /sys -name debug_csihost_state -printf '%h\n' 2>/dev/null || true
printf '\n===== RKCIF =====\n'
find /sys -name debug_cif_state -printf '%h\n' 2>/dev/null || true
printf '\n===== CIF-to-ISP SDITF =====\n'
find /sys -name debug_sditf_mode -printf '%h\n' 2>/dev/null || true
