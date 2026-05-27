#!/usr/bin/env bash
set -euo pipefail
TREE="${1:-$HOME/linux-rockchip}"
cd "$TREE"
echo 'This will discard changes only in IMX415 fullcontrol modified tracked source files.'
printf 'Type RESTORE to continue: '
read -r answer
[ "$answer" = RESTORE ] || { echo 'Cancelled.'; exit 1; }
git restore -- \
  drivers/media/i2c/imx415.c \
  drivers/media/platform/rockchip/cif/dev.c \
  drivers/media/platform/rockchip/cif/dev.h \
  drivers/media/platform/rockchip/cif/mipi-csi2.c \
  drivers/media/platform/rockchip/cif/mipi-csi2.h \
  drivers/media/platform/rockchip/cif/subdev-itf.c \
  drivers/media/platform/rockchip/cif/subdev-itf.h \
  drivers/phy/rockchip/phy-rockchip-csi2-dphy-common.h \
  drivers/phy/rockchip/phy-rockchip-csi2-dphy.c \
  drivers/phy/rockchip/phy-rockchip-samsung-dcphy.c \
  drivers/phy/rockchip/phy-rockchip-samsung-dcphy.h
echo 'Tracked fullcontrol source changes restored from HEAD.'
