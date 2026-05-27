#!/usr/bin/env bash
set -euo pipefail
LC_ALL=C
TREE="${1:-$HOME/linux-rockchip}"
EXPECTED_HEAD="9391d4ba93e29a42f30f491a8e8d5ed8a409eb03"
TOOLS="$TREE/imx415_fullcontrol_tools"

[ -d "$TREE/.git" ] || { echo "ERROR: not a git tree: $TREE" >&2; exit 2; }
[ "$(git -C "$TREE" rev-parse HEAD)" = "$EXPECTED_HEAD" ] || {
  echo "ERROR: unexpected HEAD; expected $EXPECTED_HEAD" >&2
  git -C "$TREE" rev-parse HEAD >&2
  exit 2
}
[ -r "$TOOLS/MANIFEST.tsv" ] || { echo "ERROR: overlay manifest missing" >&2; exit 2; }

fail=0
while IFS=$'\t' read -r path base_sha modified_sha bytes; do
  [ "$path" = "path" ] && continue
  [ -z "$path" ] && continue
  [ -f "$TREE/$path" ] || { echo "MISSING: $path"; fail=1; continue; }
  actual="$(sha256sum "$TREE/$path" | awk '{print $1}')"
  if [ "$actual" != "$modified_sha" ]; then
    echo "HASH_MISMATCH: $path expected=$modified_sha actual=$actual"
    fail=1
  else
    echo "OK: $path"
  fi
done < <(awk 'found || /^path\t/{found=1; print}' "$TOOLS/MANIFEST.tsv")

[ "$fail" -eq 0 ] || exit 3

git -C "$TREE" diff --check

grep -q 'debug_dphy_param' "$TREE/drivers/media/i2c/imx415.c"
grep -q 'debug_rx_policy' "$TREE/drivers/phy/rockchip/phy-rockchip-csi2-dphy.c"
grep -q 'debug_dcphy_snapshot' "$TREE/drivers/phy/rockchip/phy-rockchip-samsung-dcphy.c"
grep -q 'debug_csihost_program' "$TREE/drivers/media/platform/rockchip/cif/mipi-csi2.c"
grep -q 'debug_cif_reg' "$TREE/drivers/media/platform/rockchip/cif/dev.c"
grep -q 'debug_sditf_mode' "$TREE/drivers/media/platform/rockchip/cif/subdev-itf.c"

echo
echo 'OVERLAY_VALIDATE=PASS'
echo 'NOTE=Source hashes and diff whitespace validated. This does not compile the kernel.'
git -C "$TREE" diff --stat -- \
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
