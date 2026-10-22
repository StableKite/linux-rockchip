# IMX415 on Orange Pi 5 Max: select Camera0, Camera1 or Camera2

These three full DTBs configure **one four-lane IMX415 camera on the selected
connector**. Select the corresponding DTB when moving the camera. They do not
auto-detect the connector, enable three cameras together, or support hot plugging.
The existing Rockchip IMX415 driver is reused without changes.

| Connector | DTB suffix | I2C | Clock/pinmux | Receiver | CSI host | CIF | ISP |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Camera0 | `imx415-cam0.dtb` | 6, `0x1a` | M1 / `mipim1_camera1_clk` | DPHY0 | 2 | 2 | ISP0 vir2 |
| Camera1 | `imx415-cam1.dtb` | 3, `0x1a` | M2 / `mipim1_camera2_clk` | DPHY3 (HW1) | 4 | 4 | ISP1 vir1 |
| Camera2 | `imx415-cam2.dtb` | 7, `0x1a` | M3 / `mipim1_camera3_clk` | DCPHY0 | 0 | 0 | ISP0 vir0 |

All filenames start with `rk3588-orangepi-5-max-`. Connector names follow the
vendor's `rk3588-orangepi-5-max-camera{0,1,2}.dtsi`. This is the RK3588 Max,
not the RK3588S Orange Pi 5/5B.

The Camera1 variant explicitly enables ISP1 and its IOMMU: enabling only
`rkisp1_vir1` leaves its physical ISP disabled in the stock Max device tree.
Each variant replaces the disabled OV13855 input, sets four lanes at both ends,
and removes the old sensor's reciprocal reference. Other sensor paths stay
disabled. The stock `rk3588-orangepi-5-max.dtb` is still available.

## Build and structural verification

The source baseline is `55160519d9309fef993c74349a3495ac99f7e6a1` in
`StableKite/linux-rockchip`, branch
`stablekite-r70-imx415-rs300-urgent-20261022` (Linux 6.1.172).
The patch adds DT variants; it does not reimplement the previously published R70
sensor driver, change PREEMPT_RT settings, or modify the RS300 driver.

Use an ARM64 configuration with `CONFIG_ARCH_ROCKCHIP=y`, `CONFIG_VIDEO_IMX415=y`
or `m`, Rockchip CIF/ISP, CSI2 DPHY and Samsung DCPHY drivers enabled. Building
DTBs alone does not enable those drivers in an existing kernel.

```sh
make O="$OUT" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- rockchip_linux_defconfig
make O="$OUT" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -j12 KCFLAGS=-Werror \
  rockchip/rk3588-orangepi-5-max.dtb \
  rockchip/rk3588-orangepi-5-max-imx415-cam0.dtb \
  rockchip/rk3588-orangepi-5-max-imx415-cam1.dtb \
  rockchip/rk3588-orangepi-5-max-imx415-cam2.dtb
python3 tools/testing/selftests/media/imx415_opi5max_dt_test.py \
  "$OUT/arch/arm64/boot/dts/rockchip"
```

The checker requires `fdtget` from device-tree-compiler. It verifies the compiled
DTB, including the full reciprocal sensor/PHY/CSI/CIF/ISP graph, four lanes,
clock/GPIO assignments, IOMMUs, and disabled alternative camera paths.
`KCFLAGS=-Werror` affects C compilation; inspect the build log for DTC warnings
as well. These checks cannot establish electrical or streaming compatibility.

## Select the connector at boot

For example, Camera1 uses
`rockchip/rk3588-orangepi-5-max-imx415-cam1.dtb`. In an Armbian/U-Boot setup
that reads `fdtfile` from `/boot/armbianEnv.txt`, set:

```ini
fdtfile=rockchip/rk3588-orangepi-5-max-imx415-cam1.dtb
```

On an extlinux setup, set the selected boot entry's `FDT` to the installed file's
actual path. Install the DTB into the same versioned DTB directory used by that
kernel. These are complete DTBs, not `.dtbo` overlays; do not add their names to
an `overlays=` list. Disable camera overlays targeting the same connector.
Keep a boot entry using the stock Max DTB. Power off before moving the cable,
select the matching DTB, then boot again. No installation or boot edits are
performed by the patch or checker.

## Electrical assumptions and hardware validation

Pin assignments come from the vendor Max camera DTSIs:

| Connector | Reset | Power/PWDN candidate |
| --- | --- | --- |
| Camera0 | GPIO1 PC4 | GPIO1 PD5 |
| Camera1 | GPIO3 PB1 | GPIO3 PB3 |
| Camera2 | GPIO3 PB2 | GPIO3 PB4 |

The active-low mappings preserve the previous Camera2 trial's convention: the
current IMX415 driver drives logical reset 0 while on, 1 while off, and logical
power 1 while on, 0 while off. Confirm the actual module/adaptor's XCLR/PWDN
polarities and four data-lane wiring. Each variant requests 37.125 MHz XVCLK;
the driver can change it for a mode. Actual clock output needs measurement.
No sensor regulator voltages are guessed: these variants assume a powered
camera module with its own rails. A bare sensor or module needing controlled
DVDD/DOVDD/AVDD requires appropriate supply phandles. Dummy regulator messages
do not prove that physical power is present. Generic module/lens strings are
not evidence of an RKAIQ calibration match.

**Hardware status: untested on all three connectors.** On each connector,
record the loaded DT model, successful sensor-ID probe, `media-ctl -p`, RAW
capture, negotiated mode/fps, CRC/ECC errors, STREAMOFF/restart and ISP output.
Device numbering may change between connectors; discover `/dev/media*` and
`/dev/video*` from the media graph. Existing R70 high-speed modes, RKAIQ tuning
and PREEMPT_RT runtime behavior need separate on-board validation.

## Native FullFOV FHD ISP crop

The native `fullhd_replace_init` mode remains 1944x1097 RAW10 at the sensor.
Its `CROP_BOUNDS` reports 1936x1096 at (4, 0), matching ISP3 width/height
alignment while retaining all but eight columns and one row. RKAIQ uses this
selection when configuring the CIF/ISP pipeline. Other vendor 1944-wide modes
keep their existing 1920x1080 crop. No sensor timing or mode table is changed.

Run `python3 tools/testing/selftests/media/imx415_native_fhd_crop_test.py`
for a host-compiled check of the actual selection callback. Physical FOV,
frame rate and streaming still require validation on the target board.
