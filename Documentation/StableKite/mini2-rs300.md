# StableKite R70 / Mini2 640 native MIPI driver integration

Base: `StableKite/linux-rockchip`, `stablekite-r70-linux-6.1.172-no-imx415`
commit `0ab94198c7fbb3a82e6018d1ad0b087c4ed08654`
Git tree `9389e8d2e23d38d4f16274f62cc35b9dfe657b5a`.

## Hardware and provenance

- Supplier: Shenzhen Cheng'en; supplied sales model **UV 640 9MM / Mini2 640**.
- Reference SKU from the user/vendor discussion: **Mini264009110XHNRMA**, 9.1 mm F/1.0.
- Sensor: Mini2 640 non-radiometric VOx thermal imaging module; 640x512 / 12um / 8-14um.
- Interfaces: native 2-lane CSI-2 YUV422; I2C controls at seven-bit 0x3c. **5 V** power supply.
- Vendor confirmed 400 Mbps/lane; Linux `link-frequencies` (DDR clock) = 200 MHz.
- Module has own timing oscillator; does not need SoC-provided MCLK. No external firmware blob reported.
- Hardware and firmware revision, physical marking and timing measured on the shipped modules: **not established**.
- Manufacturer has confirmed shutter/FFC controls and operation without the USB daughterboard, but no field verification yet.
- Public kernel-driver reference: `https://github.com/Kodrea/mini2-thermal-driver`, `platforms/radxa/rock5bp/src/rs300.c` (GPL-2.0). Additional command reference: Purple River `RS300_JetsonNano` SDK.

## Actual integration files

- `drivers/media/i2c/rs300.c`: V4L2 I2C subdevice. Its command transport uses the real RS300 18-byte packets, CRC-16/XMODEM, command/status registers and 28-byte stream start/stop packets.
- `drivers/media/i2c/Kconfig` and `Makefile`: `CONFIG_VIDEO_RS300` integration.
- `arch/arm64/configs/rockchip_linux_defconfig`: `CONFIG_VIDEO_RS300=y`, independent of existing IMX415 setting.
- `arch/arm64/boot/dts/rockchip/overlay/orangepi-5-mini2-rs300.dts`: **opt-in** Orange Pi 5 camera1 CSI overlay; default DTB untouched.
- `Documentation/devicetree/bindings/media/i2c/infisense,mini2-640.yaml`: native module binding.

## Wire-level caution

The Orange Pi 5 camera1 path shown by the exact R70 DTS is `i2c7 -> csi2_dphy0 -> mipi2_csi2 -> rkcif_mipi_lvds2`.
Module I2C defaults to **1.8 V**; a suitable translator/voltage domain MUST be present if Orange Pi I2C operates at 3.3 V. The 50-pin module needs stable 5 V on pins 1 and 50, and correct common ground. Native differential CSI-2 pairs are: clock pins 12/13, D0 40/39, D1 38/37 (module pin numbers). Check FPC orientation and the board's camera1 connector mapping before hardware power.

The overlay uses `vdd-supply = <&vcc5v0_sys>` from the base DTS. This references the *logical supply*: actual harness or adapter still must carry regulated 5 V to the module; an overlay cannot supply absent hardware wiring.

## Supported today (source implementation)

- One **fixed physical** Mini2 640 mode: 640x512 at 60 Hz, non-radiometric YUV422.
- `V4L2_MBUS_FMT_YUYV8_2X8`, following Kodrea's known RK3588/RKCIF path. Actual byte order appears UYVY; userspace must inspect a captured frame and request its actual V4L2 FourCC accordingly. These labels may differ at media-bus and capture-node level.
- V4L2 controls: manual FFC, auto-FFC, spatial NR, temporal NR, DDE.
- Power via `vdd` regulator, optional reset GPIO; probe must succeed at I2C status register before media registration; stream on/off, cleanup, system suspend/resume.
- **No Y16/RAW14/RAW16** advertised until CSI-2 data type and firmware have been verified, and no temperature/radiometry API.
- Buffers, timestamps, DMA-BUF: owned by **existing** RKCIF capture driver. The sensor subdevice does not create a fake `/dev/video*` device or its own DMA allocations. R70's UDMABUF remains untouched.

## Build on the real R70 checkout (not yet executed here)

```bash
# The standard apply script verifies the exact R70 head before applying.
./apply-to-r70.sh /path/to/checked-out/R70

cd /path/to/checked-out/R70
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- O=out rockchip_linux_defconfig
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- O=out -j"$(nproc)" prepare scripts
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- O=out -j"$(nproc)" drivers/media/i2c/rs300.o
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- O=out -j"$(nproc)" dtbs
```

The existing R70 `rockchip_linux_defconfig` sets `CONFIG_PREEMPT_VOLUNTARY=y`, despite the separate requirement `CONFIG_PREEMPT_RT=y`. This driver changes neither choice; verify the ACTUAL kernel build's `.config` for PREEMPT_RT rather than assuming it from defconfig.

## First board test (blocked until hardware, physical adapter and base image)

1. Confirm module and connector orientation, 5V supply, level-shifted 1.8V I2C, CSI pinout and 2-lane D-PHY. Record exact PCB and firmware revision.
2. Enable **only** `orangepi-5-mini2-rs300.dtbo` on camera1 (not simultaneously with another camera1 overlay). Do not change the existing global/default DTB.
3. Boot, inspect `dmesg | grep -i rs300`, `media-ctl -p`, `/sys/class/video4linux`, `v4l2-ctl --list-devices`. Expected I2C ACK at `0x3c` on actual i2c7, no phantom video device if probe fails.
4. Negotiate 640x512 YUV422, capture 600+ frames by `v4l2-ctl --stream-mmap --stream-count=600 --stream-to=/tmp/thermal.yuv`; inspect timestamp intervals and frame sequence, especially initial warm-up/FFC freeze. Check image byte order.
5. Manual FFC and auto-FFC ON/OFF, NR/DDE controls; reconnect, stop/restart, suspend/resume, error recovery, PREEMPT_RT stress.
6. Compare IMX415 capture before and after enabling overlay; test camera1 port exclusive use and combined kernels after IMX415 branch merge.

## Known limitations and open risks

- **No hardware acceptance test or confirmed module HW/FW revision.** Declared performance is not verified.
- DT endpoint 200 MHz and control packet fields are based on vendor's 400 Mbps/lane statement and existing Kodrea implementation. D-PHY timing and actual link mode may require correction on real modules.
- Only one documented non-radiometric YUV422 capture mode is exposed. High-bit grayscale and radiometric output are NOT implemented or claimed.
- No runtime autosuspend by design; core remains powered while its subdevice is bound, except system suspend. Add validated runtime PM only after testing state retention, FFC drift and resume semantics.
- System resume powers the module back on, but does not automatically restart an existing V4L2 capture pipeline. Userspace must issue streamoff/streamon and reapply controls after resume.
- Initial status probe verifies that the device responds on I2C, not a unique silicon chip ID; the manufacturer has not supplied an authoritative version/chip-ID register.
- Selected overlay maps **camera1**; if IMX415 uses this port, choose a different hardware route/overlay, not an overlapping active graph.

## Future merge with IMX415

Once both branches are published, merge via a separate integration branch starting from the exact R70 parent; avoid rebasing/force-pushing either feature branch. The thermal change should touch only files listed above, except the shared defconfig and media Kconfig/Makefile. Resolve those as additive changes, keep IMX415 driver and its own overlay/port intact. Run arm64 kernel + dtbs build, test both cameras independently and simultaneously when they use distinct CSI controllers.
