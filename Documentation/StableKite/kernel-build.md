# StableKite ARM64 full kernel build

This tree supports a standard full Rockchip ARM64 build of `Image`, configured
modules, and every DTB/DTBO selected by `CONFIG_ARCH_ROCKCHIP`. The reference
toolchain is `aarch64-linux-gnu-gcc` 13.3.0 with binutils 2.42.

```sh
git clone --depth=1 --single-branch \
  --branch stablekite-r70-imx415-rs300-urgent-20261022 \
  https://github.com/StableKite/linux-rockchip.git src
cd src
OUT="$PWD/../out"
make O="$OUT" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- \
  rockchip_linux_defconfig stablekite_rt.config
make -j24 O="$OUT" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- \
  KCFLAGS=-Werror Image modules dtbs > ../build.log 2>&1
```

Use a new clone and output directory for release validation. Require make exit
status zero and inspect the entire log for C, assembler, linker, modpost and DTC
diagnostics. This is the standard Kbuild warning level; additional `W=1`
diagnostics are a separate maintenance task. No new warning-disable options
are introduced by these fixes.

The fragment enables IMX415, RS300, UDMABUF and PREEMPT_RT and retains
`CONFIG_WERROR=y`. It reproduces the previous R120 output configuration with
SHA-256 `66ba9e7faa27cc6e86a71eed3edc357975c9598fd0a8bb0ddfa0ec34832361dd`.
RT uses the experimental ARM64 support declaration already in this tree;
compilation does not establish hardware RT correctness.

## Corrected full-build diagnostics

- Declare bus cell geometry at the actual overlay scopes containing one-cell
  I2C, SPI, DSI and graph-port addresses.
- Use proper panel port nodes, hexadecimal endpoint and device unit names,
  and a PCI bridge bus number within its root controller's bus range.
- Disconnect inherited single-camera endpoints only on board variants that
  replace those inputs; use the registered Samsung PHY providers in the
  vehicle camera configuration.
- Keep MCU timing nodes outside graph ports, referenced by
  `rockchip,mcu-timing`. VOP2 and RGB retain support for legacy child nodes;
  all existing timing values are preserved.
- Resolve MPP version metadata from `$(srctree)` for out-of-tree builds.
- Backport upstream DTC commit
  [84b056a89d](https://kernel.googlesource.com/pub/scm/utils/dtc/dtc/+/84b056a89d3c5b6cf6c5eeeafd4c4b14d6333aa9),
  which fixes checks that cannot infer external overlay endpoint names and
  reciprocal links. Validation of complete graphs remains enabled. In
  particular the Mini2 overlay and its preserved receiver phandle are unchanged.

## Camera regression checks

```sh
python3 tools/testing/selftests/media/imx415_opi5max_dt_test.py \
  "$OUT/arch/arm64/boot/dts/rockchip"
python3 tools/testing/selftests/media/rs300_protocol_test.py
python3 -O tools/testing/selftests/media/rs300_protocol_test.py
python3 tools/testing/selftests/media/rs300_protocol_negative_test.py
fdtoverlay -i "$OUT/arch/arm64/boot/dts/rockchip/rk3588s-orangepi-5.dtb" \
  -o ../mini2-merged.dtb \
  "$OUT/arch/arm64/boot/dts/rockchip/overlay/orangepi-5-mini2-rs300.dtbo"
python3 tools/testing/selftests/media/rs300_dt_graph_test.py \
  "$OUT/arch/arm64/boot/dts/rockchip/rk3588s-orangepi-5.dtb" ../mini2-merged.dtb
```

A successful build is a source/binary qualification. Boot, streaming, PHY
electrical compatibility, ISP/RKAIQ and RT behavior still need hardware tests.
