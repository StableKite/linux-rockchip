# Новые userspace interfaces

Имена каталогов platform devices на Orange Pi могут отличаться; используйте `discover_fullcontrol_nodes.sh`.

## IMX415 sensor sysfs (`/sys/bus/i2c/devices/7-001a`)

Новые/расширенные файлы:

- `debug_dphy_param`: `policy=native|force|reject`, `lp_vol_ref=`, `lp_hys0..3=`, `esc0..3=`, `skew0..3=`, `clk_term=`, `data_term0..3=`.
- `debug_program`: новые policy `before_init`, `replace_init`, `after_init`, `overlay_after_ctrl`, `after_release`.
- `debug_auto_release`, `debug_auto_standby`, `debug_program_allow_ctrl_mode`.
- `debug_ioctl`, `debug_mbus`, `debug_crop`, расширенный `debug_state`.

Главная цель: получить DCPHY profile для linear Sony90 без подмены `hdr_mode`.

## CSI2-DPHY bridge platform device

- `debug_rx_enable`: gate; `0` по умолчанию.
- `debug_rx_policy`: `param=native|default|sensor|force rate=native|force rate_mbps=<n>`.
- `debug_rx_param`: forced Samsung profile fields.
- `debug_rx_state`: фактически выбранный источник profile, rate, sensor ioctl result и stream counters.

## Samsung DCPHY device

- `debug_dcphy_enable`: gate; `0` по умолчанию.
- `debug_dcphy_settle`: `enable=<0|1> clk_settle=<n> hsfreq=<n> sot_sync=<n> ignore_lane_ready=<0|1>`.
- `debug_dcphy_reg`: `r <reg>`, `w <reg> <val>`, `mw <reg> <mask> <val>`.
- `debug_dcphy_program`: staged program — `clear`, `append w`, `append mw`, `commit`, `enable 1`.
- `debug_dcphy_state`, `debug_dcphy_snapshot`.

## MIPI CSI-2 host device

- `debug_csihost_enable`.
- `debug_csihost_reg`: `r/w/mw` по mapped host register window.
- `debug_csihost_program`: staged host program до upstream stream start.
- `debug_csihost_state`: `N_LANES`, `PHY_STATE`, `ERR1/ERR2`, masks и control.

## RKCIF device

В дополнение к существующим vendor knobs добавлены:

- `debug_cif_enable`.
- `debug_cif_reg`: gated raw MMIO `r/w/mw`.
- `debug_cif_state`.

## CIF-to-ISP `sditf` device

- `debug_sditf_enable`.
- `debug_sditf_mode`: `force_mode=<0|1> rdbk_mode=<n> force_link=<0|1> link_mode=<n> apply=<0|1>`.

Этот интерфейс предназначен для проверки `online/rdbk/link_mode`, не для случайного переключения во время захвата. Меняйте mode только при остановленном stream и всегда фиксируйте baseline recovery.
