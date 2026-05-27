# IMX415 / RK3588 Full-Control Debug Overlay

## На какой исходник подготовлен overlay

Этот архив сформирован **строго поверх** исходников, выгруженных из WSL-дерева:

- дерево: `/home/robolightning/linux-rockchip`
- ветка: `rk-6.1-rkr5.1-rt`
- HEAD: `9391d4ba93e29a42f30f491a8e8d5ed8a409eb03`
- состояние дерева в момент выгрузки: clean

Архив устроен как overlay: каталоги `drivers/...` лежат относительно корня kernel tree. Его можно распаковать с заменой файлов в `~/linux-rockchip` после проверки HEAD и резервной ветки.

## Что покрывает этот набор

Набор экспортирует управляемые из userspace точки по всему **активному camera receive path**, который нужен для IMX415 full-FOV true-2376/90fps debugging:

1. **IMX415 sensor** — `drivers/media/i2c/imx415.c`
   - staged sensor register program с дополнительными стадиями;
   - override mode/reporting/crop/mbus;
   - независимый linear `RKMODULE_GET_CSI_DPHY_PARAM` override без `hdr=5` spoof.

2. **Rockchip CSI2-DPHY bridge** — `drivers/phy/rockchip/phy-rockchip-csi2-dphy*.{c,h}`
   - выбор источника RX profile: native/default/sensor-required/force;
   - forced `rkmodule_csi_dphy_param`;
   - forced reported RX `data_rate_mbps`;
   - applied-profile/state/counters.

3. **Samsung DCPHY analog receiver** — `drivers/phy/rockchip/phy-rockchip-samsung-dcphy*.{c,h}`
   - settle/hsfreq/sot-sync override;
   - direct gated regmap read/write/masked-write;
   - staged register program до lane enable;
   - snapshot relevant RX registers;
   - optional lane-ready timeout bypass for diagnosis only.

4. **MIPI CSI-2 host** — `drivers/media/platform/rockchip/cif/mipi-csi2*.{c,h}`
   - gated raw host register read/write/masked-write;
   - staged host-register program до upstream stream start;
   - state/error-mask/PHY-state snapshot.

5. **RKCIF capture block** — `drivers/media/platform/rockchip/cif/dev.{c,h}`
   - gated raw CIF MMIO read/write/masked-write;
   - state reporting, in addition to already existing vendor high-level knobs.

6. **CIF-to-ISP link mode / online-readback route** — `drivers/media/platform/rockchip/cif/subdev-itf.{c,h}`
   - `rdbk_mode` and derived/forced `link_mode` debug control;
   - optional interception of subsequent `RKISP_VICAP_CMD_MODE` requests;
   - direct coverage for the previously unresolved online/readback hypothesis.

## Что этот набор не обещает

Это не экспорт каждого внутреннего регистра всего RKISP/SoC. Он покрывает доказанно релевантную active-chain: sensor → DPHY profile selection → Samsung RX analog/settle → CSI host → CIF → CIF/ISP online-readback routing.

Если после этого доказательство укажет на внутренний RKISP processing block, отдельные ISP-файлы не были включены в исходный source bundle и потребуют отдельного review. Не называйте этот overlay «универсальным доступом ко всему SoC».

## Безопасность/default behaviour

- Все новые прямые register/profile/route overrides выключены по умолчанию.
- При выключенных debug gates native-путь сохраняется; для RX добавлен readback last-result state, а native stream error остаётся compatibility-ignored, как в vendor code.
- Direct register interfaces предназначены только для лабораторных тестов; ошибочная запись может остановить поток до reboot/recovery.
- Не используйте `debug_skip_init_regs=1` для проверки payload.
- Любой active-test заканчивайте real `index0` baseline payload proof.

## Статус проверки архива

В среде подготовки выполнены:

- соответствие base sensor-файла exact WSL source;
- `git diff --check` — без whitespace errors;
- проверка, что unified patch применяется к выгруженному base-tree и даёт byte-identical replacement files;
- статический delimiter/required-marker audit изменённых C-файлов.

**Полная kernel compilation здесь не выполнялась**, потому что в артефактной среде нет всего вашего kernel tree/build output. Перед установкой image обязательно соберите это в WSL обычной для вашего ядра процедурой.

## Применение в WSL

Сначала сохраните чистую точку возврата:

```bash
cd ~/linux-rockchip
test "$(git rev-parse HEAD)" = "9391d4ba93e29a42f30f491a8e8d5ed8a409eb03"
git status --short --branch
git branch backup/pre-imx415-fullcontrol-$(date +%Y%m%d-%H%M%S)
```

Убедившись, что unexpected local changes отсутствуют, распакуйте overlay:

```bash
cd ~/linux-rockchip
tar -xzf /mnt/c/Users/Robolightning/Downloads/imx415_fullcontrol_overlay_rk-6.1-rkr5.1-rt_9391d4ba.tar.gz
./imx415_fullcontrol_tools/scripts/validate_overlay.sh ~/linux-rockchip
git diff --stat
```

Затем запускайте вашу обычную процедуру пересборки ядра. Не устанавливайте image при ошибке компиляции.

## Первый boot после сборки

Не включайте overrides сразу. Последовательность:

1. Проверить boot и real `index0` baseline payload с новыми gates выключенными.
2. Найти новые sysfs nodes скриптом `scripts/discover_fullcontrol_nodes.sh`.
3. Снять read-only state.
4. Проверить force-profile сначала на clean `oracle75`, сохраняя linear `hdr=0` и exact sensor program.
5. Только если control остаётся payload-clean, повторить единственный официальный Sony90/LANEMODE4 target с выбранным RX override.
