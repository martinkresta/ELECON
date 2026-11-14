# ELECON firmware changelog

Newest version first. Versions before v001.11 are not listed.

## v001.16 (2026-11-13)

- Task: VARS2 step 6b part B (`UHA_COMMON/tasks/vars2_stop_legacy_tx.md`), branch `feature/vars2_stop_legacy_tx`
- CRC: `0x40B99687`
- Based on v001.15.

- Old variable frame `0x110+node` is no longer sent on CAN; only VARS2 `0x180+node` (shared `COM.c`, part A).
- `CMD_VAR_VALUE` RX case removed from `APP.c`. All active remote inputs have a VARS2 row and are read through the read layer.
- `SCOM.c`: old `0x221` traffic to the RPi removed: PC scan list (`InitPcScanList()`, 271 entries, `UpdateScanList()`, scan loop), `CMD_READ_VAR_REQUEST` (0x713) and `SendVariable()`. RAM -3.5 kB.
- `VAR_NETWORK_STATUS` is still set every 5 s from `COM_GetNetworkStatus()`; it now reaches the RPi as VARS2 `NETWORK_STATUS` on `0x184` (tunnel only, not on CAN). UhaGate6 gets no live data from this version on.

## v001.15 (2026-11-12)

- Task: GEMON invalid propagation (`ASW_ELECON/tasks/gemon_invalid_propagation.md`), branch `feature/gemon_invalid_propagation`
- CRC: `0x5DBA943D`
- Based on v001.14.

- GEMON publishes its outputs with the validity of their inputs, in three groups: battery (`STRG1/2_ENERGY_WH`, `_VOLTAGE_V10`, `_CAPACITY_AH`, `_POWER_W`, `_CURRENT_A10` -> `BAT_*`), solar (`LPV1/2_*` -> `SOLAR_POWER_W`, `SOLAR_ENERGY_TODAY_10WH`), load (`LLOAD1/2_*` -> `LOAD_W`, `CONS_TODAY_10WH`, `LOAD_A100`; `LOAD_A100` is also invalid when battery is invalid). Before: always `valid = 1`.
- Battery group is also invalid when total capacity or voltage <= 0 (division by zero / inf-NaN cast).
- An invalid group is published with `valid = 0` and its last valid values (0 before the first valid computation). Fixes the 2026-11-12 case: during an ELECON_D reboot `STRG2_*` zeros gave `BAT_SOC` ~112 % published as valid; now the last SOC is kept, flagged invalid (TECHM heater -> invalid inputs, RECU -> safe fan configuration).
- Optimal charging current / `CMD_BALANCE_INFO` unchanged, computed from the last valid battery values. Values unchanged while all inputs are valid.

## v001.14 (2026-11-12)

- Task: CAN TX stall of 2026-11-12 (`UHA_COMMON/VarDefs/can_tx_stall_2026-11-12.md`), branch `feature/can_tx_supervision`
- CRC: `0x31795646`
- Based on v001.13.

- CAN automatic bus-off recovery (ABOM) enabled (`.ioc`, `main.c`, and forced in `MCAN_Init()`); it was disabled, so a bus-off lasted until reset.
- TX supervision in MCAN: frames pending and no successful transmission for 3 s, or no successful transmission at all for 12 s → abort mailboxes, CAN init-mode cycle, HAL state repair. After 5 failed recoveries in a row → `NVIC_SystemReset()` (STARTUP byte 3 = 30).
- New eleError reports, sent over SCOM (so they reach the RPi even while CAN TX is dead) as `0x620 [4, 1, code, ...]`: `01` TX stall (TEC, REC, ESR), `02` TEC/REC monitor (immediate on error warning / passive / bus-off, at most 1 per 10 s; 60 s summary if TEC/REC peak >= 16), `03` software snapshot at a stall, `04` corrupted MCAN pointer restored.
- TX ring full: drops counted. SysInfo types `0x10..0x1A` (request `0x550 [4, type]`) return CAN diagnostics.
- No change to the VARS2, the old CAN format or the RPISERP tunnel.

## v001.13 (2026-11-08)

- Task: VARS2 read layer, part B (`UHA_COMMON/tasks/vars2_read_layer.md`), branch `feature/vars2_read_elecon`
- CRC: `0x66F07415`

- `VAR_EnableVars2Read()` after `COM_Init()`: remote variables are read from VARS2: `POW_KITCHEN_W` (AC.c), `BMS1_VOLTAGE_V10`, `SHUNT_PCK1_CURRENT_A100`, `AXPERT_SOLAR_W` (ELECON.c) and `STRG2_*`, `LLOAD2_*`, `LPV2_*` (GEMON.c, 11 variables). None of these callers evaluates `invalid` after the read (GEMON and AC ignore it, ELECON.c checks it before these reads), so after a producer timeout the last value stays in use as before.
- `SendVariable()` in `SCOM.c` uses `VAR_GetVariableLegacy()`: old `0x221` still carries `Vars[]` (old vs new comparison stays meaningful).
- Own variables are still read from `Vars[]`.

## v001.12 (2026-11-08)

- Task: VARS2 step 2 (`UHA_COMMON/tasks/elecon_vars2_tunel.md`), branch `feature/elecon_vars2_tunel`
- CRC: `0x6842378F`

- VARS2 tunnel to the RPi over RPISERP (spec sec.7): every VARS2 frame received from CAN (packet id `0x180+producer`), every VARS2 frame ELECON sends (`0x184`) and ELECON-local variables not on CAN (`0x184`, `VarMeta.PeriodMs`).
- RPISERP: TX FIFO 10 -> 64 frames, next frame started on UART TX-complete (was 1 frame per 10 ms), IRQ-safe; dropped frames counted in `gRspTxOverflow` (`RSP_GetTxOverflowCount()`).
- Old traffic unchanged: scan list `0x221`, log `0x620`, RTC sync.
- VARS2 fix: a node's own value no longer reports a timeout in `VARS2_IsValid()` (own state refreshed when the frame is sent).
- VARS2 fix: `0x18x` frames with DLC != 8 or RTR are dropped in `COM_GetRxMessage()`.

## v001.11 (2026-11-01)

- Task: VARS2 step 1 (`UHA_COMMON/tasks/vars2_dual_transmission.md`), branch `vars2-step1`
- CRC: `0x2505BC5F`

- Sends all variables in the old format (`0x110+node`) and the new VARS2 format (`0x180+node`).
- Received VARS2 frames are stored in `Vars2`; control logic still uses the old variables.
- BMS1 data published as `VAR_BMS3_*` (`Core/Src/LEMON.c`, committed after this build).
