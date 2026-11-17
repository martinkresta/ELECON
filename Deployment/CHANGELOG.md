# ELECON firmware changelog

Newest version first. Versions before v001.11 are not listed.

## v002.04 (2026-11-17)

- Task: commands step 3c (`UHA_COMMON/tasks/cmd_inverters_recu.md`, mapping agreed in `UHA_COMMON/tasks/cmd_inverters_recu_report.md`),
  branch `feature/cmd-3c` (UHA_COMMON + ASW_ELECON + ASW_ELECON_D + ASW_RECU_NUCLEO)
- CRC: `0xCC7BBD54`

- Commands (local path from the RPi, `SCOM.c` -> `CMD_ProcessFrame()` -> `APP_OnCommand()`): `CMD_AC_3KW` (401),
  `CMD_AC_5KW` (402), bool. `AC_Command()`: on = `ButtonGesture()` from `acs_Off` only (min on time, then auto off
  by power, as the wall switch); already on -> ignored. Off = relay off + `acs_Off` in every on state (also in the
  first 5 s, where a gesture would add 2 h). TECHM's `RemoteRequests` (0x130) are not touched.
- The 300 W inverter gets no command.
- New VARS2 table (RECU setpoint states 512 / 513), so the tunnel knows them.
- Flash +456 B, RAM +32 B (since v002.03).

## v002.03 (2026-11-17)

- Task: command channel step 3a (`UHA_COMMON/tasks/cmd_channel_fw.md`, spec `UHA_COMMON/VarDefs/CMD_spec.md`),
  branch `feature/cmd-channel` (UHA_COMMON + ASW_ELECON + ASW_IOBOARD_U)
- CRC: `0x25B991B7`

- New RPISERP packet `0x52 CMD_RPI_COMMAND` (RPi -> ELECON), `SCOM.c ForwardRpiCommand()`: data length exactly 8,
  rate limit 10 per 1 s and 20 per 60 s (fixed windows), CAN frame id `0x200 + (target & 0x0F)` (fixed base), DLC 8,
  data `[cmd_lo, cmd_hi, v0..v3, seq, 0]`; target 4 (ELECON) is handled locally by `CMD_ProcessFrame()`, not sent.
  ELECON does not check command ids or values (the target node does).
- Every drop: node log `LOG 48` (eleLog 0x30: reason 1 = length / 2 = rate, info, length drops, rate drops u16),
  at most one per second. Debug counters `gScomCmdDropLen`, `gScomCmdDropRate`, `gScomCmdForwarded`, `gScomCmdLocal`.
- `APP_OnCommand()`: empty switch (no ELECON commands yet). Shared: `CMD.c`, `uha_cmds_gen.c`, `COM.c` consumes
  `0x200..0x20F` frames (own -> `CMD_ProcessFrame()`, others dropped).
- Flash +1.0 KB (+1008 B), RAM +32 B (since v002.02). No change of existing behaviour.

## v002.02 (2026-11-16)

- Task: DS18B20 85.0 °C glitches (`UHA_COMMON/KnownIssues/KI-005_ds18b20_85c_glitches.md`), branch `feature/temp-85-filter`
  (UHA_COMMON + ASW_ELECON)
- CRC: `0x043CE012`

- `TEMP.c` (shared): a readout of exactly 85.0 °C (DS18B20 power-on scratchpad 0x0550, valid CRC) is ignored and the
  last value stays, unless the last valid value is within 80.0–90.0 °C or 85.0 came 3 times in a row.
- Diagnostics: every ignored readout is counted; at most once a minute two node log messages go to the RPi
  (`LOG 33`: sensor, total count, uptime in minutes; `LOG 34`: sensor, scratchpad bytes TH, config, 6, 7).
- `OW.c`: read-only getters `OW_GetScratchpad()`, `OW_IsBusy()`; `LOG.c`: `LOG_SendEvent()`. No change of the 1-Wire timing.

## v002.01 (2026-11-15)

- Task: VARS2 states step 2b (`UHA_COMMON/tasks/states_nodes.md`), branch `feature/vars2-states-2b`
  (UHA_COMMON + this project); report `UHA_COMMON/tasks/states_nodes_report.md`
- CRC: `0x03950A26`

- New VARS2 group `States.Elecon` (ids 480–483), `send: on_change` (on change, min 1 s apart, + every 20 s).
- Inverters 300 W / 3 kW / 5 kW (REL1–3): AC state machine state as enum `InverterState`; new read-only getter
  `AC_GetState()` in `AC.c`.
- Exterior light: `PWROUT_GetPct(OUT1)` (OUT1 + OUT4 are always switched together, only 0 / 100).
- Written once per second in `UpdateStates()` (in `APP_Update_1s()`, after `AC_Update_1s()`).
- The tunnel forwards the new ids of all nodes as before (frame hook, no ELECON change needed for that).
- Shared code: VARS2 `send: on_change` (step 2a), new YAML rows of all nodes. No change of control behaviour.
- Flash +1.3 KB, RAM +0.5 KB (since v002.00).

## v001.17 (2026-11-14)

- Task: VARS2 direct access part B (`UHA_COMMON/tasks/vars2_direct_access.md`), branch `feature/vars2_direct_access`
  (UHA_COMMON: `main` + `feature/vars2_direct_access_elecon`)
- CRC: `0xAFB6D7E8`
- Based on v001.16.

- Variables live in `Vars2` (value in physical units + valid flag); the old `Vars[]` / `VAR_*` API is no longer used here.
- LEMON: packs named by capacity (80 Ah on UART3, 220 Ah on UART2); writes `HomeEnergy`. The BMS driver publishes
  `PackHome80Ah` / `PackHome220Ah` itself: **invalid 5 s after the last valid BMS frame** (before: valid forever).
- GEMON combines `HomeEnergy` and `WorkshopEnergy` into `GlobalEnergy` with exact values: `BAT_SOC` is no longer
  truncated to whole %, currents / voltages / energies no longer to 0.1 / whole units. `LOAD_A` has the load validity.
- MPPT writes `Mppt` (each value valid once read, as before); temperature now signed with 0.01 degC.
- Exterior light (10 V / 20 V) and AC kitchen-power (> 50 W) thresholds in physical units, same switching points.
- Obsolete serial command `CMD_SET_VAR_VALUE` removed; the RPi tunnel is unchanged.
- VARS2 sends this node's variables by the YAML `period_ms` (CAN: same 10 variables and periods as the removed
  `COM_AddStreamedVariable()` list; tunnel: unchanged).
- Flash -1.8 KB, RAM -2.2 KB.

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
