# ELECON firmware changelog

Newest version first. Versions before v001.11 are not listed.

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
