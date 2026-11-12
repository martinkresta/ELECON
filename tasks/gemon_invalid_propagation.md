# Task: GEMON — publish computed values as invalid when their inputs are invalid

Single node: ELECON. Small change, Martin wants to deploy it the same day.

## Problem

`Core/Src/GEMON.c` `GEMON_Update_1s()` reads both storages (`STRG1_*` own, `STRG2_*` from ELECON_D),
loads (`LLOAD1/2_*`) and PV (`LPV1/2_*`), collects `invalid`, but **never uses it**, and publishes all
results with `valid = 1` (`GEMON.c:83-95`).

Observed 2026-11-12: while ELECON_D rebooted, its `STRG2_*` came as zeros flagged invalid
(`ASW_ELECON_D/Core/Src/LEMON.c:230-233`, `strgvalid`). GEMON computed a wrong `BAT_SOC` and published it
as valid; RECU reacted to the value (fans down / off by its SOC thresholds) instead of its invalid-input
fallback. Since the VARS2 read layer (v001.13) a missing ELECON_D also times out (3 s) on ELECON, so the
invalid flag is available; GEMON only has to pass it on.

Consumers of the GEMON outputs (all already handle `invalid`):
- TECHM `ELHEATER.c:107,142-146` (`BAT_SOC`, `BAT_VOLTAGE_V10`, `BAT_CURRENT_A10`): invalid → heater off
  (`eElh_InvalidInputs`).
- RECU `RECON.c:92-102` (`BAT_SOC`): invalid → fans to the safe configuration.
- ELECON_D `LEMON.c:438` reads `BAT_SOC` with `invalid = NULL` (ignores it): not changed here, report only.
- RPi (`uha.json`, `uha2.json`) shows the valid flag.

## Rules

- Read first: `UHA_FW/CLAUDE.md`, this project's `CLAUDE.md`, `GEMON.c/.h`.
- Feature branch `feature/gemon_invalid_propagation` from **`feature/can_tx_supervision`** (that is the
  deployed v001.14, not merged to `main` yet).
- Change only `GEMON.c` (+ `GEMON.h` if needed), `Core/Inc/APP_INFO.h` (version) and `Deployment/`.
  No change of the control logic (balancing, optimal charging current, `SendBalanceInfo()`), of the
  published value formats, or of other files. Anything else → ask.
- Live network: never flash. Martin does it. Never overwrite Martin's uncommitted files; `git status` first.
- Build **only ELECON**.

## What to do

1. Split the single `invalid` into one flag per output group, from exactly the inputs each group uses:

   | Group | Inputs | Outputs |
   |---|---|---|
   | battery | `STRG1/2_ENERGY_WH`, `_VOLTAGE_V10`, `_CAPACITY_AH`, `_POWER_W`, `_CURRENT_A10` | `BAT_SOC`, `BAT_VOLTAGE_V10`, `BAT_CURRENT_A10`, `BAT_POWER_W`, `BAT_ENERGY_WH` |
   | solar | `LPV1/2_POWER_W`, `LPV1/2_PROD_WH` | `SOLAR_POWER_W`, `SOLAR_ENERGY_TODAY_10WH` |
   | load | `LLOAD1/2_POWER_W`, `LLOAD1/2_CONS_WH` (+ battery voltage for `LOAD_A100`) | `LOAD_W`, `CONS_TODAY_10WH`, `LOAD_A100` (also invalid when battery is invalid) |

2. Battery group also invalid when `totalCapacity <= 0` or `BatVoltage <= 0` (today a division by zero and a
   float → int16 cast of inf/NaN is possible).
3. When a group is invalid: publish it with `valid = 0` and keep the **last valid values** (do not publish
   values computed from invalid inputs). Before the first valid computation, publish 0 with `valid = 0`.
4. Keep `mGmon` / `mEutil` and `SendBalanceInfo()` behaviour as today, except that the battery values they
   use are the last valid ones (follows from 3). Report what TECHM gets in `CMD_BALANCE_INFO` while the
   battery group is invalid; do not change it.
5. PC unit test (gcc, stubbed `VAR_GetVariable` / `VAR_SetVariable` / `RTC_GetTime` / `COM_SendMessage`):
   all valid = today's values; each input invalid → only its group invalid, last values kept; STRG2 zeros
   flagged invalid (the incident) → `BAT_SOC` keeps the last value, invalid; zero capacity/voltage; startup
   with invalid inputs → 0 + invalid; recovery.
6. Bump `ASW_VERSION` to **v001.15** (own commit, check `Deployment/v001.15` does not exist), build ELECON,
   add the `Deployment/CHANGELOG.md` entry (date, task, branch, CRC, "Based on v001.14", what changed).
7. Independent reviewer subagent: inputs per group complete, no change of values while all inputs are
   valid, no new division/cast hazards.
8. Small logical commits. Do not merge.

## Report to Martin

Changes, unit test results, version + CRC, flash/RAM before → after, the `CMD_BALANCE_INFO` finding, and
what to check after flashing: `BAT_*` valid in `uha2.json`; when ELECON_D is reset (NMT) `BAT_*` turn
invalid within ~3 s, RECU goes to its safe configuration, TECHM heater switches off, and all recover when
ELECON_D is back.
