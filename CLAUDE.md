# ASW_ELECON — Project Notes

## Hardware

- **CAN Node:** 4
- **MCU:** STM32L431RC
- **Role:** Main house electrical controller
- **Current version:** v001.10 (branch `dev_asw`)

---

## BMS Architecture — CRITICAL: struct names ≠ VAR names

ASW_ELECON controls **2 physical battery packs** locally. A third pack lives in the workshop and is managed by **ASW_ELECON_D** (CAN node 7).

| Struct in LEMON.c | VAR namespace used | Physical location |
|-------------------|--------------------|-------------------|
| `mBms1`           | `VAR_BMS2_*`       | House — battery pack 1 (80 Ah) |
| `mBms2`           | `VAR_BMS3_*`       | House — battery pack 2 (220 Ah) |
| *(no local struct)* | `VAR_BMS1_*`     | Workshop — managed by ASW_ELECON_D, received via CAN |

**Why the offset:** `VAR_BMS1_*` is reserved for the workshop pack (ELECON_D). The two house packs are numbered BMS2 and BMS3 in the VARS bus so all three packs have globally unique IDs across the CAN network.

---

## LEMON.c — Pack configuration

```c
mLemon.Cfg.Pack1_Ah = 80;   // mBms1 → VAR_BMS2_*
mLemon.Cfg.Pack2_Ah = 220;  // mBms2 → VAR_BMS3_*
```

---

## VARS IDs (UHA_COMMON/Inc/VARS.h)

| Range    | Content |
|----------|---------|
| 35–39    | BMS3 summary: SOC, CURRENT_A10, VOLTAGE_V10, ENERGY_STORED_WH, TODAY_ENERGY_WH |
| 350–365  | BMS3 cell voltages: VAR_BMS3_CELL1_MV … CELL16_MV |
| 366–381  | BMS3 cell temperatures: VAR_BMS3_CELL1_C … CELL16_C |

`NUM_OF_VARIABLES` = 400 (bumped from 350 when BMS3 cell vars were added).

---

## Recent work (v001.10)

- Added all `VAR_BMS3_*` IDs to `UHA_COMMON/Inc/VARS.h` (summary + 32 cell vars)
- Added BMS3 vars to `SCOM.c` `InitPcScanList()` for PC telemetry
- Fixed `Pack1_Ah` in `LEMON.c`: was `0` (disabled placeholder), now `80`
- `NUM_OF_VARIABLES` 350 → 400
