# ASW_ELECON — Project Notes

## Hardware

- **CAN Node:** 4
- **MCU:** STM32L431RC
- **Role:** Main house electrical controller
- **Current version:** v001.17 (VARS2 direct access, Nov 2026)

---

## Variables (VARS2)

All shared data lives in `Vars2` (see `UHA_COMMON/CLAUDE.md`, `UHA_COMMON/VarDefs/VARS2_spec.md`). `APP.c`
wires the groups to the modules; ids, periods and producers come from `UHA_COMMON/VarDefs/uha_vars.yaml`.

| Group (produced here) | Module | Notes |
|---|---|---|
| `GlobalEnergy` {Battery, Solar, Load} | GEMON | combines `HomeEnergy` + `WorkshopEnergy` (from ELECON_D); read by TECHM, RECU, ELECON_D |
| `HomeEnergy` {Storage, Load, Pv} | LEMON | always valid |
| `PackHome80Ah`, `PackHome220Ah` | `UHA_COMMON/Src/BMS.c` (via LEMON) | invalid 5 s after the last valid BMS frame |
| `Mppt` | MPPT | each field valid once read |
| `ShuntHome` | SHUNT | |
| `Meters.Elecon` | ELM (by id) | AC300 / AC3kW / AC5kW |
| `Temperatures.Elecon` | TEMP (by id) | board, office, kidroom, outside |
| `System` | SCOM | NETWORK_STATUS, tunnel only |

Read: `WorkshopEnergy` (GEMON), `Meters.IoboardD` kitchen power (AC), own `Mppt` (LEMON, exterior light).
Variables with `on_can: false` go to the RPi only through the VARS2 tunnel (`SCOM.c`, `RPISERP.c`).

## Battery packs — names vs ids

ASW_ELECON controls **2 house packs**; the third pack (workshop) is managed by **ASW_ELECON_D** (node 7).

| LEMON (`LEMON.c`) | UART | Vars2 group | Ids (old names) |
|---|---|---|---|
| `mBms80`, `Cfg.Pack80_Ah = 80` | UART3 | `PackHome80Ah` | 35–39, 350–381 (`BMS3_*`) |
| `mBms220`, `Cfg.Pack220_Ah = 220` | UART2 | `PackHome220Ah` | 30–34, 220–251 (`BMS2_*`) |
| — (ELECON_D) | — | `PackWorkshop251Ah` | 20–24, 180–211 (`BMS1_*`) |

The old names `BMS1/2/3` stay as variable names (ids, UhaGatePy, PC tool); the C names follow the pack.
