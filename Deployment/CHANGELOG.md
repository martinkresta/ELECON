# ELECON firmware changelog

Newest version first. Versions before v001.11 are not listed.

## v001.11 (2026-11-01)

- Task: VARS2 step 1 (`UHA_COMMON/tasks/vars2_dual_transmission.md`), branch `vars2-step1`
- CRC: `0x2505BC5F`

- Sends all variables in the old format (`0x110+node`) and the new VARS2 format (`0x180+node`).
- Received VARS2 frames are stored in `Vars2`; control logic still uses the old variables.
- BMS1 data published as `VAR_BMS3_*` (`Core/Src/LEMON.c`, committed after this build).
