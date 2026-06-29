# 4WD Precision Motor Control — Plan Index

Bootstrap and fallback entry point only. This file exists so a new session knows
*what to ask plan-rag about* — it carries the current blocker and any undecided
user calls, plus a map of where everything else lives. Retrieve all other facts
(status detail, phases, architecture, decisions, wiring, QA) through the
`plan-rag` MCP tools, not from here.

## Now — Current State / Next Step

- **I2C↔RR-motor conflict: RESOLVED ([[DECISIONS]] §D10).** The sensor bus is a
  software bit-bang I2C on PB6/PB7 (`soft_i2c`); the hardware I2C1 peripheral is
  permanently disabled on this stack. The former "decide before Phase 3" blocker
  no longer applies.
- **Implemented in code:** Phases 1–7. Phase 7 line tracing adds
  `line_sensor.{h,c}` for ADC1-DMA sampling and normalized PA0 tracking error;
  `LINE_TRACE` now feeds `pid_line` output into left/right wheel RPM setpoints
  through the FSM. Phase 8 (VL53L1X obstacle avoidance) has no module yet.
- **HW-verified:** Phase 1 (2026-06-26), Phase 3 (2026-06-25). Phase 4 code is
  complete but **pending HW tuning** (per-wheel speed-tracking error < 5% not yet
  measured). Phases 5–7 code are complete but **unverified** on hardware.
- **Next step:** Phase 4 hardware tuning + < 5% speed-tracking verification, then
  tune Phase 5 yaw/turn/ramp parameters and Phase 7 line PID/sign/calibration on
  hardware.

## Document Map

| Document | Holds |
|---|---|
| [[OVERVIEW]] | Project goal, scope, constraints |
| [[PHASES]] | Capability phases + per-phase gate→CubeMX→code→verify procedure |
| [[ARCHITECTURE]] | Directory layout, control-loop call graph, module contracts |
| [[USER]] | Canonical pin map / wiring, hardware-gate steps, CubeMX + flash commands |
| [[DECISIONS]] | Decision history, source corrections, benchmark references |
| [[REVIEW]] | Acceptance metrics, per-phase verification status, fallbacks |
