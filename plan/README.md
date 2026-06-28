# 4WD Precision Motor Control — Plan Index

Bootstrap and fallback entry point only. This file exists so a new session knows
*what to ask plan-rag about* — it carries the current blocker and any undecided
user calls, plus a map of where everything else lives. Retrieve all other facts
(status detail, phases, architecture, decisions, wiring, QA) through the
`plan-rag` MCP tools, not from here.

## Now — Current Blocker / Pending User Decision

- **I2C1 vs RR-motor pin conflict (decide before Phase 3).** I2C1 PB6/PB7 caused
  interference on the current shield/NUCLEO stack and must not be used. I2C1
  remap PB8/PB9 is the only F103 I2C1 alternate pair, but those pins are
  currently RR motor TIM4_CH3/CH4. Decide: move the RR motor pins, or switch to
  another I2C instance. Until resolved, do not wire MPU-6050 / LCD / VL53L1X on
  the shared I2C bus.

## Document Map

| Document | Holds |
|---|---|
| [[OVERVIEW]] | Project goal, scope, constraints |
| [[PHASES]] | Capability phases + per-phase gate→CubeMX→code→verify procedure |
| [[ARCHITECTURE]] | Directory layout, control-loop call graph, module contracts |
| [[USER]] | Canonical pin map / wiring, hardware-gate steps, CubeMX + flash commands |
| [[DECISIONS]] | Decision history, source corrections, benchmark references |
| [[REVIEW]] | Acceptance metrics, per-phase verification status, fallbacks |
