# 4WD Precision Motor Control — Plan Index

Canonical status and document map for this project. Source of truth for *where*
information lives; the details live in the linked documents.

## Current Status

| Item | State |
|---|---|
| Plan documents | `generated` (2026-06-23, not yet reviewed) |
| Implementation | **Phase 1 PWM motor bring-up verified** (2026-06-26) — all four motors run forward/reverse on TIM1-TIM4 PWM at 20 kHz; project rebuilt for HSI-derived 64 MHz with no HSE |
| Next action | Resolve the I2C1 remap vs RR motor pin conflict before wiring MPU-6050 / LCD / VL53L1X on the shared I2C bus |

The build proceeds one capability phase at a time. Each phase opens with a
**hardware-connection gate** (user wires the parts) and a **CubeMX boilerplate
step** before any driver code is written. See [[PHASES]] for the gate workflow.

## Document Map

| Document | Holds |
|---|---|
| [[OVERVIEW]] | Project goal, scope, constraints |
| [[PHASES]] | Capability phases + per-phase gate→CubeMX→code→verify procedure |
| [[ARCHITECTURE]] | Directory layout, main control-loop call graph, per-module function contracts |
| [[USER]] | **Canonical pin map / wiring**, hardware-gate connection steps, CubeMX + flash commands |
| [[DECISIONS]] | Decision history, source corrections, benchmark references |
| [[REVIEW]] | Engineering-goal acceptance metrics, per-phase verification, fallbacks |

## Open Questions (unresolved)

- I2C1 PB6/PB7 caused interference in the current shield/NUCLEO stack and must
  not be used. I2C1 remap PB8/PB9 is the only F103 I2C1 alternate pin pair, but
  those pins are currently RR motor TIM4_CH3/CH4; RR motor pins must be moved or
  the project must switch to another I2C instance before Phase 3.
