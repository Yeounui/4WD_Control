# 4WD Precision Motor Control — Plan Index

Canonical status and document map for this project. Source of truth for *where*
information lives; the details live in the linked documents.

## Current Status

| Item | State |
|---|---|
| Plan documents | `generated` (2026-06-23, not yet reviewed) |
| Implementation | not started — no `Core/` sources generated yet |
| Next action | **Phase 0** — create the CubeMX base project for STM32F103RE (see [[PHASES]] §Phase 0) |

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

## Resolved Open Questions

- **MCU identity** — `plan.md` said `F103RE` but `SRAM 20 KB`. User confirmed the
  chip is **STM32F103RE** (64 KB SRAM / 512 KB Flash); the "20 KB" figure was a
  typo. See [[DECISIONS]] §D2.
- **Debug output path** — all debug pins named in `plan.md` collide with assigned
  functions. Resolved: reuse **USART2** for debug during bench tuning before HC-06
  is attached. See [[DECISIONS]] §D3.

## Open Questions (unresolved)

- **Board identity** — there is no standard NUCLEO-F103RE; the physical board
  (generic RET6 dev board vs. custom) affects the ST-Link connection and any
  on-board LED/button pins. Confirm before Phase 0 wiring. See [[USER]].
