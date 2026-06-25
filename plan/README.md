# 4WD Precision Motor Control — Plan Index

Canonical status and document map for this project. Source of truth for *where*
information lives; the details live in the linked documents.

## Current Status

| Item | State |
|---|---|
| Plan documents | `generated` (2026-06-23, not yet reviewed) |
| Implementation | **Phase 1 hardware verified** (2026-06-24) — all four motors run; Hall/shock/XSHUT pins resolved; LL EXTI generation and mixed HAL/LL build verified |
| **Phase 2 verified** (2026-06-24) | HC-06 Bluetooth link and parser transport verified on hardware. Phase 6 now owns command actuation: FWD/LEFT/RIGHT drive FSM motion, STOP→IDLE, MAN→MANUAL, and RST clears a released emergency back to IDLE. |
| **Phase 3 verified** (2026-06-25) | MPU-6050 responds at 0x68 with WHO_AM_I=0x68 and produces a stable YAW stream. At-rest drift acceptance MET on hardware: two 60 s USART2 captures measured +0.01 and -0.14 °/min (well under the <1 °/min criterion), noise p2p 0.09–0.67°. |
| **Phase 4 implemented** (2026-06-25) | Encoder + 4× speed PID wired into the 100 Hz control tick (Encoder_Sample → PID_Update → Motor_SetDuty); motor driver converted to two-PWM H-bridge duty control (Motor_SetDuty, ±3599). Build green. PENDING hardware: speed-tracking error <5% verify + SPEED_KP/KI/KD and ENCODER_COUNTS_PER_REV tuning. |
| **Phase 6 implemented** (2026-06-25) | Seven-state FSM, deferred HC-06 command processing, latched PA6 shock emergency, and bounded PCF8574 LCD status output implemented without a buzzer/PC0. Clean build, review fixes, flash verification, and MPU runtime regression passed. PENDING hardware: LCD and shock-sensor behavior. |
| Next action | Connect the Phase 6 LCD and shock sensor, then verify LCD state/yaw output, emergency motor stop, and AT+RST recovery. Phase 4 speed tuning remains pending. |

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

- _(none open)_ — motor speed modulation resolved in Phase 4: two-PWM H-bridge
  duty via `Motor_SetDuty` (±3599, 20 kHz); see [[ARCHITECTURE]] motor actuator.
