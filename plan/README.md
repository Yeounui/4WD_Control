# 4WD Precision Motor Control — Plan Index

Canonical status and document map for this project. Source of truth for *where*
information lives; the details live in the linked documents.

## Current Status

| Item | State |
|---|---|
| Plan documents | `generated` (2026-06-23, not yet reviewed) |
| Implementation | **Phase 1 hardware verified** (2026-06-24) — all four motors run; Hall/shock/XSHUT pins resolved; LL EXTI generation and mixed HAL/LL build verified |
| **Phase 2 verified** (2026-06-24) | HC-06 manual comms; Bluetooth link + AT-command mapping confirmed on hardware (echo: FWD→STRAIGHT, LEFT/RIGHT→TURN, STOP→IDLE, MAN→MANUAL, RST→EMERGENCY, unknown→ERROR). Motor actuation deferred to FSM seam (Phase 6). |
| **Phase 3 implemented** (2026-06-24) | MPU-6050 IMU + 2-state Kalman yaw on I2C1; runtime stationarity-based R = Var(accel_angle) with Flash persistence (0x0801FC00) of R/bias; AT+SAVE/AT+GET added on USART2; yaw streamed as centi-degrees. Code-complete, build clean, review-passed (variance-floor + load-guard fix). PENDING hardware verification — MPU-6050 not yet wired. |
| **Phase 4 implemented** (2026-06-25) | Encoder + 4× speed PID wired into the 100 Hz control tick (Encoder_Sample → PID_Update → Motor_SetDuty); motor driver converted to two-PWM H-bridge duty control (Motor_SetDuty, ±3599). Build green. PENDING hardware: speed-tracking error <5% verify + SPEED_KP/KI/KD and ENCODER_COUNTS_PER_REV tuning. |
| Next action | Phase 4 hardware bring-up — tune speed-PID gains and ENCODER_COUNTS_PER_REV on the bench, confirm per-wheel tracking error <5%. |

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
