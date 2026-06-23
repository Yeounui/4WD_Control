# Review, QA & Acceptance

Canonical home for the engineering-goal acceptance metrics and per-phase
verification approach. [[PHASES]] verify-steps reference the metrics here.

## Engineering-Goal Acceptance Metrics

| Metric | Baseline | Target | Verified in |
|---|---|---|---|
| Straight-line lateral deviation (1 m run) | no PID | **< 3 cm** | Phase 5 / 9 |
| 90° turn angular error | timer method ±8° | gyro integration **< ±2°** | Phase 5 / 9 |
| Wheel speed tracking error | open-loop servo | **< 5%** with Hall PID | Phase 4 |
| Distance measurement error (VL53L1X) | HC-SR04 ±3 mm | **< ±1 mm** at 20 cm | Phase 8 |
| Kalman vs complementary yaw drift | complementary baseline | **< 1°/min** at rest | Phase 3 / 9 |

## Per-Phase Verification

- **Phase 0** — project builds; empty firmware flashes and runs.
- **Phase 1** — 4 servos stop at 1500 µs; pulse-width sweep moves each wheel
  forward/reverse as specified.
- **Phase 2** — AT-command loopback echoes; phone pairs; each command maps to the
  expected `FSM_SetState`.
- **Phase 3** — 500 stationary samples collected over USART2; variance recorded as
  `R`; yaw stream stable. Kalman drift metric measured.
- **Phase 4** — RPM/speed read per wheel; speed-tracking error < 5%.
- **Phase 5** — 1 m straight deviation < 3 cm; 90° turn error < ±2°.
- **Phase 6** — LCD shows live state/sensors; manual toggle works; shock EXTI
  forces EMERGENCY from any state; reset returns to IDLE; buzzer fires.
- **Phase 7** — line follow stays on track around a test loop.
- **Phase 8** — 3 sensors enumerate on 0x52/0x54/0x56; distance error < ±1 mm at
  20 cm; obstacle triggers S-curve decel → TURN → STRAIGHT.
- **Phase 9** — all metrics above re-measured end-to-end; Kalman-vs-complementary
  drift comparison graph produced from USART2→CSV data.

## Data Collection

- Yaw / gyro / Kalman samples stream over **USART2** (bench, HC-06 detached —
  [[DECISIONS]] §D3) → captured to CSV on the host for offline comparison.
- Once HC-06 occupies USART2, on-target streaming is unavailable; rely on
  pre-collected CSVs and the LCD readout.

## Fallbacks

- **Debug path lost after HC-06 attach** — collect tuning data on USART2 *before*
  attaching HC-06; thereafter use LCD status. (No spare debug UART — [[DECISIONS]]
  §D3.)
- **VL53L1X RAM footprint too large** — measure ST API static RAM before Phase 8
  integration; if it threatens the budget, reduce ranging config or sensor count.
- **Turn drift from gyro integration** — fall back to the timer method baseline
  (±8°) if gyro integration underperforms, and record the gap.

## Review Gate

Per project protocol, each implemented phase is reviewed by the `code-reviewer`
subagent in parallel with `codex:review` before its status is advanced toward
`verified` in [[README]]. `verified` requires a specific passing build/flash/
observation, not just compilation.
