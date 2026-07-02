# Review, QA & Acceptance

Canonical home for the engineering-goal acceptance metrics and per-phase
verification approach. [[PHASES]] verify-steps reference the metrics here.

## Engineering-Goal Acceptance Metrics

| Metric | Baseline | Target | Verified in |
|---|---|---|---|
| Straight-line lateral deviation (1 m run) | no PID | **< 3 cm** | Phase 5 / 9 |
| 90° turn angular error | timer method ±8° | gyro integration **< ±2°** | Phase 5 / 9 |
| Wheel speed tracking error | open-loop PWM duty drive | **< 5%** with Hall PID after tuning | Phase 4 |
| Distance measurement error (VL53L1X) | HC-SR04 ±3 mm | **< ±1 mm** at 20 cm | Phase 8 |
| Kalman vs complementary yaw drift | complementary baseline | **< 1°/min** at rest | Phase 3 / 9 |

## Per-Phase Verification

- **Phase 0** — project builds; empty firmware flashes and runs.
- **Phase 1** — verified 2026-06-26 on hardware: each motor stops/coasts at duty
  0, runs forward from positive signed PWM duty on its first shield input, and
  runs reverse from negative duty on its second; no command drives both PWM
  directions non-zero for the same motor.
- **Phase 2** — AT-command loopback echoes; phone pairs; each command maps to the
  expected `FSM_SetState`.
- **Phase 3** — 500 stationary samples collected over USART2; variance recorded as
  `R`; yaw stream stable. Kalman drift metric measured.
- **Phase 4** — RPM/speed read per wheel; speed-tracking error < 5%. _Code
  complete (commits c92e8ea/9464ac5/813ed5c/4eda73e; encoder reimplemented as
  period-based RPM, single magnet/wheel, in c3236b3). A duty↔RPM feedforward
  plus narrow-range trim-PID was added on top (uncommitted; see [[REVIEW]]
  §Phase 4 Feedforward Calibration). HW duty-sweep calibration, feedforward
  coefficient fit, and the < 5% measurement are still pending._
- **Phase 5** — 1 m straight deviation < 3 cm; 90° turn error < ±2°. _Code
  complete (commit 8e4f2ef); yaw gains, turn angle, ramp limits, and correction
  sign remain unverified on hardware._
- **Phase 6** — LCD shows live state/sensors; manual toggle works; shock EXTI
  forces EMERGENCY from any state; reset returns to IDLE; buzzer fires. _Code
  complete (commit 8eb02af); unverified on hardware._
- **Phase 7** — line follow stays on track around a test loop. _Code complete
  (commit 9b3d18d); line PID/sign/calibration remain unverified on hardware._
- **Phase 8** — 3 sensors enumerate on 0x54/0x56/0x58 after XSHUT address
  assignment; distance error < ±1 mm at 20 cm; obstacle triggers S-curve decel →
  TURN → STRAIGHT. _Code complete (commit 6f00a92); ranging and thresholds remain
  unverified on hardware._
- **Phase 9** — all metrics above re-measured end-to-end; Kalman-vs-complementary
  drift comparison graph produced from USART2→CSV data.

## Phase 4 Bench Tuning — speed telemetry

Firmware emits a per-wheel telemetry line on **USART2** every `STREAM_PERIOD_MS`
(50 ms), alongside the existing `YAW=` line (bench: HC-06 detached, USB-TTL on
PA2/PA3 — [[DECISIONS]] §D3):

```
SPD=sLF,mLF,sRF,mRF,sLR,mLR,sRR,mRR;CNT=cLF,cRF,cLR,cRR
```

- `s*`/`m*` = commanded vs measured speed in **deci-RPM** (÷10 → RPM); order LF, RF, LR, RR.
- `c*` = cumulative Hall pulse count (`Encoder_GetCount`), same order.

`m` is derived from one ~10 ms (`SPEED_PERIOD_MS`) pulse-delta window, so its
quantum is `6000 / ENCODER_COUNTS_PER_REV` RPM (≈300 RPM at the `20.0f`
placeholder). Treat `m` as a **liveness/direction** readout only — it cannot
resolve a 5% error. Use `CNT` deltas for the actual metric.

**Procedure:**
1. **Validate encoder scale first.** In IDLE (motors off), hand-rotate one wheel
   exactly N turns; `counts/rev = ΔCNT / N`. If it differs from
   `ENCODER_COUNTS_PER_REV` (`20.0f`), correct the constant and reflash — RPM is
   meaningless until this is right ([[DECISIONS]] §D11).
2. **Measure + tune.** Command FORWARD (80 RPM setpoint). Compute steady-state
   average RPM per wheel from `CNT` over a multi-second window Δt:
   `RPM_avg = (ΔCNT / ENCODER_COUNTS_PER_REV) / Δt × 60` (a few seconds so ±1
   count is well under 5%); error = `|RPM_avg − 80| / 80`. Adjust `SPEED_KP/KI/KD`
   (`main.c`, currently 5/0/0), rebuild + reflash, repeat until all four wheels
   are < 5%.

Gains are compile-time (no runtime gain command); each iteration is a reflash.
The P-only loop (`Kp=5, Ki=0`) runs on the same coarse per-tick RPM and is
expected to jitter/steady-state-droop — tune around it (add `Ki`) rather than
treating the jitter as a defect.

## Phase 4 Feedforward Calibration — duty↔RPM sweep

To remove the structural steady-state droop of the pure-P speed loop above,
`main.c`'s speed loop now computes `duty = SpeedFF_Duty(setpoint) + PID_trim`:
a per-wheel linear feedforward `offset + gain·|RPM|` (`speed_ff_offset[]`,
`speed_ff_gain[]`, currently placeholder `800.0f`/`20.0f` for all four wheels)
supplies most of the duty, and `pid_speed` (now clamped to a narrow **±800**
trim range, was ±3599) only corrects the residual error.

**Calibration procedure (compile-time flags, reflash per run):**
1. Set `MOTOR_TEST_ON_BOOT` to `0U` and `MOTOR_SWEEP_ON_BOOT` to `1U` in
   `main.c` (the two are mutually exclusive — a `#error` enforces this),
   rebuild, reflash.
2. On boot, `MotorSweep_Run` drives each wheel forward-only from
   `MOTOR_SWEEP_MIN_DUTY` (800) to `MOTOR_SWEEP_MAX_DUTY` (3199) in
   `MOTOR_SWEEP_STEP` (200) increments, holding each duty for
   `MOTOR_SWEEP_HOLD_MS` (1500 ms) before sampling `Encoder_GetSpeed`, and
   streams over USART2: `SWEEP_WHEEL=<name>` then
   `SWEEP=<name>,<duty>,<rpm×1000>` per step.
3. Per wheel, fit `RPM ≈ (duty − offset) / gain` (linear regression on the
   captured `SWEEP=` points) and replace the placeholder `speed_ff_gain[]`/
   `speed_ff_offset[]` values in `main.c` with the fitted per-wheel results.
4. Revert `MOTOR_SWEEP_ON_BOOT` to `0U`, reflash, and re-run the Phase 4 bench
   tuning procedure above to confirm the < 5% speed-tracking target with the
   tuned feedforward + trim.

Status: code added and host-build-verified (`cmake --build build/Debug`, no
warnings); the sweep has not been run on hardware yet, and
`speed_ff_gain[]`/`speed_ff_offset[]` remain unfit placeholders.

## Data Collection

- Yaw / gyro / Kalman samples stream over **USART2** (bench, HC-06 detached —
  [[DECISIONS]] §D3) → captured to CSV on the host for offline comparison.
- Once HC-06 occupies USART2, on-target streaming is unavailable; rely on
  pre-collected CSVs and the LCD readout.

## Fallbacks

- **Debug path lost after HC-06 attach** — collect tuning data on USART2 *before*
  attaching HC-06; thereafter use LCD status. (No spare debug UART — [[DECISIONS]]
  §D3.)
- **Hardware I2C1 unusable; use software I2C** — enabling I2C1 drags `I2C1_SMBA`
  onto PB5 and kills RF-reverse PWM ([[DECISIONS]] §D10). All hardware I2C
  placements collide (I2C1 PB6/7→SMBA on PB5; I2C1 remap PB8/9 = RR motor; I2C2
  PB10/11 = LR-reverse). The sensor bus is therefore a **software bit-bang I2C on
  PB6/PB7** (`soft_i2c`); never re-enable the hardware I2C1 peripheral on this stack.
- **VL53L1X RAM footprint too large** — current Debug build links at 6152 B RAM
  and 60004 B FLASH with the STSW-IMG007 API. If later ranging options or modules
  threaten the budget, reduce ranging config or sensor count.
- **Turn drift from gyro integration** — fall back to the timer method baseline
  (±8°) if gyro integration underperforms, and record the gap.

## Review Gate

Per project protocol, each implemented phase is reviewed by the `code-reviewer`
subagent in parallel with `codex:review` before its status is advanced toward
`verified` in [[README]]. `verified` requires a specific passing build/flash/
observation, not just compilation.
