# Architecture

Behavioral contracts only — no implementation bodies (per
`.claude/rules/Edit_Workflow.md`). Struct field layouts live in the headers; this
file states each unit's responsibility, I/O, and call connections.

## Directory Layout

```
Inc/   main.h motor.h mpu6050.h kalman.h pid.h encoder.h
       paramstore.h soft_i2c.h hc06.h lcd.h fsm.h scurve.h
Src/   main.c motor.c mpu6050.c kalman.c pid.c encoder.c
       paramstore.c soft_i2c.c hc06.c lcd.c fsm.c scurve.c
Drivers/
└── VL53L1X/   vl53l1_api.{h,c}   (ST official API, UM2356)
```

## Entry Points & Control-Loop Call Graph

The system has one synchronous super-loop plus three asynchronous ISR entry
points. The active control path is rooted in the main loop after `dt` is computed;
do not read the per-wheel PID, S-curve, or FSM state handlers as orphaned.

### `main()` — boot + super loop
- Init order: CubeMX peripheral init → `Motor_Init` → `Encoder_Init` → `FSM_Init`
  → `HC06_Init` → `SoftI2C_Init` → `MPU6050_Init` → `Kalman_Init` → speed PID init
  → `LCD_Init`.
- Then loops; `HC06_Process` and pending parameter saves are checked every pass,
  while sensor/control/display work is paced from `HAL_GetTick()`.

### Control tick — orchestrator

| Step | Calls | Cadence |
|---|---|---|
| Bluetooth/service | `HC06_Process`, parameter-save request handling | every super-loop pass |
| Timebase | compute `dt` from `HAL_GetTick()` | every control pass with `dt > 0` |
| Speed loop | `Encoder_Sample` → `Encoder_GetSpeed` ×4 → `PID_Update(pid_speed)` ×4 → `Motor_SetDuty` ×4 | each `SPEED_PERIOD_MS` |
| IMU + yaw estimate | `MPU6050_Read` → `Kalman_Update` | every control pass |
| FSM/profile/state | `FSM_Dispatch(yaw, omega_dps, dt)`; STRAIGHT calls `SCurve_Update` + `PID_Update(pid_yaw)` | every control pass after fresh yaw/gyro data |
| Telemetry | `YAW=`, `SPD=`, `CNT=` over USART2 | each `STREAM_PERIOD_MS` |
| Display | `LCD_Update` | each `LCD_PERIOD_MS` |

### ISR entry points
- **LL EXTI Hall (PB0/PB1/PB2/PA4)** → clear the pending line, then
  `Encoder_OnPulse` (per wheel).
- **LL EXTI Shock (PA6)** → clear the pending line, then
  `FSM_RequestEmergencyFromISR` — latches emergency and stops motors if the FSM is initialized.
- **USART2 DMA RX** → `HC06_OnReceive`; command parsing is drained by `HC06_Process`.

---

## Modules

### motor — `motor.{h,c}`
Owns the four H-bridge PWM direction pairs. Positive signed duty drives the
forward input, negative signed duty drives the reverse input, and zero duty
coasts/stops with both compare values at 0. Phase 1 uses open-loop duty; later
PID phases can feed this signed-duty actuator.

| Fn | Responsibility | Input | Output / effect | Connections |
|---|---|---|---|---|
| `Motor_Init` | Start all PWM channels and set compares to 0 | — | all motors stopped/coasting | called by `main` |
| `Motor_SetDuty` | Clamp signed duty and write exactly one compare channel non-zero | `id` (LF/RF/LR/RR), signed duty | one motor forward/reverse/coast | called by speed/drive logic; calls HAL TIM compare macros |
| `Motor_SetDirection` | Convenience full-scale forward/reverse/stop wrapper | `id` (LF/RF/LR/RR), direction | one motor forward/reverse/stopped | called by direct test/manual helpers; calls `Motor_SetDuty` |
| `Motor_SetAll` | Apply one direction to all four motors | direction | all motors updated | calls `Motor_SetDirection` ×4 |
| `Motor_StopAll` | Set every motor signed duty to 0 | — | all motors stopped/coasting | called by init/emergency/idle paths |

### pid — `pid.{h,c}`
Generic PID with anti-windup. Three uses: `pid_yaw`, `pid_speed` (×4), `pid_line`.
See [[OVERVIEW]] for what each corrects.

| Fn | Responsibility | Input | Output | Connections |
|---|---|---|---|---|
| `PID_Update` | One PID step with integral clamp + output clamp | `pid*`, setpoint, measurement, `dt` | control output (float) | called by FSM straight (`pid_yaw`), speed loop (`pid_speed`), line handler (`pid_line`) |
| `PID_Reset` | Clear integral + prev_error | `pid*` | effect: state zeroed | called by FSM/speed-loop transition and disabled-output paths |

### kalman — `kalman.{h,c}`
1-D Kalman filter fusing gyro rate + accel-derived angle into a yaw estimate.

| Fn | Responsibility | Input | Output | Connections |
|---|---|---|---|---|
| `Kalman_Update` | Predict (angle += ω·dt, P += Q) then update (K = P/(P+R)) | `accel_angle`, `omega_dps`, `dt` | estimated angle (deg) | called by control tick; ω/accel from `MPU6050_Read`; output consumed by `FSM_Dispatch` |

`R` measured from stationary samples; bias/R can be persisted through `paramstore`.

### soft_i2c — `soft_i2c.{h,c}`
Bit-bang I2C master on PB6/PB7. This is the permanent sensor/display bus because
hardware I2C conflicts with motor PWM on this pin stack (see [[DECISIONS]] §D10).

| Fn | Responsibility | Input | Output | Connections |
|---|---|---|---|---|
| `SoftI2C_Init` | Configure the software I2C pins/open-drain behavior | — | bus ready | called by `main` before I2C devices |
| `SoftI2C_*` transactions | Start/address/read/write/stop byte-level transfer | address/register/data | I2C transfer status/data | called by `mpu6050`, `lcd`, and later VL53L1X wrapper |

### paramstore — `paramstore.{h,c}`
Flash-backed persistence for tunable Kalman parameters.

| Fn | Responsibility | Input | Output | Connections |
|---|---|---|---|---|
| `ParamStore_Load` | Load persisted Kalman R/bias if present | out params | success flag | called by `main` after `Kalman_Init` |
| `ParamStore_Save` | Persist requested Kalman R/bias | R, bias | success flag | called by main-loop save request handling |

### mpu6050 — `mpu6050.{h,c}`
MPU-6050 driver on the software I2C bus (raw gyro + accel).

| Fn | Responsibility | Input | Output | Connections |
|---|---|---|---|---|
| `MPU6050_Init` | Wake device, set gyro/accel ranges | — | effect: device configured | called by `main`; calls `soft_i2c` |
| `MPU6050_Read` | Read gyro Z rate + compute accel tilt angle | — | `omega_dps`, `accel_angle` (via out-params) | called by control tick; calls `soft_i2c`; feeds `Kalman_Update` |

### encoder — `encoder.{h,c}`
DIY Hall-sensor wheel encoder. RPM from pulse count over a fixed sample window:
`RPM = (Δpulses / ENCODER_COUNTS_PER_REV) / dt · 60`. `ENCODER_COUNTS_PER_REV`
is Hall magnets × 2 (both EXTI edges counted); placeholder `20.0f`, tune to
hardware.

| Fn | Responsibility | Input | Output | Connections |
|---|---|---|---|---|
| `Encoder_OnPulse` | Increment cumulative pulse count for one wheel | `wheel` | effect: `pulse_count[wheel]++` | called by LL EXTI Hall ISR |
| `Encoder_Sample` | Per-window: compute RPM from pulse-count delta | `dt` | effect: `speed_rpm[wheel]` updated | called by speed loop each `SPEED_PERIOD_MS` |
| `Encoder_GetSpeed` | Return latest speed for one wheel | `wheel` | speed (**RPM**) | called by speed loop; feeds `PID_Update(pid_speed)` |
| `Encoder_GetCount` | Return cumulative pulse count (encoder-scale validation / telemetry) | `wheel` | pulses (uint32) | read by USART2 `CNT=` telemetry |

### scurve — `scurve.{h,c}`
Jerk-limited S-curve speed profile over a speed setpoint. It is standalone,
allocation-free, and stores current speed, current acceleration, target,
acceleration limit, jerk limit, and phase.

| Fn | Responsibility | Input | Output | Connections |
|---|---|---|---|---|
| `SCurve_Init` | Reset profile at 0 speed and arm ACCEL phase | `sc*`, target, `accel_max`, `jerk` | effect: profile armed with absolute accel/jerk limits | called by FSM straight entry |
| `SCurve_Update` | Advance one tick; rate-limit acceleration by jerk and clamp at target | `sc*`, `dt` | current speed setpoint | called by `FSM_Straight_Update`; output feeds `FSM_SetSideTargets` |

### fsm — `fsm.{h,c}`
Central 7-state machine. Holds current state + per-state context. It publishes
per-wheel RPM setpoints for the speed PID loop; only idle/emergency/reset paths
stop motors directly.

**States:** `IDLE · STRAIGHT · LINE_TRACE · TURN · AVOID · MANUAL · EMERGENCY`

**Transitions:**
```
IDLE       → STRAIGHT    : auto start / AT+FWD
IDLE       → LINE_TRACE  : line detected (tracking module)
IDLE       → MANUAL      : HC-06 manual command
STRAIGHT   → TURN        : AT+LEFT / AT+RIGHT
STRAIGHT   → AVOID       : obstacle (VL53L1X; future)
LINE_TRACE → AVOID       : obstacle (VL53L1X; future)
TURN       → STRAIGHT    : integrated |ω|·dt ≥ θ_goal
AVOID      → TURN        : start avoidance rotation (future)
AVOID      → STRAIGHT    : avoidance complete (future)
MANUAL     → IDLE        : AT+STOP / disconnect path
ANY        → EMERGENCY   : shock EXTI — overrides all
EMERGENCY  → IDLE        : BT AT reset, only if shock input is clear
```

| Fn | Responsibility | Input | Output / effect | Connections |
|---|---|---|---|---|
| `FSM_Init` | Initialize state/context and yaw PID placeholder gains | — | effect: state=IDLE unless emergency latched | called by `main` |
| `FSM_SetState` | Transition + reset relevant per-state context | target state | effect: state changed; STRAIGHT re-arms yaw capture; TURN clears accumulator | called by `HC06_Parse`, shock/reset paths, handlers |
| `FSM_Dispatch` | Run current state's handler after fresh IMU/Kalman update | `yaw`, `omega_dps`, `dt` | effect: speed setpoints updated or motors stopped | called by control tick |
| `FSM_Straight_Update` | Yaw-PID straight drive with S-curve base RPM | yaw est, dt | effect: left/right RPM targets set to `base ± corr` | calls `SCurve_Update`, `PID_Update(pid_yaw)`, `FSM_SetSideTargets` |
| `FSM_Turn_Update` | Gyro-integrated pivot turn | `omega_dps`, `dt`, implicit 90° placeholder target | effect: pivot RPM targets; on done stop + →STRAIGHT | calls `FSM_ApplyMotion`, `FSM_SetState(STRAIGHT)` |
| `FSM_ApplyMotion` | Map forward/left/right/stop to per-wheel RPM targets | motion | effect: `speed_setpoint[]` updated | used by TURN and MANUAL paths |
| `FSM_IsMotionEnabled` | Tell the speed loop whether PID output may drive motors | — | bool | true for STRAIGHT/TURN and non-stop MANUAL, false for idle/fail-safe states |
| `FSM_GetSpeedSetpoint` | Return one wheel's current RPM target | wheel | RPM setpoint | consumed by speed PID loop |

`LINE_TRACE` and `AVOID` currently fail safe by zeroing targets until their sensor
modules land. Phase 5 yaw gains, turn angle, S-curve accel/jerk limits, and yaw
correction sign are tuning placeholders until hardware verification.

### hc06 — `hc06.{h,c}`
USART2 Bluetooth AT-command interface. Commands: `AT+FWD`→STRAIGHT,
`AT+LEFT`/`AT+RIGHT`→TURN 90°, `AT+STOP`→IDLE, `AT+MAN`→MANUAL, `AT+RST`→EMERGENCY
reset.

| Fn | Responsibility | Input | Output / effect | Connections |
|---|---|---|---|---|
| `HC06_Init` | Start USART2 DMA RX into static line buffer | — | effect: RX armed | called by `main`; calls HAL UART/DMA |
| `HC06_OnReceive` | Accumulate bytes into a static RX buffer | rx byte/buf | effect: buffer filled / line queued | called by USART2 ISR or DMA callback path |
| `HC06_Process` | Drain completed command lines | — | effect: queued commands parsed | called by main loop |
| `HC06_Parse` | Map an AT command string to a state change | `cmd` (const char*) | effect: state change | called by `HC06_Process`; calls `FSM_SetState`/`FSM_SetDirection` |

### lcd — `lcd.{h,c}`
I2C character LCD (PCF8574 backpack) status display on the software I2C bus.

| Fn | Responsibility | Input | Output / effect | Connections |
|---|---|---|---|---|
| `LCD_Init` | Init LCD in 4-bit/I2C mode | — | effect: LCD ready | called by `main`; calls `soft_i2c` |
| `LCD_Update` | Render state + key sensor values | state name, yaw, MPU status | effect: LCD refreshed | called by control tick at `LCD_PERIOD_MS`; calls `soft_i2c` |

### vl53l1x — `vl53l1x.{h,c}`
Wrapper over the ST VL53L1X API for 3 multi-drop sensors (`TOF_FRONT/LEFT/RIGHT`).
Not implemented yet; it will use the same software I2C bus.

**XSHUT multi-drop address sequence (canonical):**
```
Default I2C addr 0x52. With all XSHUT LOW (all off):
  1. Release XSHUT_FRONT → power Front only → set addr 0x54
  2. Release XSHUT_LEFT  → power Left only  → set addr 0x56
  3. Release XSHUT_RIGHT → Right keeps default 0x52
```
(Pattern confirmed against the references in [[DECISIONS]] §D1.)

| Fn | Responsibility | Input | Output | Connections |
|---|---|---|---|---|
| `TOF_Init_All` | Run XSHUT sequence + assign 3 addresses + ST-API init | — | effect: 3 sensors ranging on distinct addrs | called by `main`/Phase 8; drives XSHUT GPIO, calls `VL53L1X_SetI2CAddress` + ST API |
| `TOF_ReadDistance_mm` | Read one sensor's distance | `id` | distance (mm) | called by control tick + `TOF_IsObstacle`; calls ST API ranging |
| `TOF_IsObstacle` | Threshold check for one direction | `id`, `threshold_mm` | bool | called by FSM straight/line handlers; calls `TOF_ReadDistance_mm` |
