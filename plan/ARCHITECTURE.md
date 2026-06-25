# Architecture

Behavioral contracts only — no implementation bodies (per
`.claude/rules/Edit_Workflow.md`). Struct field layouts live in the headers; this
file states each unit's responsibility, I/O, and call connections.

## Directory Layout

```
Inc/   main.h motor.h mpu6050.h kalman.h paramstore.h pid.h encoder.h
       vl53l1x.h hc06.h lcd.h fsm.h scurve.h
Src/   main.c motor.c mpu6050.c kalman.c paramstore.c pid.c encoder.c
       vl53l1x.c hc06.c lcd.c fsm.c scurve.c
Drivers/
└── VL53L1X/   vl53l1_api.{h,c}   (ST official API, UM2356)
```

## Entry Points & Control-Loop Call Graph

The system has one synchronous entry point (the main super-loop tick) plus three
asynchronous ISR entry points. **The 10 ms tick is the call-graph root** — every
controller below is reached from it; do not read the per-wheel PID or S-curve
functions as orphaned.

### `main()` — boot + super loop
- Init order: `Motor_Init` → `HC06_Init` → `MPU6050_Init` → `Encoder` setup →
  `FSM_Init` → (`LCD_Init`, `TOF_Init_All` as phases land).
- Then loops; on each 10 ms SysTick flag it runs **one control tick**.

### Control tick (100 Hz) — orchestrator
Runs every 10 ms, in order:

| Step | Calls | Cadence |
|---|---|---|
| Read IMU | `MPU6050_Read` → omega, accel-angle | every tick |
| Yaw estimate | `Kalman_Update` | every tick |
| Speed loop | `Encoder_GetSpeed` ×4 → `PID_Update(pid_speed)` ×4 → `Motor_SetDuty` (actuator) | every tick |
| Profile | `SCurve_Update` | every tick |
| State handler | `FSM_Dispatch` (runs current state's handler) | every tick |
| Ranging | `TOF_ReadDistance_mm` / `TOF_IsObstacle` | every 5 ticks (50 ms) |
| Display | `LCD_Update` | every 20 ticks (200 ms) |

### ISR entry points
- **LL EXTI Hall (PB0/PB1/PB2/PA4)** → clear the pending line, then
  `Encoder_Update` (per wheel).
- **LL EXTI Shock (PA6)** → clear the pending line, then
  `FSM_SetState(EMERGENCY)` — overrides all states.
- **USART2 DMA RX** → `HC06_OnReceive` → `HC06_Parse` on line terminator.

---

## Modules

### motor — `motor.{h,c}`
Owns the four H-bridge direction pairs. Speed modulation is implemented via `Motor_SetDuty` (see below); the direction helpers now delegate to it.

| Fn | Responsibility | Input | Output / effect | Connections |
|---|---|---|---|---|
| `Motor_Init` | Start all 8 PWM channels, set every duty to 0 | — | all motors stopped | called by `main`; calls HAL_TIM_PWM_Start ×8 |
| `Motor_SetDirection` | Stop one H-bridge pair, then assert forward or reverse | `id` (LF/RF/LR/RR), direction | one motor forward/reverse/stopped | called by FSM handlers; delegates to `Motor_SetDuty` |
| `Motor_SetAll` | Apply one direction to all four motors | direction | all motors updated | called by FSM turn/stop helpers; calls `Motor_SetDirection` ×4 |
| `Motor_StopAll` | Coast every motor (duty 0) | — | all motors stopped | called by init/emergency/idle paths; calls `Motor_SetDuty` ×4 |

### motor actuator — `Motor_SetDuty` in `motor.c`
Signed duty actuator for speed modulation. Each motor is a two-PWM H-bridge driven
by two TIM channels (FWD channel + REV channel). Mapping: LF fwd=TIM2_CH2/rev=TIM1_CH3,
RF fwd=TIM3_CH1/rev=TIM3_CH2, LR fwd=TIM1_CH2/rev=TIM2_CH3, RR fwd=TIM4_CH3/rev=TIM4_CH4;
ARR=3599 all timers (20 kHz). `duty>0` → fwd=`|duty|`, rev=0; `duty<0` → fwd=0, rev=`|duty|`;
0 → both 0 (coast). `Motor_SetDirection`/`Motor_SetAll`/`Motor_StopAll` now delegate to
`Motor_SetDuty`.

| Fn | Responsibility | Input | Output / effect | Connections |
|---|---|---|---|---|
| `Motor_SetDuty` | Set signed PWM duty for one motor (clamped [-3599, +3599]) | `id`, `duty` (int16_t) | motor speed actuation | called by speed loop; drives two TIM channels per motor |

### pid — `pid.{h,c}`
Generic PID with anti-windup. Three instances: `pid_yaw`, `pid_speed` (×4),
`pid_line`. See [[OVERVIEW]] for what each corrects.

| Fn | Responsibility | Input | Output | Connections |
|---|---|---|---|---|
| `PID_Update` | One PID step with integral clamp + output clamp | `pid*`, `error`, `dt` | control output (float) | called by FSM straight (`pid_yaw`), speed loop (`pid_speed`), line handler (`pid_line`) |
| `PID_Reset` | Clear integral + prev_error | `pid*` | effect: state zeroed | called by `FSM_SetState` on transitions |

### kalman — `kalman.{h,c}`
1-D Kalman filter fusing gyro rate + accel-derived angle into a yaw estimate (2-state Lauszus angle+bias structure).

| Fn | Responsibility | Input | Output | Connections |
|---|---|---|---|---|
| `Kalman_Update` | Predict (angle += ω·dt, P += Q) then update (K = P/(P+R)) | `kf*`, `omega_dps`, `accel_angle`, `dt` | estimated angle (deg) | called by control tick; ω/accel from `MPU6050_Read`; output consumed by `pid_yaw` and `FSM_Turn_Update` |
| `Kalman_SetR` | Set measurement-noise variance (clamped to ≥1e-6f) | `kf*`, R (deg²) | effect: filter R updated | called by `ParamStore_Load` or AT+SAVE handler |
| `Kalman_GetR` | Query current R | `kf*` | R (deg²) | called by AT+GET handler |
| `Kalman_GetBias` | Query current gyro bias | `kf*` | bias (dps) | called by AT+GET handler |
| `Kalman_GetAngle` | Query current yaw estimate | `kf*` | angle (deg) | called by AT+GET handler |
| `Kalman_SetBias` | Set gyro bias state | `kf*`, bias | effect: internal bias updated | called by `ParamStore_Load` |

**R estimation:** runtime stationarity-based. Gate on gyro at rest (|ω_dps| < 1.0) but accumulate variance of `accel_angle` (deg²), not gyro rate. `Q` tuned ([[USER]] §empirical).

### mpu6050 — `mpu6050.{h,c}`
MPU-6050 I2C driver (raw gyro + accel).

| Fn | Responsibility | Input | Output | Connections |
|---|---|---|---|---|
| `MPU6050_Init` | Wake device, verify WHO_AM_I == 0x68, set gyro/accel ranges | — | effect: device configured; early return on failed WHO_AM_I | called by `main`; calls HAL I2C |
| `MPU6050_Read` | Read gyro Z rate + compute accel tilt angle | — | `omega_dps`, `accel_angle` (via out-params) | called by control tick; calls `HAL_I2C_Mem_Read`; feeds `Kalman_Update` |

### paramstore — `paramstore.{h,c}`
Persistent storage of Kalman R and gyro bias in internal Flash (last page 0x0801FC00, 1 KB on STM32F103RBTx).

Record layout: `{magic: 0x4B414C31, float R, float bias}` (16 bytes, fits in 1 KB page).

| Fn | Responsibility | Input | Output | Connections |
|---|---|---|---|---|
| `ParamStore_Load` | Read Flash page; validate magic + finite R/bias (≥1e-6f); restore to Kalman filter | — | effect: filter R/bias set from Flash or defaults on invalid/missing data | called by `main` after `Kalman_Init`; calls `Kalman_SetR`, `Kalman_SetBias` |
| `ParamStore_Save` | Write current Kalman R/bias + magic to Flash page (erase + program) | — | effect: page written (atomic from main loop, never ISR) | called by main loop when AT+SAVE flag is set (flag set by `HC06_Parse` in RX ISR) |

### encoder — `encoder.{h,c}`
DIY Hall-sensor wheel encoder. RPM from pulse period: `RPM = 60 / T_pulse_s`,
`speed = RPM·2π·r/60`.

| Fn | Responsibility | Input | Output | Connections |
|---|---|---|---|---|
| `Encoder_Update` | On Hall pulse, compute rpm/speed from tick delta | `enc*`, `wheel_radius_m` | effect: `enc.rpm`, `enc.speed_ms` updated | called by LL EXTI Hall ISR; reads TIM tick |
| `Encoder_GetSpeed` | Return latest speed for one wheel | `enc*` | speed (m/s) | called by speed loop; feeds `PID_Update(pid_speed)` |

### scurve — `scurve.{h,c}`
S-curve speed profile (ACCEL→CRUISE→DECEL→DONE) over a speed setpoint.

| Fn | Responsibility | Input | Output | Connections |
|---|---|---|---|---|
| `SCurve_Init` | Set target speed, accel rate, phase=ACCEL | `sc*`, target, `accel_rate` | effect: profile armed | called by FSM straight-start / avoid-decel |
| `SCurve_Update` | Advance one tick toward target, update phase | `sc*` | current speed setpoint | called by control tick; output feeds the speed controller |

### fsm — `fsm.{h,c}`
Central 7-state machine. Holds current state + per-state context.

**States:** `IDLE · STRAIGHT · LINE_TRACE · TURN · AVOID · MANUAL · EMERGENCY`

**Transitions:**
```
IDLE       → STRAIGHT    : auto start button
IDLE       → LINE_TRACE  : line detected (tracking module)
IDLE       → MANUAL      : HC-06 connected
STRAIGHT   → TURN        : turn cmd (BT AT)
STRAIGHT   → AVOID       : obstacle (VL53L1X)
LINE_TRACE → AVOID       : obstacle (VL53L1X)
TURN       → STRAIGHT    : θ ≥ θ_goal
AVOID      → TURN        : start avoidance rotation
AVOID      → STRAIGHT    : avoidance complete
MANUAL     → IDLE        : HC-06 disconnected
ANY        → EMERGENCY   : shock EXTI — overrides all
EMERGENCY  → IDLE        : BT AT reset
```

| Fn | Responsibility | Input | Output / effect | Connections |
|---|---|---|---|---|
| `FSM_Init` | Initialize IDLE or retain a pre-init emergency latch | — | effect: safe stopped state | called by `main`; calls `Motor_StopAll` |
| `FSM_SetState` | Apply a non-emergency state transition | target state | success/failure; stopped targets for safe/future states | called by deferred HC-06 processing |
| `FSM_SetDirection` | Map forward/left/right/stop to automatic or MANUAL motion | motion | effect: state/motion target updated | called by HC-06 command processing |
| `FSM_RequestEmergencyFromISR` | Latch EMERGENCY and stop motors immediately | — | effect: emergency dominates later dispatch | called by PA6 LL EXTI ISR |
| `FSM_ResetEmergency` | Clear EMERGENCY only when active-low PA6 is released and no EXTI is pending | — | success/failure; returns to IDLE on success | called by `AT+RST` processing |
| `FSM_Dispatch` | Refresh signed per-wheel speed targets for the current state | — | effect: setpoints updated; unsupported future states stop | called by control loop before speed PID |
| `FSM_Straight_Update` | Yaw-PID straight drive | yaw est, dt | effect: per-side motor command | calls `PID_Update(pid_yaw)` and `Motor_SetDuty`; checks `TOF_IsObstacle`→`FSM_SetState(AVOID)` |
| `FSM_LineTrace_Update` | Centroid-PID line follow | centroid error, dt | effect: inner/outer speed | calls `PID_Update(pid_line)` and `Motor_SetDuty`; checks `TOF_IsObstacle` |
| `FSM_Turn_Update` | Gyro-integrated pivot turn | `omega_dps`, `dt`, `target_deg` | effect: pivot; on done stop + →STRAIGHT | calls `Motor_SetDirection`/`Motor_StopAll`, `FSM_SetState(STRAIGHT)` |
| `FSM_Avoid_Update` | S-curve decel then →TURN, then →STRAIGHT | ToF, dt | effect: avoidance chain | calls `SCurve_Update`, `FSM_SetState(TURN/STRAIGHT)` |
| `FSM_Manual_Update` | Hold manual drive from last BT cmd | — | effect: motors per cmd | calls `Motor_*` |

### hc06 — `hc06.{h,c}`
USART2 Bluetooth AT-command interface. Commands: `AT+FWD`→STRAIGHT,
`AT+LEFT`/`AT+RIGHT`→TURN 90°, `AT+STOP`→IDLE, `AT+MAN`→MANUAL, `AT+RST`→EMERGENCY
reset, `AT+SAVE` (persist R/bias to Flash), `AT+GET` (return R, BIAS, YAW as scaled integers).

| Fn | Responsibility | Input | Output / effect | Connections |
|---|---|---|---|---|
| `HC06_Init` | Start USART2 DMA RX into static line buffer | — | effect: RX armed | called by `main`; calls HAL UART/DMA |
| `HC06_OnReceive` | Accumulate DMA bytes and publish one complete static command | rx byte | effect: pending command set; no blocking transmit/state change | called by USART2 DMA callback |
| `HC06_Process` | Consume and parse a pending command in main context | — | effect: FSM action, parameter output, or save flag | called before `FSM_Dispatch`; performs bounded command handoff |

### lcd — `lcd.{h,c}`
I2C character LCD (PCF8574 backpack) status display.

| Fn | Responsibility | Input | Output / effect | Connections |
|---|---|---|---|---|
| `LCD_Init` | Probe configurable PCF8574 address (default 0x27) and init 4-bit mode | — | availability flag; absent LCD returns safely | called by `main`; uses 5 ms I2C timeouts |
| `LCD_Update` | Render current FSM state and yaw/MPU status | state name, yaw, MPU status | effect: two 16-character lines refreshed | called every 200 ms when LCD is available |

### vl53l1x — `vl53l1x.{h,c}`
Wrapper over the ST VL53L1X API for 3 multi-drop sensors (`TOF_FRONT/LEFT/RIGHT`).

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
