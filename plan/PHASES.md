# Phases

The `plan.md` 9-day calendar is reorganized into dependency-ordered capability
phases (dates dropped — see [[DECISIONS]] §D6). Each phase that introduces
hardware opens with a **hardware-connection gate** the user performs, then
proceeds through CubeMX boilerplate → code → verify.

## Standard Phase Workflow

```
1. Hardware gate (USER)   → wire the parts for this phase; confirm power/ground.
                            Wiring checklist: [[USER]] §Hardware Connection Gates.
2. CubeMX boilerplate     → Claude authors/updates the .ioc for this phase's
   (CLAUDE)                 peripherals and runs STM32CubeMX headless to
                            regenerate the CMake project. See [[USER]] §CubeMX.
3. Code (CLAUDE)          → implement the phase's driver/logic inside
                            /* USER CODE BEGIN/END */ blocks (per orchestration
                            protocol: classify scope → call-codex / call-llama).
4. Verify                 → flash, observe on hardware, check the phase's
                            success criteria in [[REVIEW]].
```

> **Ownership:** the user only does the physical hardware gate (step 1). Claude
> does CubeMX generation (step 2), coding (step 3), and drives verification
> (step 4). See [[DECISIONS]] §D5.

---

## Phase 0 — Base Project & Bring-up
- **Gate (USER):** power board, connect ST-Link via SWD ([[USER]] §Phase 0).
  Confirm board identity (open question — no standard NUCLEO-F103RE).
- **CubeMX:** device **STM32F103RE**; HSE/PLL clock to 72 MHz; SysTick; toolchain
  CMake. Generate base project.
- **Code:** none beyond generated skeleton.
- **Verify:** project builds; an empty program flashes and runs (e.g. a heartbeat
  on a free GPIO if available). ST-Link attach via `usbipd-wsl` skill.

## Phase 1 — Servo PWM Drive (4WD)
- **Gate (USER):** 4 servo signal lines PA8–PA11, **separate servo power + common
  ground** ([[USER]] §Phase 1).
- **CubeMX:** TIM1 CH1–4 PWM, 50 Hz / 20 ms period, prescaler+ARR for µs
  resolution.
- **Code:** `servo.{h,c}` — `Servo_Init`, `Servo_SetPulse`, `Servo_SetAll`
  ([[ARCHITECTURE]] §servo).
- **Verify:** all 4 wheels stop at 1500 µs; forward/reverse track pulse width.

## Phase 2 — Bluetooth Manual Comms (HC-06)
- **Gate (USER):** HC-06 on USART2 (cross TX/RX), PA2/PA3 ([[USER]] §Phase 2).
- **CubeMX:** USART2 + DMA RX (also serves as bench debug UART, [[DECISIONS]] §D3).
- **Code:** `hc06.{h,c}` — DMA RX, `HC06_OnReceive`, `HC06_Parse` AT-command map
  ([[ARCHITECTURE]] §hc06).
- **Verify:** AT-command loopback; phone pairs and commands change state.

## Phase 3 — IMU + Kalman Yaw (MPU-6050)
- **Gate (USER):** MPU-6050 on I2C1 PB6/PB7 ([[USER]] §Phase 3).
- **CubeMX:** I2C1 (standard/fast mode).
- **Code:** `mpu6050.{h,c}` (raw gyro+accel), `kalman.{h,c}` (`Kalman_Update`).
- **Verify:** collect 500 stationary gyro samples over USART2 → variance = `R`;
  yaw output streams over USART2 ([[REVIEW]], [[USER]] §empirical).

## Phase 4 — Hall Encoder + Speed PID
- **Gate (USER):** 4 Hall sensors PB0–PB3 + wheel magnets; record magnets/rev
  ([[USER]] §Phase 4).
- **CubeMX:** EXTI0–3 — **set to LL driver** (not HAL), [[DECISIONS]] §D4.
- **Code:** `encoder.{h,c}` (`Encoder_Update` in LL ISR, `Encoder_GetSpeed`),
  `pid.{h,c}` with `pid_speed` ×4 in the control tick.
- **Verify:** RPM/speed read per wheel; per-wheel speed tracking error
  ([[REVIEW]]).

## Phase 5 — Straight Drive + S-Curve + Turn *(software only)*
- **Gate:** none (reuses servos + IMU + encoders).
- **CubeMX:** none.
- **Code:** `pid_yaw` straight correction in `FSM_Straight_Update`;
  `scurve.{h,c}`; `FSM_Turn_Update` gyro-integrated pivot ([[ARCHITECTURE]]).
- **Verify:** 1 m straight lateral deviation; 90° turn angular error ([[REVIEW]]).

## Phase 6 — FSM + LCD + Manual + Emergency
- **Gate (USER):** I2C LCD on I2C1; shock sensor PB4; buzzer PC0 ([[USER]] §Phase 6).
- **CubeMX:** GPIO PC0 output; EXTI4 — **LL driver**.
- **Code:** `fsm.{h,c}` (7-state machine + transitions + dispatch), `lcd.{h,c}`;
  wire MANUAL entry/exit and shock-EXTI → EMERGENCY ([[ARCHITECTURE]] §fsm).
- **Verify:** state + sensors show on LCD; manual mode toggles via HC-06; shock
  triggers EMERGENCY and overrides all states; buzzer sounds on emergency.

## Phase 7 — Line Tracing
- **Gate (USER):** tracking module analog out → PA0 ([[USER]] §Phase 7).
- **CubeMX:** ADC1 CH0 + DMA.
- **Code:** centroid calc → `pid_line` → inner/outer wheel speed; integrate
  `LINE_TRACE` state ([[ARCHITECTURE]] §fsm).
- **Verify:** car follows a line; centroid PID stable.

## Phase 8 — ToF Obstacle Avoidance (VL53L1X ×3)
- **Gate (USER):** 3 VL53L1X on shared I2C1, XSHUT PB8/PB9/PB10, bus pull-ups
  ([[USER]] §Phase 8).
- **CubeMX:** GPIO PB8–PB10 output (XSHUT). Add `Drivers/VL53L1X/` ST API to the
  CMake sources.
- **Code:** `vl53l1x.{h,c}` — `TOF_Init_All` (XSHUT address sequence),
  `TOF_ReadDistance_mm`, `TOF_IsObstacle`; `FSM_Avoid_Update` (S-curve decel →
  TURN → STRAIGHT). **Verify ST API static RAM footprint first** ([[OVERVIEW]]).
- **Verify:** 3 sensors report on distinct addresses; Front/Left/Right distances;
  obstacle triggers AVOID chain.

## Phase 9 — Full Integration + Data Collection
- **Gate:** none.
- **Code:** glue only; tuning.
- **Verify:** run all engineering-goal measurements in [[REVIEW]] — 1 m deviation,
  90° turn (timer vs gyro), avoidance scenario, Kalman vs complementary drift.
