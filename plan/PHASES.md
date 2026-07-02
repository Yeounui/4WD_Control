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
  Board: standard Nucleo-F103RB (on-board ST-Link/V2-1).
- **CubeMX:** device **STM32F103RB**; no HSE installed, so use HSI/2 × PLL16 =
  64 MHz; SysTick; toolchain CMake. Generate base project.
- **Code:** none beyond generated skeleton.
- **Verify:** project builds; an empty program flashes and runs (e.g. a heartbeat
  on a free GPIO if available). ST-Link attach via `usbipd-wsl` skill.

## Phase 1 — H-Bridge Direction Drive (4WD)
- **Gate (USER):** connect the shield's two direction inputs per motor using the
  canonical shield-pin/MCU-pin map in [[USER]] §Phase 1; motor supply and MCU
  share ground.
- **CubeMX:** eight TIM PWM outputs, ARR=3199 and prescaler=0 for 20 kHz at the
  64 MHz timer clocks. TIM3 uses partial remap for RF PB4/PB5.
- **Code:** `motor.{h,c}` — `Motor_Init`, `Motor_SetDuty`,
  `Motor_SetDirection`, `Motor_SetAll`, `Motor_StopAll`
  ([[ARCHITECTURE]] §motor).
- **Verify:** tested one wheel at a time on hardware (2026-06-26): LF/RF/LR/RR
  each stop at duty 0, run forward with positive duty, and run reverse with
  negative duty. Firmware never drives both directions with non-zero compare
  values for one motor.

## Phase 2 — Bluetooth Manual Comms (HC-06)
- **Gate (USER):** HC-06 on USART2 (cross TX/RX), PA2/PA3 ([[USER]] §Phase 2).
- **CubeMX:** USART2 + DMA RX (also serves as bench debug UART, [[DECISIONS]] §D3).
- **Code:** `hc06.{h,c}` — DMA RX, `HC06_OnReceive`, `HC06_Parse` AT-command map
  ([[ARCHITECTURE]] §hc06).
- **Verify:** AT-command loopback; phone pairs and commands change state.

## Phase 3 — IMU + Kalman Yaw (MPU-6050)
- **Gate (USER):** MPU-6050 on the resolved software I2C bus: SCL→PB6,
  SDA→PB7, AD0→GND ([[USER]] §Phase 3).
- **CubeMX:** no hardware I2C peripheral; keep I2C1 disabled. The bus is driven
  by `soft_i2c` because hardware I2C pin options conflict with the motor shield
  ([[DECISIONS]] §D10).
- **Code:** `mpu6050.{h,c}` (raw gyro+accel), `kalman.{h,c}` (`Kalman_Update`).
- **Verify:** collect 500 stationary gyro samples over USART2 → variance = `R`;
  yaw output streams over USART2 ([[REVIEW]], [[USER]] §empirical).

## Phase 4 — Hall Encoder + Speed PID
- **Gate (USER):** Hall sensors + wheel magnets; FL→PB0, FR→PB1, LR→PB2,
  RR→PA4; record magnets/rev ([[USER]] §Phase 4).
- **CubeMX:** EXTI0, EXTI1, EXTI2, EXTI4 with pull-ups and both-edge triggers;
  generate `MX_GPIO_Init` and the EXTI IRQ path with LL.
- **Code:** `encoder.{h,c}` (period-based RPM, single magnet/wheel:
  `Encoder_OnPulse` EXTI ISR measures pulse period, `Encoder_Sample`/
  `Encoder_GetSpeed` derive RPM), `pid.{h,c}` with `pid_speed` ×4 used as a
  narrow duty **trim** on top of a per-wheel duty→RPM feedforward
  (`SpeedFF_Duty` in `main.c`; see [[REVIEW]] §Phase 4 Feedforward Calibration).
- **Verify:** RPM/speed read per wheel; per-wheel speed tracking error
  ([[REVIEW]]).

## Phase 5 — Straight Drive + S-Curve + Turn *(software only)*
- **Gate:** none (reuses motors + IMU + encoders).
- **CubeMX:** none.
- **Code:** `pid_yaw` straight correction in `FSM_Straight_Update`;
  `scurve.{h,c}`; `FSM_Turn_Update` gyro-integrated pivot ([[ARCHITECTURE]]).
- **Verify:** 1 m straight lateral deviation; 90° turn angular error ([[REVIEW]]).

## Phase 6 — FSM + LCD + Manual + Emergency
- **Gate (USER):** I2C LCD on the software I2C bus; shock sensor PA6; buzzer PC0
  ([[USER]] §Phase 6).
- **CubeMX:** GPIO PC0 output; PA6 EXTI6 with pull-up and both-edge trigger;
  keep the generated EXTI IRQ path on LL.
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
- **Gate (USER):** 3 VL53L1X on the software I2C bus: SCL→PB6, SDA→PB7;
  XSHUT Front→PA1, Left→PA7, Right→PA8; bus pull-ups ([[USER]] §Phase 8).
- **CubeMX:** configure PA1/PA7/PA8 as initially-LOW outputs. Keep hardware I2C
  disabled and add `Drivers/VL53L1X/` ST API to the CMake sources.
- **Code:** `vl53l1x.{h,c}` — `TOF_Init_All` (XSHUT address sequence),
  `TOF_ReadDistance_mm`, `TOF_IsObstacle`; `FSM_Avoid_Update` (S-curve decel →
  TURN → STRAIGHT). ST API footprint is within the STM32F103RB memory budget.
- **Verify:** 3 sensors report on distinct addresses; Front/Left/Right distances;
  obstacle triggers AVOID chain.

## Phase 9 — Full Integration + Data Collection
- **Gate:** none.
- **Code:** glue only; tuning.
- **Verify:** run all engineering-goal measurements in [[REVIEW]] — 1 m deviation,
  90° turn (timer vs gyro), avoidance scenario, Kalman vs complementary drift.
