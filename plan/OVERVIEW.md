# Overview

## Goal

Firmware for a 4-wheel-drive (4WD) precision robot car on STM32F103RE. Each wheel
is driven by an independent continuous-rotation servo. The car drives straight
with closed-loop yaw correction, makes accurate in-place turns, follows a line,
avoids obstacles with time-of-flight ranging, and accepts manual control over
Bluetooth — all coordinated by a finite-state machine running on a fixed control
tick.

## Platform

| Item | Detail |
|---|---|
| MCU | STM32F103RE (Cortex-M3, 72 MHz, **64 KB SRAM / 512 KB Flash**) |
| Drive | KR120 continuous-rotation servo × 4 (independent 4WD) |
| Wireless | HC-06 Bluetooth — USART2 AT-command manual control |
| Libraries | STM32 HAL (base) · LL (latency-critical EXTI) · VL53L1X ST API |

> The original `plan.md` listed "SRAM 20 KB"; this was a typo for the F103RE.
> See [[DECISIONS]] §D2. The static-allocation discipline below is retained
> regardless of the larger SRAM.

## In Scope

- 4-channel servo PWM drive (TIM1)
- Bluetooth manual control + AT-command parser (HC-06 / USART2)
- IMU yaw estimation with a 1-D Kalman filter (MPU-6050 / I2C1)
- DIY Hall-sensor wheel encoders + per-wheel speed PID (EXTI, LL)
- Straight-line yaw PID, S-curve speed profiling, gyro-integrated turns
- 7-state FSM coordinating all behaviors
- I2C LCD status display
- Line tracing via ADC-DMA tracking module
- Obstacle avoidance via 3× VL53L1X ToF (I2C multi-drop with XSHUT)
- Shock-sensor emergency stop (EXTI, LL)

## Constraints

- **No dynamic allocation** — `malloc`/`new` prohibited; all buffers statically
  declared. (Retained as discipline even though SRAM is 64 KB.)
- **HAL + LL mixed** per ST AN5044: HAL for I2C / ADC-DMA / TIM-PWM / USART-DMA /
  SysTick; LL for latency-critical EXTI (Hall + shock). See [[DECISIONS]] §D4.
- **CubeMX-first**: every phase generates peripheral boilerplate in STM32CubeMX,
  then driver logic is hand-written inside `USER CODE` sections. See
  [[DECISIONS]] §D5.
- Verify the VL53L1X ST API static RAM footprint before integration ([[PHASES]]
  §Phase 8).

## Where details live

- Pin map / wiring → [[USER]]
- Module function contracts, control-loop timing, FSM → [[ARCHITECTURE]]
- Acceptance metrics (deviation, turn error, etc.) → [[REVIEW]]
- Phase procedure with hardware gates → [[PHASES]]
