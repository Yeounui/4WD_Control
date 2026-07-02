# Overview

## Goal

Firmware for a 4-wheel-drive (4WD) precision robot car on STM32F103RB. Each wheel
is driven by an independent DC motor through a two-input H-bridge shield. The car drives straight
with closed-loop yaw correction, makes accurate in-place turns, follows a line,
avoids obstacles with time-of-flight ranging, and accepts manual control over
Bluetooth — all coordinated by a finite-state machine running on a fixed control
tick.

## Platform

| Item | Detail |
|---|---|
| MCU | STM32F103RB (Cortex-M3, HSI/2 × PLL16 = **64 MHz**, **20 KB SRAM / 128 KB Flash**) |
| Drive | DC motor × 4 through H-bridge shield (two PWM direction inputs per motor) |
| Wireless | HC-06 Bluetooth — USART2 AT-command manual control |
| Libraries | STM32 HAL (base) · LL (latency-critical EXTI) · VL53L1X ST API |

> The original `plan.md` "SRAM 20 KB" was correct — the chip is the F103RB
> (reversed 2026-06-24, see [[DECISIONS]] §D2). The static-allocation discipline
> below matters all the more given only 20 KB SRAM.

## In Scope

- 8-channel TIM PWM H-bridge direction drive (signed duty per motor;
  forward/reverse/coast-at-zero)
- Bluetooth manual control + AT-command parser (HC-06 / USART2)
- IMU yaw estimation with a 1-D Kalman filter (MPU-6050 / software I2C on PB6/PB7)
- DIY Hall-sensor wheel encoders + per-wheel speed PID (EXTI)
- Straight-line yaw PID, S-curve speed profiling, gyro-integrated turns
- 7-state FSM coordinating all behaviors
- I2C LCD status display
- Line tracing via ADC-DMA tracking module
- Obstacle avoidance via 3× VL53L1X ToF (I2C multi-drop with XSHUT)
- Shock-sensor emergency stop (EXTI)

## Constraints

- **No dynamic allocation** — `malloc`/`new` prohibited; all buffers statically
  declared. (All the more important given only 20 KB SRAM.)
- **HAL + LL mixed** per ST AN5044: HAL for TIM PWM / I2C / ADC-DMA /
  USART-DMA / SysTick; LL for latency-critical EXTI (Hall + shock). See
  [[DECISIONS]] §D4.
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
