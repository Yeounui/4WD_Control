# User Tasks & Local Constraints

Things only the user can do: physical wiring, CubeMX generation, flashing, and
ST-Link/USB attach. **This file is the canonical pin map / wiring source** —
other documents link here instead of repeating the table.

## Canonical Pin Map (STM32F103RE)

| Peripheral | Pin | Mode | Module | Driver |
|---|---|---|---|---|
| TIM1 CH1 | PA8 | PWM Output | FL Servo | HAL |
| TIM1 CH2 | PA9 | PWM Output | FR Servo | HAL |
| TIM1 CH3 | PA10 | PWM Output | RL Servo | HAL |
| TIM1 CH4 | PA11 | PWM Output | RR Servo | HAL |
| I2C1 SCL | PB6 | I2C | MPU-6050 / VL53L1X ×3 / LCD | HAL |
| I2C1 SDA | PB7 | I2C | MPU-6050 / VL53L1X ×3 / LCD | HAL |
| USART2 TX | PA2 | UART | HC-06 (and bench debug, see [[DECISIONS]] §D3) | HAL (DMA) |
| USART2 RX | PA3 | UART | HC-06 | HAL (DMA) |
| ADC1 CH0 | PA0 | ADC + DMA | Tracking module | HAL |
| EXTI0 | PB0 | EXTI | Hall sensor FL | **LL** |
| EXTI1 | PB1 | EXTI | Hall sensor FR | **LL** |
| EXTI2 | PB2 | EXTI | Hall sensor RL | **LL** |
| EXTI3 | PB3 | EXTI | Hall sensor RR | **LL** |
| EXTI4 | PB4 | EXTI | Shock sensor | **LL** |
| GPIO | PB8 | Output | XSHUT VL53L1X Front | HAL |
| GPIO | PB9 | Output | XSHUT VL53L1X Left | HAL |
| GPIO | PB10 | Output | XSHUT VL53L1X Right | HAL |
| GPIO | PC0 | Output | Buzzer | HAL |

> **Board identity (open question):** there is no standard NUCLEO-F103RE. Confirm
> the physical board before Phase 0 — it determines the ST-Link connection (on-board
> vs. external) and whether any of the pins above clash with on-board peripherals.

## Hardware Connection Gates

Before each coding phase, wire the listed parts and confirm power. Full phase
procedure is in [[PHASES]]; this section is the wiring checklist the user acts on.

- **Phase 0 — base board**: power the board, connect ST-Link (SWD: SWDIO, SWCLK,
  GND, optionally NRST + 3V3). No peripherals yet.
- **Phase 1 — 4× servo**: FL→PA8, FR→PA9, RL→PA10, RR→PA11 signal lines.
  **Separate servo power supply** (servos draw far more than the MCU rail can
  give); tie servo-supply GND to MCU GND (common ground). Do not back-feed servo
  power into the board.
- **Phase 2 — HC-06**: HC-06 RX←PA2 (TX), HC-06 TX→PA3 (RX), VCC (3.3–6 V per
  module), GND. Cross TX/RX. (Bench debug also uses PA2/PA3 before HC-06 attach.)
- **Phase 3 — MPU-6050**: SCL→PB6, SDA→PB7, VCC 3.3 V, GND, AD0→GND (address
  0x68). Shares I2C1 bus.
- **Phase 4 — 4× Hall sensor + magnets**: signal lines FL→PB0, FR→PB1, RL→PB2,
  RR→PB3; VCC, GND. Mount one (or more) magnet(s) per wheel; record magnets-per-
  revolution for the encoder constant.
- **Phase 6 — I2C LCD + shock sensor + buzzer**: LCD (PCF8574 backpack) SCL→PB6,
  SDA→PB7, VCC 5 V, GND (shares I2C1). Shock sensor signal→PB4, VCC, GND. Buzzer
  →PC0, GND.
- **Phase 7 — tracking module**: analog out→PA0 (ADC1_CH0), VCC, GND.
- **Phase 8 — 3× VL53L1X**: all SCL→PB6, all SDA→PB7 (shared bus); XSHUT lines
  Front→PB8, Left→PB9, Right→PB10; VCC, GND each. Pull-ups on SDA/SCL (one set for
  the bus). Address separation is done in firmware via XSHUT — see [[ARCHITECTURE]]
  §vl53l1x.

## CubeMX Generation (per phase) — done by Claude, not the user

CubeMX generation is **not** a user task. Claude authors the `.ioc` and runs
STM32CubeMX **6.17.0** headless (verified working — see [[DECISIONS]] §D5):

```
/opt/st/stm32cubemx_6.17.0/STM32CubeMX -q <script.txt>
# script: load STM32F103RETx; project name <n>; project toolchain CMake; project path <dir>; project generate; exit
```

Per phase, the `.ioc` targets device **STM32F103RETx**, enables the peripherals
listed in [[PHASES]], sets the **LL** driver for Hall/shock EXTI ([[DECISIONS]]
§D4), and toolchain = CMake. Driver logic is then hand-written only inside
`/* USER CODE BEGIN/END */` blocks. The user's only CubeMX-step involvement is the
physical hardware gate above.

## Build & Flash

Toolchain and commands follow the workspace `stm32-development-workflow` skill
(CMake + Ninja + arm-none-eabi-gcc, flash via STM32CubeProgrammer). To expose the
ST-Link inside WSL2, use the `usbipd-wsl` skill to attach the USB device.

> Concrete build/flash commands depend on the generated project name and are
> filled in once Phase 0 produces the CubeMX project.

## Empirically-determined values (filled in during bring-up)

- Kalman `R` — variance of 500 stationary gyro samples ([[PHASES]] §Phase 3).
- Kalman `Q`, PID gains (`pid_yaw`, `pid_speed`, `pid_line`) — tuned on hardware.
- Magnets-per-revolution for the Hall encoder constant ([[PHASES]] §Phase 4).
- VL53L1X obstacle thresholds (mm) per direction.
