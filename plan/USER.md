# User Tasks & Local Constraints

Things only the user can do: physical wiring, CubeMX generation, flashing, and
ST-Link/USB attach. **This file is the canonical pin map / wiring source** —
other documents link here instead of repeating the table.

## Canonical Pin Map (STM32F103RB)

| Peripheral | Pin | Mode | Module | Driver |
|---|---|---|---|---|
| H-bridge LF forward (shield 25) | PB3 | GPIO Output | Left-front motor input 1 | HAL |
| H-bridge LF reverse (shield 26) | PA10 | GPIO Output | Left-front motor input 2 | HAL |
| H-bridge RF forward (shield 17) | PB4 | GPIO Output | Right-front motor input 1 | HAL |
| H-bridge RF reverse (shield 16) | PB5 | GPIO Output | Right-front motor input 2 | HAL |
| H-bridge LR forward (shield 12) | PA9 | GPIO Output | Left-rear motor input 1 | HAL |
| H-bridge LR reverse (shield 27) | PB10 | GPIO Output | Left-rear motor input 2 | HAL |
| H-bridge RR forward (shield 21) | PB8 | GPIO Output | Right-rear motor input 1 | HAL |
| H-bridge RR reverse (shield 22) | PB9 | GPIO Output | Right-rear motor input 2 | HAL |
| I2C1 SCL | PB6 | I2C | MPU-6050 / VL53L1X ×3 / LCD | HAL |
| I2C1 SDA | PB7 | I2C | MPU-6050 / VL53L1X ×3 / LCD | HAL |
| USART2 TX | PA2 | UART | HC-06 (and bench debug, see [[DECISIONS]] §D3) | HAL (DMA) |
| USART2 RX | PA3 | UART | HC-06 | HAL (DMA) |
| ADC1 CH0 | PA0 | ADC + DMA | Tracking module | HAL |
| EXTI0 | PB0 | EXTI | Hall sensor FL | LL |
| EXTI1 | PB1 | EXTI | Hall sensor FR | LL |
| EXTI2 | PB2 | EXTI | Hall sensor RL | LL |
| EXTI4 | PA4 (Arduino A2) | EXTI | Hall sensor RR | LL |
| EXTI6 | PA6 (Arduino D12) | EXTI | Shock sensor | LL |
| GPIO | PA1 (Arduino A1) | Output | XSHUT VL53L1X Front | HAL |
| GPIO | PA7 (Arduino D11) | Output | XSHUT VL53L1X Left | HAL |
| GPIO | PA8 (Arduino D7) | Output | XSHUT VL53L1X Right | HAL |

> **Board identity (resolved 2026-06-24):** standard **Nucleo-F103RB** with on-board
> ST-Link/V2-1 (its VCP is wired to USART2 PA2/PA3 — shared with HC-06, use one at a
> time). On-board LD2 (PA5) and B1 (PC13) are unused by this design, so no pin clash.

## Hardware Connection Gates

Before each coding phase, wire the listed parts and confirm power. Full phase
procedure is in [[PHASES]]; this section is the wiring checklist the user acts on.

- **Phase 0 — base board**: power the board, connect ST-Link (SWD: SWDIO, SWCLK,
  GND, optionally NRST + 3V3). No peripherals yet.
- **Phase 1 — 4× DC motor through H-bridge shield**: LF shield 25/26→PB3/PA10,
  RF 17/16→PB4/PB5, LR 12/27→PA9/PB10, RR 21/22→PB8/PB9. In each pair the first
  input is forward and the second is reverse; both LOW is stop. Use the shield's
  motor supply, tie shield GND to MCU GND, and do not power motors from the MCU
  rail.
- **Phase 2 — HC-06**: HC-06 RX←PA2 (TX), HC-06 TX→PA3 (RX), VCC (3.3–6 V per
  module), GND. Cross TX/RX. (Bench debug also uses PA2/PA3 before HC-06 attach.)
- **Phase 3 — MPU-6050**: SCL→PB6, SDA→PB7, VCC 3.3 V, GND, AD0→GND (address
  0x68). Shares I2C1 bus.
- **Phase 4 — 4× Hall sensor + magnets**: signal lines FL→PB0, FR→PB1, RL→PB2,
  RR→PA4. Mount one
  (or more) magnet(s) per wheel; record magnets-per-revolution for the encoder
  constant.
- **Phase 6 — I2C LCD + shock sensor**: LCD (PCF8574 backpack) SCL→PB6,
  SDA→PB7, VCC 5 V, GND (shares I2C1). Shock sensor signal→PA6.
- **Phase 7 — tracking module**: analog out→PA0 (ADC1_CH0), VCC, GND.
- **Phase 8 — 3× VL53L1X**: all SCL→PB6, all SDA→PB7 (shared bus); XSHUT
  Front→PA1, Left→PA7, Right→PA8. VCC, GND each.
  Pull-ups on SDA/SCL (one set for the bus). Address separation is done in
  firmware via XSHUT — see [[ARCHITECTURE]] §vl53l1x.

## CubeMX Generation (per phase) — done by Claude, not the user

CubeMX generation is **not** a user task. Claude authors the `.ioc` and runs
STM32CubeMX **6.17.0** headless (verified working — see [[DECISIONS]] §D5):

```
/opt/st/stm32cubemx_6.17.0/STM32CubeMX -q <script.txt>
# script: load STM32F103RBTx; project name <n>; project toolchain CMake; project path <dir>; project generate; exit
```

Per phase, the `.ioc` targets device **STM32F103RBTx**, enables the peripherals
listed in [[PHASES]], sets the **LL** driver for Hall/shock EXTI ([[DECISIONS]]
§D4), and toolchain = CMake. Driver logic is then hand-written only inside
`/* USER CODE BEGIN/END */` blocks. The user's only CubeMX-step involvement is the
physical hardware gate above.

## Build & Flash

Toolchain and commands follow the workspace `stm32-firmware-workflow` skill
(CMake + Ninja + arm-none-eabi-gcc, flash via STM32CubeProgrammer). To expose the
ST-Link inside WSL2, use the `usbipd-wsl` skill to attach the USB device.

> Concrete build/flash commands depend on the generated project name and are
> filled in once Phase 0 produces the CubeMX project.

## Empirically-determined values (filled in during bring-up)

- Kalman `R` — variance of 500 stationary gyro samples ([[PHASES]] §Phase 3).
- Kalman `Q`, PID gains (`pid_yaw`, `pid_speed`, `pid_line`) — tuned on hardware.
- Magnets-per-revolution for the Hall encoder constant ([[PHASES]] §Phase 4).
- VL53L1X obstacle thresholds (mm) per direction.
