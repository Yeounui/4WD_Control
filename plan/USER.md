# User Tasks & Local Constraints

Things only the user can do: physical wiring, motor orientation observation, and
USB/ST-Link attach when the host cannot see the board. CubeMX generation,
building, flashing, and serial capture are performed from this workspace when
the tools and ST-Link are available. **This file is the canonical pin map /
wiring source** — other documents link here instead of repeating the table.

## Canonical Pin Map (STM32F103RB)

| Peripheral | Pin | Mode | Module | Driver |
|---|---|---|---|---|
| H-bridge LF forward (shield 25) | PB3 / TIM2_CH2 | PWM AF push-pull | Left-front motor input 1 | HAL TIM |
| H-bridge LF reverse (shield 26) | PA10 / TIM1_CH3 | PWM AF push-pull | Left-front motor input 2 | HAL TIM |
| H-bridge RF forward (shield 17) | PB4 / TIM3_CH1 | PWM AF push-pull | Right-front motor input 1 | HAL TIM |
| H-bridge RF reverse (shield 16) | PB5 / TIM3_CH2 | PWM AF push-pull | Right-front motor input 2 | HAL TIM |
| H-bridge LR forward (shield 12) | PA9 / TIM1_CH2 | PWM AF push-pull | Left-rear motor input 1 | HAL TIM |
| H-bridge LR reverse (shield 27) | PB10 / TIM2_CH3 | PWM AF push-pull | Left-rear motor input 2 | HAL TIM |
| H-bridge RR forward (shield 21) | PB8 / TIM4_CH3 | PWM AF push-pull | Right-rear motor input 1 | HAL TIM |
| H-bridge RR reverse (shield 22) | PB9 / TIM4_CH4 | PWM AF push-pull | Right-rear motor input 2 | HAL TIM |
| Soft-I2C SCL | PB6 | GPIO open-drain (bit-bang) | MPU-6050 / VL53L1X ×3 / LCD shared bus | soft_i2c |
| Soft-I2C SDA | PB7 | GPIO open-drain (bit-bang) | MPU-6050 / VL53L1X ×3 / LCD shared bus | soft_i2c |
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
| GPIO | PC0 | Output | Buzzer | HAL |

> **Board identity (resolved 2026-06-24):** standard **Nucleo-F103RB** with on-board
> ST-Link/V2-1 (its VCP is wired to USART2 PA2/PA3 — shared with HC-06, use one at a
> time). On-board LD2 (PA5) and B1 (PC13) are unused by this design, so no pin clash.

> **Sensor bus = software I2C, not hardware I2C1 (resolved 2026-06-26):** enabling
> the hardware I2C1 peripheral pulls `I2C1_SMBA` onto PB5 and kills RF-reverse PWM
> ([[DECISIONS]] §D10). Every hardware I2C placement collides (I2C1 PB6/7→SMBA PB5;
> I2C1 remap PB8/9 = RR motor; I2C2 PB10/11 = LR-reverse). The MPU-6050 / LCD /
> VL53L1X bus is therefore a **bit-bang software I2C on PB6 (SCL) / PB7 (SDA)**
> (`soft_i2c`), needing external pull-ups on both lines. Wire the gyro SDA→PB7,
> SCL→PB6, AD0→GND (addr 0x68). Never re-enable hardware I2C1 on this stack.
> Hardware-verified: WHO_AM_I=0x68, MPU OK.

## Hardware Connection Gates

Before each coding phase, wire the listed parts and confirm power. Full phase
procedure is in [[PHASES]]; this section is the wiring checklist the user acts on.

- **Phase 0 — base board**: power the board, connect ST-Link (SWD: SWDIO, SWCLK,
  GND, optionally NRST + 3V3). No peripherals yet.
- **Phase 1 — 4× DC motor through H-bridge shield**: LF shield 25/26→PB3/PA10,
  RF 17/16→PB4/PB5, LR 12/27→PA9/PB10, RR 21/22→PB8/PB9. In each pair the first
  input is forward PWM and the second is reverse PWM; both compare values 0 is
  coast/stop. Use the shield's motor supply, tie shield GND to MCU GND, and do
  not power motors from the MCU rail.
- **Phase 2 — HC-06**: HC-06 RX←PA2 (TX), HC-06 TX→PA3 (RX), VCC (3.3–6 V per
  module), GND. Cross TX/RX. (Bench debug also uses PA2/PA3 before HC-06 attach.)
- **Phase 3 — MPU-6050**: planned I2C1 remap SCL→PB8, SDA→PB9, VCC 3.3 V,
  GND, AD0→GND (address 0x68). Do not wire until the PB8/PB9 RR motor conflict
  is resolved. Shares the I2C1 bus.
- **Phase 4 — 4× Hall sensor + magnets**: signal lines FL→PB0, FR→PB1, RL→PB2,
  RR→PA4. Mount one
  (or more) magnet(s) per wheel; record magnets-per-revolution for the encoder
  constant.
- **Phase 6 — I2C LCD + shock sensor + buzzer**: LCD (PCF8574 backpack) uses
  the resolved shared I2C1 bus; current planned remap is SCL→PB8, SDA→PB9 after
  the RR motor conflict is cleared. VCC 5 V, GND. Shock sensor signal→PA6.
  Buzzer→PC0, GND.
- **Phase 7 — tracking module**: analog out→PA0 (ADC1_CH0), VCC, GND.
- **Phase 8 — 3× VL53L1X**: all sensors use the resolved shared I2C1 bus;
  current planned remap is all SCL→PB8, all SDA→PB9 after the RR motor conflict
  is cleared. XSHUT Front→PA1, Left→PA7, Right→PA8. VCC, GND each.
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
ST-Link inside WSL2, attach the USB device to WSL first.

```
cmake --build build/Debug
cube programmer -c port=SWD -d build/Debug/4WD_Control.elf -v
stty -F /dev/ttyACM0 115200 raw -echo
```

## Empirically-determined values (filled in during bring-up)

- Kalman `R` — variance of 500 stationary gyro samples ([[PHASES]] §Phase 3).
- Kalman `Q`, PID gains (`pid_yaw`, `pid_speed`, `pid_line`) — tuned on hardware.
- Magnets-per-revolution for the Hall encoder constant ([[PHASES]] §Phase 4).
- VL53L1X obstacle thresholds (mm) per direction.
