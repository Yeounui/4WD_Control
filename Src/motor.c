#include "motor.h"

#include "main.h"

typedef struct {
  GPIO_TypeDef *forward_port; uint16_t forward_pin;
  GPIO_TypeDef *reverse_port; uint16_t reverse_pin;
} MotorPins;

static const MotorPins motor_pins[MOTOR_COUNT] = {
  {MOTOR_LF_FWD_GPIO_Port, GPIO_PIN_3, MOTOR_LF_REV_GPIO_Port, GPIO_PIN_10},
  {MOTOR_RF_FWD_GPIO_Port, GPIO_PIN_4, MOTOR_RF_REV_GPIO_Port, GPIO_PIN_5},
  {MOTOR_LR_FWD_GPIO_Port, GPIO_PIN_9, MOTOR_LR_REV_GPIO_Port, GPIO_PIN_10},
  {MOTOR_RR_FWD_GPIO_Port, GPIO_PIN_8, MOTOR_RR_REV_GPIO_Port, GPIO_PIN_9}
};

void Motor_Init(void) {
  Motor_StopAll();
}

void Motor_SetDirection(MotorId id, MotorDirection direction) {
  const MotorPins *pins;
  if (id >= MOTOR_COUNT) { return; }

  pins = &motor_pins[id];

  HAL_GPIO_WritePin(pins->forward_port, pins->forward_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(pins->reverse_port, pins->reverse_pin, GPIO_PIN_RESET);

  switch (direction) {
    case MOTOR_FORWARD:
      HAL_GPIO_WritePin(pins->forward_port, pins->forward_pin, GPIO_PIN_SET);
      break;

    case MOTOR_REVERSE:
      HAL_GPIO_WritePin(pins->reverse_port, pins->reverse_pin, GPIO_PIN_SET);
      break;

    case MOTOR_STOP:
    default:
      break;
  }
}

void Motor_SetAll(MotorDirection direction) {
  MotorId id;

  for (id = MOTOR_LF; id < MOTOR_COUNT; id++) {
    Motor_SetDirection(id, direction);
  }
}

void Motor_StopAll(void) {
  MotorId id;

  for (id = MOTOR_LF; id < MOTOR_COUNT; id++) {
    HAL_GPIO_WritePin(motor_pins[id].forward_port, motor_pins[id].forward_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_pins[id].reverse_port, motor_pins[id].reverse_pin, GPIO_PIN_RESET);
  }
}
