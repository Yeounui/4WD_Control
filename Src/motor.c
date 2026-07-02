#include "motor.h"

#include "main.h"

#define MOTOR_MAX_DUTY 3199

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

typedef struct {
  TIM_HandleTypeDef *fwd_htim; uint32_t fwd_ch;
  TIM_HandleTypeDef *rev_htim; uint32_t rev_ch;
} MotorPwm;

static const MotorPwm motor_pwm[MOTOR_COUNT] = {
  {&htim2, TIM_CHANNEL_2, &htim1, TIM_CHANNEL_3},
  {&htim3, TIM_CHANNEL_1, &htim3, TIM_CHANNEL_2},
  {&htim1, TIM_CHANNEL_2, &htim2, TIM_CHANNEL_3},
  {&htim4, TIM_CHANNEL_3, &htim4, TIM_CHANNEL_4}
};

void Motor_Init(void) {
  MotorId id;

  for (id = MOTOR_LF; id < MOTOR_COUNT; id++) {
    HAL_TIM_PWM_Start(motor_pwm[id].fwd_htim, motor_pwm[id].fwd_ch);
    HAL_TIM_PWM_Start(motor_pwm[id].rev_htim, motor_pwm[id].rev_ch);
  }

  for (id = MOTOR_LF; id < MOTOR_COUNT; id++) {
    __HAL_TIM_SET_COMPARE(motor_pwm[id].fwd_htim, motor_pwm[id].fwd_ch, 0);
    __HAL_TIM_SET_COMPARE(motor_pwm[id].rev_htim, motor_pwm[id].rev_ch, 0);
  }
}

void Motor_SetDuty(MotorId id, int16_t duty) {
  uint32_t fwd_value = 0;
  uint32_t rev_value = 0;

  if (id >= MOTOR_COUNT) {
    return;
  }

  if (duty > MOTOR_MAX_DUTY) {
    duty = MOTOR_MAX_DUTY;
  } else if (duty < -MOTOR_MAX_DUTY) {
    duty = -MOTOR_MAX_DUTY;
  }

  if (duty > 0) {
    fwd_value = (uint32_t)duty;
  } else if (duty < 0) {
    rev_value = (uint32_t)(-duty);
  }

  __HAL_TIM_SET_COMPARE(motor_pwm[id].fwd_htim, motor_pwm[id].fwd_ch, fwd_value);
  __HAL_TIM_SET_COMPARE(motor_pwm[id].rev_htim, motor_pwm[id].rev_ch, rev_value);
}

void Motor_SetDirection(MotorId id, MotorDirection direction) {
  switch (direction) {
    case MOTOR_FORWARD:
      Motor_SetDuty(id, MOTOR_MAX_DUTY);
      break;

    case MOTOR_REVERSE:
      Motor_SetDuty(id, -MOTOR_MAX_DUTY);
      break;

    case MOTOR_STOP:
    default:
      Motor_SetDuty(id, 0);
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
    Motor_SetDuty(id, 0);
  }
}
