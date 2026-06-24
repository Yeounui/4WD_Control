#ifndef MOTOR_H
#define MOTOR_H

typedef enum
{
  MOTOR_LF = 0,
  MOTOR_RF,
  MOTOR_LR,
  MOTOR_RR,
  MOTOR_COUNT
} MotorId;

typedef enum
{
  MOTOR_STOP = 0,
  MOTOR_FORWARD,
  MOTOR_REVERSE
} MotorDirection;

void Motor_Init(void);
void Motor_SetDirection(MotorId id, MotorDirection direction);
void Motor_SetAll(MotorDirection direction);
void Motor_StopAll(void);

#endif /* MOTOR_H */
