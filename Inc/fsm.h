#ifndef FSM_H
#define FSM_H

#include "motor.h"

#include <stdint.h>

typedef enum
{
  FSM_STATE_IDLE = 0,
  FSM_STATE_STRAIGHT,
  FSM_STATE_LINE_TRACE,
  FSM_STATE_TURN,
  FSM_STATE_AVOID,
  FSM_STATE_MANUAL,
  FSM_STATE_EMERGENCY,
  FSM_STATE_COUNT
} FSM_State;

typedef enum
{
  FSM_MOTION_STOP = 0,
  FSM_MOTION_FORWARD,
  FSM_MOTION_LEFT,
  FSM_MOTION_RIGHT
} FSM_Motion;

void FSM_Init(void);
void FSM_Dispatch(float yaw, float omega_dps, float dt);
uint8_t FSM_SetState(FSM_State state);
void FSM_SetDirection(FSM_Motion motion);
uint8_t FSM_ResetEmergency(void);
void FSM_RequestEmergencyFromISR(void);
FSM_State FSM_GetState(void);
const char *FSM_GetStateName(void);
float FSM_GetSpeedSetpoint(MotorId wheel);
uint8_t FSM_IsMotionEnabled(void);
uint8_t FSM_IsEmergency(void);

#endif /* FSM_H */
