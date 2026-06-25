#include "fsm.h"

#include "main.h"
#include "motor.h"

#define FSM_DRIVE_RPM 80.0f
#define FSM_TURN_RPM  60.0f

static volatile FSM_State current_state = FSM_STATE_IDLE;
static volatile uint8_t emergency_latched;
static volatile uint8_t fsm_initialized;
static FSM_Motion turn_motion = FSM_MOTION_LEFT;
static FSM_Motion manual_motion = FSM_MOTION_STOP;
static float speed_setpoint[MOTOR_COUNT];

static void FSM_StopTargets(void)
{
  MotorId wheel;

  for (wheel = MOTOR_LF; wheel < MOTOR_COUNT; wheel++)
  {
    speed_setpoint[wheel] = 0.0f;
  }
}

static void FSM_SetSideTargets(float left_rpm, float right_rpm)
{
  speed_setpoint[MOTOR_LF] = left_rpm;
  speed_setpoint[MOTOR_LR] = left_rpm;
  speed_setpoint[MOTOR_RF] = right_rpm;
  speed_setpoint[MOTOR_RR] = right_rpm;
}

static void FSM_ApplyMotion(FSM_Motion motion)
{
  switch (motion)
  {
    case FSM_MOTION_FORWARD:
      FSM_SetSideTargets(FSM_DRIVE_RPM, FSM_DRIVE_RPM);
      break;

    case FSM_MOTION_LEFT:
      FSM_SetSideTargets(-FSM_TURN_RPM, FSM_TURN_RPM);
      break;

    case FSM_MOTION_RIGHT:
      FSM_SetSideTargets(FSM_TURN_RPM, -FSM_TURN_RPM);
      break;

    case FSM_MOTION_STOP:
    default:
      FSM_StopTargets();
      break;
  }
}

void FSM_Init(void)
{
  fsm_initialized = 1U;
  turn_motion = FSM_MOTION_LEFT;
  manual_motion = FSM_MOTION_STOP;
  FSM_StopTargets();
  if (emergency_latched != 0U)
  {
    current_state = FSM_STATE_EMERGENCY;
  }
  else
  {
    current_state = FSM_STATE_IDLE;
  }
  Motor_StopAll();
}

void FSM_Dispatch(void)
{
  if (emergency_latched != 0U)
  {
    current_state = FSM_STATE_EMERGENCY;
  }

  switch (current_state)
  {
    case FSM_STATE_STRAIGHT:
      FSM_ApplyMotion(FSM_MOTION_FORWARD);
      break;

    case FSM_STATE_TURN:
      FSM_ApplyMotion(turn_motion);
      break;

    case FSM_STATE_MANUAL:
      FSM_ApplyMotion(manual_motion);
      break;

    case FSM_STATE_LINE_TRACE:
    case FSM_STATE_AVOID:
      /* Future sensor-dependent handlers fail safe until their drivers exist. */
      FSM_StopTargets();
      break;

    case FSM_STATE_EMERGENCY:
    case FSM_STATE_IDLE:
    default:
      FSM_StopTargets();
      Motor_StopAll();
      break;
  }
}

uint8_t FSM_SetState(FSM_State state)
{
  if (state >= FSM_STATE_COUNT)
  {
    return 0U;
  }

  if ((emergency_latched != 0U) && (state != FSM_STATE_EMERGENCY))
  {
    return 0U;
  }

  if (state == FSM_STATE_EMERGENCY)
  {
    emergency_latched = 1U;
  }

  current_state = state;
  if ((state == FSM_STATE_IDLE) || (state == FSM_STATE_EMERGENCY)
      || (state == FSM_STATE_LINE_TRACE) || (state == FSM_STATE_AVOID))
  {
    FSM_StopTargets();
    Motor_StopAll();
  }
  else if (state == FSM_STATE_MANUAL)
  {
    manual_motion = FSM_MOTION_STOP;
    FSM_StopTargets();
    Motor_StopAll();
  }

  return 1U;
}

void FSM_SetDirection(FSM_Motion motion)
{
  if ((motion > FSM_MOTION_RIGHT) || (emergency_latched != 0U))
  {
    return;
  }

  if (current_state == FSM_STATE_MANUAL)
  {
    manual_motion = motion;
  }
  else if (motion == FSM_MOTION_FORWARD)
  {
    (void)FSM_SetState(FSM_STATE_STRAIGHT);
  }
  else if ((motion == FSM_MOTION_LEFT) || (motion == FSM_MOTION_RIGHT))
  {
    turn_motion = motion;
    (void)FSM_SetState(FSM_STATE_TURN);
  }
  else
  {
    (void)FSM_SetState(FSM_STATE_IDLE);
  }
}

uint8_t FSM_ResetEmergency(void)
{
  uint32_t primask;
  uint8_t reset_ok;

  primask = __get_PRIMASK();
  __disable_irq();

  if ((LL_GPIO_IsInputPinSet(SHOCK_GPIO_Port, SHOCK_Pin) == 0U)
      || (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_6) != RESET))
  {
    emergency_latched = 1U;
    current_state = FSM_STATE_EMERGENCY;
    reset_ok = 0U;
  }
  else
  {
    emergency_latched = 0U;
    current_state = FSM_STATE_IDLE;
    manual_motion = FSM_MOTION_STOP;
    reset_ok = 1U;
  }
  FSM_StopTargets();
  Motor_StopAll();

  if (primask == 0U)
  {
    __enable_irq();
  }

  if (emergency_latched != 0U)
  {
    reset_ok = 0U;
  }

  return reset_ok;
}

void FSM_RequestEmergencyFromISR(void)
{
  emergency_latched = 1U;
  current_state = FSM_STATE_EMERGENCY;
  if (fsm_initialized != 0U)
  {
    Motor_StopAll();
  }
}

FSM_State FSM_GetState(void)
{
  return current_state;
}

const char *FSM_GetStateName(void)
{
  static const char *const names[FSM_STATE_COUNT] = {
    "IDLE",
    "STRAIGHT",
    "LINE TRACE",
    "TURN",
    "AVOID",
    "MANUAL",
    "EMERGENCY"
  };
  FSM_State state;

  state = current_state;
  if (state >= FSM_STATE_COUNT)
  {
    return "INVALID";
  }

  return names[state];
}

float FSM_GetSpeedSetpoint(MotorId wheel)
{
  if (wheel >= MOTOR_COUNT)
  {
    return 0.0f;
  }

  return speed_setpoint[wheel];
}

uint8_t FSM_IsMotionEnabled(void)
{
  if (emergency_latched != 0U)
  {
    return 0U;
  }

  switch (current_state)
  {
    case FSM_STATE_STRAIGHT:
    case FSM_STATE_TURN:
      return 1U;

    case FSM_STATE_MANUAL:
      return (manual_motion != FSM_MOTION_STOP) ? 1U : 0U;

    case FSM_STATE_IDLE:
    case FSM_STATE_LINE_TRACE:
    case FSM_STATE_AVOID:
    case FSM_STATE_EMERGENCY:
    default:
      return 0U;
  }
}

uint8_t FSM_IsEmergency(void)
{
  return emergency_latched;
}
