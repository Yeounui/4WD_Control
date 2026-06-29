#include "fsm.h"

#include "main.h"
#include "motor.h"
#include "pid.h"
#include "scurve.h"

#include <math.h>

#define FSM_DRIVE_RPM 80.0f
#define FSM_TURN_RPM  60.0f
#define FSM_YAW_KP 0.0f             /* Tunable placeholder: yaw proportional gain. */
#define FSM_YAW_KI 0.0f             /* Tunable placeholder: yaw integral gain. */
#define FSM_YAW_KD 0.0f             /* Tunable placeholder: yaw derivative gain. */
#define FSM_TURN_TARGET_DEG 90.0f   /* Tunable placeholder: commanded turn angle. */
#define FSM_ACCEL_RATE_RPM_S 200.0f /* Tunable placeholder: straight ramp acceleration limit. */
#define FSM_JERK_RPM_S2 1000.0f     /* Tunable placeholder: straight ramp jerk limit. */

static volatile FSM_State current_state = FSM_STATE_IDLE;
static volatile uint8_t emergency_latched;
static volatile uint8_t fsm_initialized;
static FSM_Motion turn_motion = FSM_MOTION_LEFT;
static FSM_Motion manual_motion = FSM_MOTION_STOP;
static float speed_setpoint[MOTOR_COUNT];
static PID pid_yaw;
static SCurve straight_ramp;
static float target_yaw;
static uint8_t straight_armed;
static float turn_accum_deg;

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

static void FSM_Straight_Update(float yaw, float dt)
{
  float base;
  float corr;

  if (straight_armed == 0U)
  {
    target_yaw = yaw;
    PID_Reset(&pid_yaw);
    SCurve_Init(&straight_ramp, FSM_DRIVE_RPM, FSM_ACCEL_RATE_RPM_S, FSM_JERK_RPM_S2);
    straight_armed = 1U;
  }

  base = SCurve_Update(&straight_ramp, dt);
  corr = PID_Update(&pid_yaw, target_yaw, yaw, dt);
  FSM_SetSideTargets(base - corr, base + corr);
  /* Sign convention left=base-corr, right=base+corr is to be verified at tuning; harmless while Kp=0. */
}

static void FSM_Turn_Update(float omega_dps, float dt)
{
  FSM_ApplyMotion(turn_motion);
  turn_accum_deg += fabsf(omega_dps) * dt;
  if (turn_accum_deg >= FSM_TURN_TARGET_DEG)
  {
    FSM_StopTargets();
    Motor_StopAll();
    (void)FSM_SetState(FSM_STATE_STRAIGHT);
  }
}

void FSM_Init(void)
{
  fsm_initialized = 1U;
  turn_motion = FSM_MOTION_LEFT;
  manual_motion = FSM_MOTION_STOP;
  PID_Init(&pid_yaw, FSM_YAW_KP, FSM_YAW_KI, FSM_YAW_KD,
           -FSM_DRIVE_RPM, FSM_DRIVE_RPM);
  straight_armed = 0U;
  turn_accum_deg = 0.0f;
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

void FSM_Dispatch(float yaw, float omega_dps, float dt)
{
  if (emergency_latched != 0U)
  {
    current_state = FSM_STATE_EMERGENCY;
  }

  switch (current_state)
  {
    case FSM_STATE_STRAIGHT:
      FSM_Straight_Update(yaw, dt);
      break;

    case FSM_STATE_TURN:
      FSM_Turn_Update(omega_dps, dt);
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
  if (state == FSM_STATE_STRAIGHT)
  {
    straight_armed = 0U;
  }
  else if (state == FSM_STATE_TURN)
  {
    turn_accum_deg = 0.0f;
  }
  else if ((state == FSM_STATE_IDLE) || (state == FSM_STATE_EMERGENCY)
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
