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
#define FSM_LINE_KP 0.0f            /* Tunable placeholder: line proportional gain. */
#define FSM_LINE_KI 0.0f            /* Tunable placeholder: line integral gain. */
#define FSM_LINE_KD 0.0f            /* Tunable placeholder: line derivative gain. */
#define FSM_TURN_TARGET_DEG 90.0f   /* Tunable placeholder: commanded turn angle. */
#define FSM_ACCEL_RATE_RPM_S 200.0f /* Tunable placeholder: straight ramp acceleration limit. */
#define FSM_JERK_RPM_S2 1000.0f     /* Tunable placeholder: straight ramp jerk limit. */
#define FSM_AVOID_CLEAR_MS 300U

static volatile FSM_State current_state = FSM_STATE_IDLE;
static volatile uint8_t emergency_latched;
static volatile uint8_t fsm_initialized;
static volatile uint8_t obstacle_latched;
static FSM_Motion turn_motion = FSM_MOTION_LEFT;
static FSM_Motion manual_motion = FSM_MOTION_STOP;
typedef enum
{
  FSM_AVOID_DECEL = 0,
  FSM_AVOID_TURN,
  FSM_AVOID_CLEAR
} FSM_AvoidPhase;

static float speed_setpoint[MOTOR_COUNT];
static PID pid_yaw;
static PID pid_line;
static SCurve straight_ramp;
static SCurve avoid_ramp;
static float target_yaw;
static uint8_t straight_armed;
static uint8_t line_armed;
static uint8_t line_motion_enabled;
static FSM_AvoidPhase avoid_phase;
static float turn_accum_deg;
static uint32_t avoid_clear_start_ms;

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

static void FSM_LineTrace_Update(float line_error, uint8_t line_valid, float dt)
{
  float corr;

  if (line_valid == 0U)
  {
    line_armed = 0U;
    line_motion_enabled = 0U;
    PID_Reset(&pid_line);
    FSM_StopTargets();
    return;
  }

  if (line_armed == 0U)
  {
    PID_Reset(&pid_line);
    line_armed = 1U;
  }

  corr = PID_Update(&pid_line, 0.0f, line_error, dt);
  FSM_SetSideTargets(FSM_DRIVE_RPM - corr, FSM_DRIVE_RPM + corr);
  line_motion_enabled = 1U;
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

static void FSM_Avoid_Update(float omega_dps, float dt)
{
  switch (avoid_phase)
  {
    case FSM_AVOID_DECEL:
    {
      float base;

      base = SCurve_Update(&avoid_ramp, dt);
      FSM_SetSideTargets(base, base);
      if (avoid_ramp.phase == SCURVE_PHASE_DONE)
      {
        FSM_StopTargets();
        turn_motion = FSM_MOTION_LEFT;
        turn_accum_deg = 0.0f;
        avoid_phase = FSM_AVOID_TURN;
      }
      break;
    }

    case FSM_AVOID_TURN:
      FSM_ApplyMotion(turn_motion);
      turn_accum_deg += fabsf(omega_dps) * dt;
      if (turn_accum_deg >= FSM_TURN_TARGET_DEG)
      {
        FSM_StopTargets();
        avoid_clear_start_ms = HAL_GetTick();
        avoid_phase = FSM_AVOID_CLEAR;
      }
      break;

    case FSM_AVOID_CLEAR:
    default:
      FSM_SetSideTargets(FSM_DRIVE_RPM, FSM_DRIVE_RPM);
      if ((HAL_GetTick() - avoid_clear_start_ms) >= FSM_AVOID_CLEAR_MS)
      {
        obstacle_latched = 0U;
        (void)FSM_SetState(FSM_STATE_STRAIGHT);
      }
      break;
  }
}

void FSM_Init(void)
{
  fsm_initialized = 1U;
  turn_motion = FSM_MOTION_LEFT;
  manual_motion = FSM_MOTION_STOP;
  PID_Init(&pid_yaw, FSM_YAW_KP, FSM_YAW_KI, FSM_YAW_KD,
           -FSM_DRIVE_RPM, FSM_DRIVE_RPM);
  PID_Init(&pid_line, FSM_LINE_KP, FSM_LINE_KI, FSM_LINE_KD,
           -FSM_DRIVE_RPM, FSM_DRIVE_RPM);
  straight_armed = 0U;
  line_armed = 0U;
  line_motion_enabled = 0U;
  obstacle_latched = 0U;
  avoid_phase = FSM_AVOID_DECEL;
  avoid_clear_start_ms = 0U;
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

void FSM_Dispatch(float yaw, float omega_dps, float dt, float line_error, uint8_t line_valid)
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

    case FSM_STATE_LINE_TRACE:
      FSM_LineTrace_Update(line_error, line_valid, dt);
      break;

    case FSM_STATE_MANUAL:
      FSM_ApplyMotion(manual_motion);
      break;

    case FSM_STATE_AVOID:
      FSM_Avoid_Update(omega_dps, dt);
      break;

    case FSM_STATE_EMERGENCY:
    case FSM_STATE_IDLE:
    default:
      FSM_StopTargets();
      Motor_StopAll();
      break;
  }
}

void FSM_SetObstacle(uint8_t obstacle)
{
  if ((obstacle == 0U) || (emergency_latched != 0U))
  {
    return;
  }
  if (obstacle_latched != 0U)
  {
    return;
  }

  obstacle_latched = 1U;
  if ((current_state == FSM_STATE_STRAIGHT) || (current_state == FSM_STATE_LINE_TRACE))
  {
    (void)FSM_SetState(FSM_STATE_AVOID);
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
    line_armed = 0U;
    line_motion_enabled = 0U;
  }
  else if (state == FSM_STATE_TURN)
  {
    turn_accum_deg = 0.0f;
    line_armed = 0U;
    line_motion_enabled = 0U;
  }
  else if (state == FSM_STATE_LINE_TRACE)
  {
    line_armed = 0U;
    line_motion_enabled = 0U;
    PID_Reset(&pid_line);
  }
  else if ((state == FSM_STATE_IDLE) || (state == FSM_STATE_EMERGENCY)
      || (state == FSM_STATE_AVOID))
  {
    FSM_StopTargets();
    Motor_StopAll();
    if (state == FSM_STATE_AVOID)
    {
      avoid_ramp.current = FSM_DRIVE_RPM;
      avoid_ramp.accel = 0.0f;
      avoid_ramp.target = 0.0f;
      avoid_ramp.accel_max = FSM_ACCEL_RATE_RPM_S;
      avoid_ramp.jerk = FSM_JERK_RPM_S2;
      avoid_ramp.phase = SCURVE_PHASE_DECEL;
      avoid_phase = FSM_AVOID_DECEL;
      turn_accum_deg = 0.0f;
    }
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

  emergency_latched = 0U;
  current_state = FSM_STATE_IDLE;
  manual_motion = FSM_MOTION_STOP;
  reset_ok = 1U;
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
    case FSM_STATE_AVOID:
      return 1U;

    case FSM_STATE_LINE_TRACE:
      return line_motion_enabled;

    case FSM_STATE_MANUAL:
      return (manual_motion != FSM_MOTION_STOP) ? 1U : 0U;

    case FSM_STATE_IDLE:
    case FSM_STATE_EMERGENCY:
    default:
      return 0U;
  }
}

uint8_t FSM_IsEmergency(void)
{
  return emergency_latched;
}
