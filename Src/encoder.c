#include <stdint.h>

#include "encoder.h"

static volatile uint32_t pulse_count[MOTOR_COUNT];
static volatile uint32_t last_pulse_tick[MOTOR_COUNT];
static volatile uint32_t period_ms[MOTOR_COUNT];
static volatile uint8_t has_pulse[MOTOR_COUNT];
static float speed_rpm[MOTOR_COUNT];

extern uint32_t HAL_GetTick(void);

void Encoder_Init(void)
{
  MotorId wheel;

  for (wheel = MOTOR_LF; wheel < MOTOR_COUNT; wheel++) {
    pulse_count[wheel] = 0U;
    last_pulse_tick[wheel] = 0U;
    period_ms[wheel] = 0U;
    has_pulse[wheel] = 0U;
    speed_rpm[wheel] = 0.0f;
  }
}

void Encoder_OnPulse(MotorId wheel)
{
  uint32_t now;

  if (wheel >= MOTOR_COUNT) {
    return;
  }

  now = HAL_GetTick();
  pulse_count[wheel]++;
  if (has_pulse[wheel] != 0U) {
    uint32_t elapsed = now - last_pulse_tick[wheel];

    if (elapsed >= ENCODER_DEBOUNCE_FLOOR_MS) {
      period_ms[wheel] = elapsed;
    }
  } else {
    has_pulse[wheel] = 1U;
  }
  last_pulse_tick[wheel] = now;
}

void Encoder_Sample(float dt)
{
  MotorId wheel;
  uint32_t now;

  (void)dt;
  now = HAL_GetTick();

  for (wheel = MOTOR_LF; wheel < MOTOR_COUNT; wheel++) {
    uint32_t last_tick = last_pulse_tick[wheel];
    uint32_t period = period_ms[wheel];

    if ((has_pulse[wheel] == 0U) || (period == 0U)
        || ((now - last_tick) > ENCODER_STALL_TIMEOUT_MS)) {
      speed_rpm[wheel] = 0.0f;
    } else {
      speed_rpm[wheel] = 60000.0f / (float)period;
    }
  }
}

float Encoder_GetSpeed(MotorId wheel)
{
  if (wheel >= MOTOR_COUNT) {
    return 0.0f;
  }

  return speed_rpm[wheel];
}

uint32_t Encoder_GetCount(MotorId wheel)
{
  if (wheel >= MOTOR_COUNT) {
    return 0U;
  }

  return pulse_count[wheel];
}
