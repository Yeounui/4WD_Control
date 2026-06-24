#include <stdint.h>

#include "encoder.h"

static volatile uint32_t pulse_count[MOTOR_COUNT];
static uint32_t last_count[MOTOR_COUNT];
static float speed_rpm[MOTOR_COUNT];

void Encoder_Init(void)
{
  MotorId wheel;

  for (wheel = MOTOR_LF; wheel < MOTOR_COUNT; wheel++) {
    pulse_count[wheel] = 0U;
    last_count[wheel] = 0U;
    speed_rpm[wheel] = 0.0f;
  }
}

void Encoder_OnPulse(MotorId wheel)
{
  if (wheel >= MOTOR_COUNT) {
    return;
  }

  pulse_count[wheel]++;
}

void Encoder_Sample(float dt)
{
  MotorId wheel;

  if (dt <= 0.0f) {
    return;
  }

  for (wheel = MOTOR_LF; wheel < MOTOR_COUNT; wheel++) {
    uint32_t snapshot = pulse_count[wheel];
    uint32_t delta = snapshot - last_count[wheel];
    float revolutions;

    last_count[wheel] = snapshot;
    revolutions = (float)delta / ENCODER_COUNTS_PER_REV;
    speed_rpm[wheel] = (revolutions / dt) * 60.0f;
  }
}

float Encoder_GetSpeed(MotorId wheel)
{
  if (wheel >= MOTOR_COUNT) {
    return 0.0f;
  }

  return speed_rpm[wheel];
}
