#include <assert.h>
#include <stdio.h>
#include <stdint.h>

#include "encoder.h"

static void pump_pulses(MotorId wheel, unsigned int count)
{
  unsigned int pulse;

  for (pulse = 0U; pulse < count; pulse++) {
    Encoder_OnPulse(wheel);
  }
}

static void test_init_zeros_all_speeds(void)
{
  MotorId wheel;

  Encoder_Init();

  for (wheel = MOTOR_LF; wheel < MOTOR_COUNT; wheel++) {
    assert(Encoder_GetSpeed(wheel) == 0.0f);
  }
}

static void test_one_revolution_in_one_tick(void)
{
  Encoder_Init();
  pump_pulses(MOTOR_LF, (uint32_t)ENCODER_COUNTS_PER_REV);

  Encoder_Sample(0.01f);

  assert(Encoder_GetSpeed(MOTOR_LF) == 6000.0f);
}

static void test_wheels_are_independent(void)
{
  Encoder_Init();
  pump_pulses(MOTOR_LF, (uint32_t)ENCODER_COUNTS_PER_REV);
  pump_pulses(MOTOR_RR, (uint32_t)ENCODER_COUNTS_PER_REV / 2U);

  Encoder_Sample(0.01f);

  assert(Encoder_GetSpeed(MOTOR_LF) == 6000.0f);
  assert(Encoder_GetSpeed(MOTOR_RR) == 3000.0f);
  assert(Encoder_GetSpeed(MOTOR_RF) == 0.0f);
  assert(Encoder_GetSpeed(MOTOR_LR) == 0.0f);
}

static void test_nonpositive_dt_preserves_pending_delta(void)
{
  Encoder_Init();
  pump_pulses(MOTOR_RF, (uint32_t)ENCODER_COUNTS_PER_REV);
  Encoder_Sample(0.01f);
  assert(Encoder_GetSpeed(MOTOR_RF) == 6000.0f);

  pump_pulses(MOTOR_RF, (uint32_t)ENCODER_COUNTS_PER_REV / 2U);
  Encoder_Sample(0.0f);
  assert(Encoder_GetSpeed(MOTOR_RF) == 6000.0f);

  Encoder_Sample(0.01f);
  assert(Encoder_GetSpeed(MOTOR_RF) == 3000.0f);
}

static void test_invalid_wheel_is_safe(void)
{
  MotorId invalid = (MotorId)MOTOR_COUNT;

  Encoder_Init();
  Encoder_OnPulse(invalid);
  Encoder_Sample(0.01f);

  assert(Encoder_GetSpeed(invalid) == 0.0f);
  assert(Encoder_GetSpeed(MOTOR_LF) == 0.0f);
  assert(Encoder_GetSpeed(MOTOR_RF) == 0.0f);
  assert(Encoder_GetSpeed(MOTOR_LR) == 0.0f);
  assert(Encoder_GetSpeed(MOTOR_RR) == 0.0f);
}

int main(void)
{
  test_init_zeros_all_speeds();
  test_one_revolution_in_one_tick();
  test_wheels_are_independent();
  test_nonpositive_dt_preserves_pending_delta();
  test_invalid_wheel_is_safe();

  puts("PASS");
  return 0;
}
