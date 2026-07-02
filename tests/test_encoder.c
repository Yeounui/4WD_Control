#include <assert.h>
#include <stdio.h>
#include <stdint.h>

#include "encoder.h"

static uint32_t fake_tick_ms;

uint32_t HAL_GetTick(void)
{
  return fake_tick_ms;
}

static void set_tick(uint32_t tick_ms)
{
  fake_tick_ms = tick_ms;
}

static void advance_tick(uint32_t delta_ms)
{
  fake_tick_ms += delta_ms;
}

static void assert_close(float actual, float expected)
{
  assert(actual > (expected - 0.001f));
  assert(actual < (expected + 0.001f));
}

static void test_init_zeros_all_speeds(void)
{
  MotorId wheel;

  set_tick(0U);
  Encoder_Init();

  for (wheel = MOTOR_LF; wheel < MOTOR_COUNT; wheel++) {
    assert(Encoder_GetSpeed(wheel) == 0.0f);
    assert(Encoder_GetCount(wheel) == 0U);
  }
}

static void test_one_revolution_period_reports_rpm(void)
{
  set_tick(100U);
  Encoder_Init();

  Encoder_OnPulse(MOTOR_LF);
  advance_tick(750U);
  Encoder_OnPulse(MOTOR_LF);

  Encoder_Sample(0.01f);

  assert_close(Encoder_GetSpeed(MOTOR_LF), 80.0f);
  assert(Encoder_GetCount(MOTOR_LF) == 2U);
}

static void test_wheels_are_independent(void)
{
  set_tick(100U);
  Encoder_Init();

  Encoder_OnPulse(MOTOR_LF);
  Encoder_OnPulse(MOTOR_RR);
  advance_tick(500U);
  Encoder_OnPulse(MOTOR_RR);
  advance_tick(250U);
  Encoder_OnPulse(MOTOR_LF);

  Encoder_Sample(0.01f);

  assert_close(Encoder_GetSpeed(MOTOR_LF), 80.0f);
  assert_close(Encoder_GetSpeed(MOTOR_RR), 120.0f);
  assert(Encoder_GetSpeed(MOTOR_RF) == 0.0f);
  assert(Encoder_GetSpeed(MOTOR_LR) == 0.0f);
}

static void test_debounce_rejects_short_period(void)
{
  set_tick(100U);
  Encoder_Init();

  Encoder_OnPulse(MOTOR_RF);
  advance_tick(ENCODER_DEBOUNCE_FLOOR_MS - 1U);
  Encoder_OnPulse(MOTOR_RF);
  Encoder_Sample(0.0f);
  assert(Encoder_GetSpeed(MOTOR_RF) == 0.0f);
  assert(Encoder_GetCount(MOTOR_RF) == 2U);

  advance_tick(750U);
  Encoder_OnPulse(MOTOR_RF);
  Encoder_Sample(0.0f);
  assert_close(Encoder_GetSpeed(MOTOR_RF), 80.0f);
  assert(Encoder_GetCount(MOTOR_RF) == 3U);
}

static void test_stall_timeout_reports_zero(void)
{
  set_tick(100U);
  Encoder_Init();

  Encoder_OnPulse(MOTOR_LR);
  advance_tick(750U);
  Encoder_OnPulse(MOTOR_LR);
  Encoder_Sample(0.01f);
  assert_close(Encoder_GetSpeed(MOTOR_LR), 80.0f);

  advance_tick(ENCODER_STALL_TIMEOUT_MS + 1U);
  Encoder_Sample(0.01f);
  assert(Encoder_GetSpeed(MOTOR_LR) == 0.0f);
}

static void test_invalid_wheel_is_safe(void)
{
  MotorId invalid = (MotorId)MOTOR_COUNT;

  set_tick(100U);
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
  test_one_revolution_period_reports_rpm();
  test_wheels_are_independent();
  test_debounce_rejects_short_period();
  test_stall_timeout_reports_zero();
  test_invalid_wheel_is_safe();

  puts("PASS");
  return 0;
}
