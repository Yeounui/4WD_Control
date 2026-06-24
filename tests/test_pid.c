#include <assert.h>
#include <stdio.h>

#include "pid.h"

static const float dt = 0.01f;

static float absf_local(float value)
{
  if (value < 0.0f)
  {
    return -value;
  }

  return value;
}

static void test_zero_error_after_init(void)
{
  PID pid;
  float output;

  PID_Init(&pid, 1.5f, 0.5f, 0.1f, -100.0f, 100.0f);
  output = PID_Update(&pid, 5.0f, 5.0f, dt);

  assert(output == 0.0f);
  assert(pid.integral == 0.0f);
  assert(pid.prev_error == 0.0f);
}

static void test_error_sign_symmetry(void)
{
  PID pid_positive;
  PID pid_negative;
  float positive_output;
  float negative_output;

  PID_Init(&pid_positive, 2.0f, 0.0f, 0.0f, -100.0f, 100.0f);
  PID_Init(&pid_negative, 2.0f, 0.0f, 0.0f, -100.0f, 100.0f);

  positive_output = PID_Update(&pid_positive, 4.0f, 1.0f, dt);
  negative_output = PID_Update(&pid_negative, 1.0f, 4.0f, dt);

  assert(positive_output > 0.0f);
  assert(negative_output < 0.0f);
  assert(positive_output == -negative_output);
}

static void test_output_clamping(void)
{
  PID pid;
  float positive_output;
  float negative_output;

  PID_Init(&pid, 10.0f, 0.0f, 0.0f, -25.0f, 25.0f);
  positive_output = PID_Update(&pid, 100.0f, 0.0f, dt);
  negative_output = PID_Update(&pid, -100.0f, 0.0f, dt);

  assert(positive_output == 25.0f);
  assert(negative_output == -25.0f);
}

static void test_anti_windup_and_recovery(void)
{
  PID pid;
  float output;
  int step;
  int recovered;

  PID_Init(&pid, 0.2f, 5.0f, 0.0f, -10.0f, 10.0f);

  for (step = 0; step < 600; ++step)
  {
    output = PID_Update(&pid, 100.0f, 0.0f, dt);
    assert(pid.integral <= pid.i_max);
    assert(pid.integral >= pid.i_min);
    assert(output == 10.0f);
  }

  recovered = 0;
  for (step = 0; step < 400; ++step)
  {
    output = PID_Update(&pid, -5.0f, 0.0f, dt);
    assert(pid.integral <= pid.i_max);
    assert(pid.integral >= pid.i_min);
    assert(output >= pid.out_min);
    assert(output <= pid.out_max);

    if (output < 10.0f)
    {
      recovered = 1;
      break;
    }
  }

  assert(recovered != 0);
  assert(step > 0);
}

static void test_zero_dt_keeps_state(void)
{
  PID pid;
  float output;
  float integral_before;
  float prev_error_before;

  PID_Init(&pid, 1.0f, 2.0f, 3.0f, -100.0f, 100.0f);
  PID_Update(&pid, 2.0f, 1.0f, dt);
  integral_before = pid.integral;
  prev_error_before = pid.prev_error;

  output = PID_Update(&pid, 5.0f, 1.0f, 0.0f);

  assert(pid.integral == integral_before);
  assert(pid.prev_error == prev_error_before);
  assert(output == (pid.kp * 4.0f) + (pid.ki * integral_before));
}

static void test_convergence(void)
{
  PID pid;
  float measured;
  float setpoint;
  float output;
  float plant_gain;
  int step;

  PID_Init(&pid, 2.5f, 1.2f, 0.05f, -100.0f, 100.0f);
  measured = 0.0f;
  setpoint = 10.0f;
  plant_gain = 4.0f;

  for (step = 0; step < 1500; ++step)
  {
    output = PID_Update(&pid, setpoint, measured, dt);
    measured += plant_gain * (output - measured) * dt;
  }

  assert(absf_local(setpoint - measured) < 0.5f);
}

int main(void)
{
  test_zero_error_after_init();
  test_error_sign_symmetry();
  test_output_clamping();
  test_anti_windup_and_recovery();
  test_zero_dt_keeps_state();
  test_convergence();

  printf("PASS\n");
  return 0;
}
