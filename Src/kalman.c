#include "kalman.h"

static float angle;
static float bias;
static float covariance[2][2];
static float q_angle;
static float q_bias;
static float r_measure;

void Kalman_Init(void)
{
  angle = 0.0f;
  bias = 0.0f;
  covariance[0][0] = 0.0f;
  covariance[0][1] = 0.0f;
  covariance[1][0] = 0.0f;
  covariance[1][1] = 0.0f;
  q_angle = 0.001f;
  q_bias = 0.003f;
  r_measure = 0.03f;
}

float Kalman_Update(float accel_angle, float omega_dps, float dt)
{
  float rate;
  float innovation_covariance;
  float gain_0;
  float gain_1;
  float innovation;
  float covariance_00;
  float covariance_01;

  rate = omega_dps - bias;
  angle += dt * rate;

  covariance[0][0] += dt * ((dt * covariance[1][1]) - covariance[0][1]
                            - covariance[1][0] + q_angle);
  covariance[0][1] -= dt * covariance[1][1];
  covariance[1][0] -= dt * covariance[1][1];
  covariance[1][1] += q_bias * dt;

  innovation_covariance = covariance[0][0] + r_measure;
  gain_0 = covariance[0][0] / innovation_covariance;
  gain_1 = covariance[1][0] / innovation_covariance;
  innovation = accel_angle - angle;

  angle += gain_0 * innovation;
  bias += gain_1 * innovation;

  covariance_00 = covariance[0][0];
  covariance_01 = covariance[0][1];

  covariance[0][0] -= gain_0 * covariance_00;
  covariance[0][1] -= gain_0 * covariance_01;
  covariance[1][0] -= gain_1 * covariance_00;
  covariance[1][1] -= gain_1 * covariance_01;

  return angle;
}

void Kalman_SetR(float r)
{
  r_measure = r;
}

void Kalman_SetBias(float new_bias)
{
  bias = new_bias;
}

float Kalman_GetR(void)
{
  return r_measure;
}

float Kalman_GetBias(void)
{
  return bias;
}

float Kalman_GetAngle(void)
{
  return angle;
}
