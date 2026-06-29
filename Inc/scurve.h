#ifndef SCURVE_H
#define SCURVE_H

typedef enum
{
  SCURVE_PHASE_ACCEL = 0,
  SCURVE_PHASE_CRUISE,
  SCURVE_PHASE_DECEL,
  SCURVE_PHASE_DONE
} SCurvePhase;

typedef struct
{
  float current;
  float accel;
  float target;
  float accel_max;
  float jerk;
  SCurvePhase phase;
} SCurve;

void SCurve_Init(SCurve *sc, float target, float accel_max, float jerk);
float SCurve_Update(SCurve *sc, float dt);

#endif /* SCURVE_H */
