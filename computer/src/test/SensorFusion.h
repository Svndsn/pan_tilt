#pragma once

// The leniency of shouldUpdateData(), in microseconds
#define DATA_UPDATE_POLL_TOLERANCE 5

typedef struct {
  float roll;
  float pitch;
  float yaw;
} ThreeAxis;

typedef enum { UNIT_DEGREES, UNIT_RADIANS } AngleUnit;

class SensorFusion {

public:
  SensorFusion(float alpha, float pitchAccelOffset, float rollAccelOffset,
               float yawAccelOffset);

  void getAngles(ThreeAxis &accelerometer, ThreeAxis &gyroscope, float Ts,
                 ThreeAxis *angleOutputs);
  void ResetAngles();

private:
  float m_alpha;
  float m_beta;

  float m_pitchAccelOffset;
  float m_rollAccelOffset;
  float m_yawAccelOffset;

  float m_pitch;
  float m_roll;
  float m_yaw;
};
