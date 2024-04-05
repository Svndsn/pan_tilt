#pragma once

typedef struct {
  float roll;
  float pitch;
  float yaw;
} Axis;

class SensorFusion {
public:
  SensorFusion(float alpha, float gyroBias = 0.f, float accelBias = 0.f);

  void getAngles(Axis &accelerometer, Axis &gyroscope, float Ts,
                 Axis *angleOutputs);
  void ResetAngles();

private:
  float m_alpha;
  float m_beta;

  float m_accelBias;
  float m_gyroBias;

  float m_pitch;
  float m_roll;
  float m_yaw;
};
