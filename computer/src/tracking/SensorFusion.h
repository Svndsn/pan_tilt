#pragma once
// Used for the gyro and accelerometer in PS4 controller

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
  void ReSetAbsoluteAngles();

private:
  float m_alpha;
  float m_beta;

  float m_accelBias;
  float m_gyroBias;

  Axis m_angle;
  Axis m_prevGyroMesure;
};
