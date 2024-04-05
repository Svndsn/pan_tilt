#include "SensorFusion.h"
#include <assert.h>
#include <math.h>

#define PI 3.14159265359
#define RAD_TO_DEG 180 / PI

SensorFusion::SensorFusion(float alpha, float gyroBias, float accelBias) {

  assert(alpha >= 0 && alpha <= 1);
  m_alpha = alpha;
  m_beta = 1 - alpha;

  m_gyroBias = gyroBias;
  m_accelBias = accelBias;

  m_pitch = 0;
  m_roll = 0;
  m_yaw = 0;
};

void SensorFusion::ResetAngles() {
  m_pitch = 0;
  m_roll = 0;
  m_yaw = 0;
}

void SensorFusion::getAngles(Axis &accel, Axis &gyro, float Ts,
                             Axis *angleOutputs) {
  // Subtract offsets bias
  gyro.pitch = abs(gyro.pitch) < m_gyroBias ? 0 : gyro.pitch;
  gyro.roll = abs(gyro.roll) < m_gyroBias ? 0 : gyro.roll;
  gyro.yaw = abs(gyro.yaw) < m_gyroBias ? 0 : gyro.yaw;
  accel.pitch = abs(accel.pitch) < m_accelBias ? 0 : accel.pitch;
  accel.roll = abs(accel.roll) < m_accelBias ? 0 : accel.roll;
  accel.yaw = abs(accel.yaw) < m_accelBias ? 0 : accel.yaw;

  // Calculate pitch and roll angles from accel data
  float pitchAccel =
      atan2(-accel.roll, sqrt(pow(accel.yaw, 2) + pow(accel.pitch, 2)));
  float rollAccel =
      atan2(accel.pitch, sqrt(pow(accel.yaw, 2) + pow(accel.roll, 2)));

  // Complimentary Filter for gyro and accel data
  m_pitch = m_alpha * (m_pitch + gyro.pitch * Ts) + m_beta * pitchAccel;
  m_roll = m_alpha * (m_roll + gyro.roll * Ts) + m_beta * rollAccel;

  // Normal integration for yaw (no magnetometer data available)
  m_yaw = m_yaw + (gyro.yaw * Ts);

  // Convert to degrees
  angleOutputs->pitch = m_pitch * RAD_TO_DEG;
  angleOutputs->roll = m_roll * RAD_TO_DEG;
  angleOutputs->yaw = m_yaw * RAD_TO_DEG;
};
