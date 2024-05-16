#include "SensorFusion.h"
#include <assert.h>
#include <fmt/core.h>
#include <math.h>

// 180 / pi
#define RAD_TO_DEG 57.2957795

SensorFusion::SensorFusion(float alpha, float gyroBias, float accelBias) {

  assert(alpha >= 0 && alpha <= 1);
  m_alpha = alpha;
  m_beta = 1 - alpha;

  m_gyroBias = gyroBias;
  m_accelBias = accelBias;

  m_angle.pitch = 0;
  m_angle.roll = 0;
  m_angle.yaw = 0;

  m_prevGyroMesure.pitch = 0;
  m_prevGyroMesure.roll = 0;
  m_prevGyroMesure.yaw = 0;
};

void SensorFusion::ResetAbsoluteAngles() {
  m_angle.pitch = 0;
  m_angle.roll = 0;
  m_angle.yaw = 0;
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
  // Yaw can't be calculated with only accel data
  float pitchAccel =
      atan2(-accel.roll, sqrt(pow(accel.yaw, 2) + pow(accel.pitch, 2)));
  float rollAccel =
      atan2(accel.pitch, sqrt(pow(accel.yaw, 2) + pow(accel.roll, 2)));

  // Trapezoidal integration for gyro data
  float pitchGyro = Ts * (gyro.pitch + m_prevGyroMesure.pitch) / 2;
  float rollGyro = Ts * (gyro.roll + m_prevGyroMesure.roll) / 2;
  float yawGyro = Ts * (gyro.yaw + m_prevGyroMesure.yaw) / 2;

  // Complimentary Filter for pitch and roll (gyro and accel data)
  m_angle.pitch = m_alpha * (m_angle.pitch + pitchGyro) + m_beta * pitchAccel;
  m_angle.roll = m_alpha * (m_angle.roll + rollGyro) + m_beta * rollAccel;
  // Normal integration for yaw (no magnetometer data available)
  m_angle.yaw = m_angle.yaw + yawGyro;

  m_prevGyroMesure = gyro;

  // Convert to degrees
  angleOutputs->pitch = m_angle.pitch * RAD_TO_DEG;
  angleOutputs->roll = m_angle.roll * RAD_TO_DEG;
  angleOutputs->yaw = m_angle.yaw * RAD_TO_DEG;
};
