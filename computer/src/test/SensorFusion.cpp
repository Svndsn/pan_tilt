#include "SensorFusion.h"
#include "fmt/core.h"
#include <assert.h>
#include <math.h>

#define PI 3.14159265359
#define RAD_TO_DEG 180 / PI

SensorFusion::SensorFusion(float alpha, float pitchAccelOffset,
                           float rollAccelOffset, float yawAccelOffset) {

  assert(alpha >= 0 && alpha <= 1);
  m_alpha = alpha;
  m_beta = 1 - alpha;

  m_pitchAccelOffset = pitchAccelOffset;
  m_rollAccelOffset = rollAccelOffset;
  m_yawAccelOffset = yawAccelOffset;

  m_pitch = 0;
  m_roll = 0;
  m_yaw = 0;
};

void SensorFusion::ResetAngles() {
  m_pitch = 0;
  m_roll = 0;
  m_yaw = 0;
}

void SensorFusion::getAngles(ThreeAxis &accelerometer, ThreeAxis &gyroscope,
                             float Ts, ThreeAxis *angleOutputs) {
  // Subtract offsets bias
  // accelerometer.pitch -= m_pitchAccelOffset;
  // accelerometer.roll -= m_rollAccelOffset;
  // accelerometer.yaw -= m_yawAccelOffset;

  // Calculate pitch and roll angles from accelerometer data
  float rollAccel =
      atan2(accelerometer.pitch,
            sqrt(pow(accelerometer.yaw, 2) + pow(accelerometer.roll, 2)));
  float pitchAccel = atan2(accelerometer.yaw, accelerometer.roll);

  // Complimentary Filter for gyro and accelerometer data
  m_pitch = m_alpha * (m_pitch + gyroscope.pitch * Ts) + m_beta * pitchAccel;
  m_roll = m_alpha * (m_roll + gyroscope.roll * Ts) + m_beta * rollAccel;

  // Normal integration for yaw (no magnetometer data available)
  m_yaw = m_yaw + (gyroscope.yaw * Ts);

  // Convert to degrees
  angleOutputs->pitch = m_pitch * RAD_TO_DEG;
  angleOutputs->roll = m_roll * RAD_TO_DEG;
  angleOutputs->yaw = m_yaw * RAD_TO_DEG;
};
