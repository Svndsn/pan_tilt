#pragma once
#include "serialib.h"
#include <cstdint>
#include <mutex>
#include <string>

enum class Angle { Tilt, Pan };

class AngleHandler {
public:
  // Constructor
  AngleHandler();

  // Destructor
  ~AngleHandler();

  // Method to receive the angle from the UART
  void ReceiveAngles();

  // Method to send the angle to the UART
  void SendAngles() const;

  // Method to get the tilt angle
  std::pair<int16_t, int16_t> GetAngles() const;

  // Method to set the pan angle
  void SetAngles(int16_t targetPanAngle, int16_t targetTiltAngle);

  // Method to set relative angles
  void SetRelativeAngles(int16_t panAngle, int16_t tiltAngle);

private:
  std::string FindSerialDevice() const;

  void SendAngleUART(Angle type) const;
  void ReceiveAnglesUART();

  mutable serialib m_serialConnection;
  int16_t m_axisRightX, m_tiltAngle;
  mutable std::mutex m_angleMutex;
};
