#pragma once
#include "serialib.h"
#include <cstdint>
#include <mutex>
#include <string>

class UARTAngleHandler {
public:
  // Constructor
  UARTAngleHandler();

  // Destructor
  ~UARTAngleHandler();

  // Method to receive the angle from the UART
  void ReceiveAngles();

  // Method to send the angle to the UART
  void SendAngles() const;

  // Method to get the tilt angle
  std::pair<int16_t, int16_t> GetAngles() const;

  // Method to set the pan angle
  void SetAngles(int16_t targetPanAngle, int16_t targetTiltAngle);

private:
  std::string FindSerialDevice();

  serialib m_serialConnection;
  int16_t m_panAngle, m_tiltAngle;
  mutable std::mutex m_angleMutex;
};
