#pragma once
#include "serialib.h"
#include <cstdint>
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

  // Method to get the tilt angles
  std::pair<int16_t, int16_t> GetAngles() const;

  // Method to get the setpoint angles
  std::pair<int16_t, int16_t> GetSetpoints() const;

  // Method to set the pan angle
  void SetAbsoluteAngles(int16_t absPanAngle, int16_t absTiltAngle);

  // Method to set relative angles
  void SetRelativeAngles(int16_t relPanAngle, int16_t relTiltAngle);

private:
  // Method to find the serial device
  std::string FindSerialDevice() const;

  void SendSetpointUART(Angle type, int16_t setpoint, bool relative) const;
  void ReceiveAnglesUART();
  void DataToAngle(uint8_t data);

  mutable serialib m_serialConnection;
  int16_t m_panSetpointAngle, m_tiltSetpointAngle;
  int16_t m_panAngle, m_tiltAngle;
};
