#pragma once
#include "serialib.h"
#include <cstdint>
#include <queue>
#include <string>

enum class Angle { Tilt, Pan };
enum class RxState { Type, Length, Data, Checksum };
enum class CommandType {
  SetpointPanAbsolute,
  SetpointPanRelative,
  AnglePan,
  SetpointTiltAbsolute,
  SetpointTiltRelative,
  AngleTilt,
  DebugString,
  VoltagePan,
  VoltageTilt
};

class AngleHandler {
public:
  // Constructor
  AngleHandler();

  // Destructor
  ~AngleHandler();

  // Method to receive data from the UART
  void ReceiveUART();

  // Method to get the tilt angles
  std::pair<float, float> GetAngles() const;

  // Method to get the setpoint angles
  std::pair<float, float> GetSetpoints() const;

  // Method to get the voltage
  std::pair<float, float> GetVoltage() const;

  // Method to set the pan angle
  void SetAbsoluteAngles(float absPanAngle, float absTiltAngle);

  // Method to set relative angles
  void SetRelativeAngles(float relPanAngle, float relTiltAngle);

private:
  // Method to find the serial device
  std::string FindSerialDevice() const;

  void SendSetpointUART(Angle type, float setpoint, bool relative) const;

  void CheckSum(uint8_t *data, size_t length, uint8_t &checksum) const;

  mutable serialib m_serialConnection;
  std::queue<uint8_t> m_RxDataQueue;
  RxState m_RxState;

  // The setpoint angles
  float m_panSetpointAngle, m_tiltSetpointAngle;
  // The current system angles
  float m_panAngle, m_tiltAngle;
  // The current system voltage
  float m_panVoltage, m_tiltVoltage;
};
