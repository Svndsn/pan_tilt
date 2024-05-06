#include "AngleHandler.h"
#include "serialib.h"
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fmt/core.h>
#include <string>
#include <vector>

#define PAN_ANGLE_MIN -80
#define PAN_ANGLE_MAX 80
#define TILT_ANGLE_MIN -90
#define TILT_ANGLE_MAX 90
#define MAX_DATA_LENGTH 16

AngleHandler::AngleHandler() {
  // Connection to serial port
  int err = m_serialConnection.openDevice(FindSerialDevice().c_str(), 
                                          115200,
                                          SERIAL_DATABITS_8, 
                                          SERIAL_PARITY_NONE,
                                          SERIAL_STOPBITS_1);

  if (err < 1) fmt::print("Serial error: Error code: {}\n", err);

  m_panAngle = 0;
  m_tiltAngle = 0;
  m_panSetpointAngle = 0;
  m_tiltSetpointAngle = 0;
}

AngleHandler::~AngleHandler() {
  if (m_serialConnection.isDeviceOpen()) {
    m_serialConnection.closeDevice();
  }
}

std::pair<int16_t, int16_t> AngleHandler::GetAngles() const {
  int16_t panAngle = m_panAngle;
  int16_t tiltAngle = m_tiltAngle;
  return std::make_pair(panAngle, tiltAngle);
}

std::pair<int16_t, int16_t> AngleHandler::GetSetpoints() const {
  return std::make_pair(m_panSetpointAngle, m_tiltSetpointAngle);
}

void AngleHandler::ReceiveAngles() {
  ReceiveAnglesUART();
}

void AngleHandler::SetAbsoluteAngles(int16_t absPanAngle, int16_t absTiltAngle) {
  // Set the setpoint angles
  m_panSetpointAngle = absPanAngle;
  m_tiltSetpointAngle = absTiltAngle;

  // Bound the angles to the limits
  if (m_panSetpointAngle < PAN_ANGLE_MIN) m_panSetpointAngle = PAN_ANGLE_MIN;
  else if (m_panSetpointAngle > PAN_ANGLE_MAX) m_panSetpointAngle = PAN_ANGLE_MAX;
  
  if (m_tiltSetpointAngle < TILT_ANGLE_MIN) m_tiltSetpointAngle = TILT_ANGLE_MIN;
  else if (m_tiltSetpointAngle > TILT_ANGLE_MAX) m_tiltSetpointAngle = TILT_ANGLE_MAX;

  // Send the absolute setpoint angles to the microcontroller
  SendSetpointUART(Angle::Pan, absPanAngle, false);
  SendSetpointUART(Angle::Tilt, absTiltAngle, false);
}

void AngleHandler::SetRelativeAngles(int16_t relPanAngle, int16_t relTiltAngle) {
  // Set the setpoint angles
  m_panSetpointAngle = m_panAngle + relPanAngle;
  m_tiltSetpointAngle = m_tiltAngle + relTiltAngle;

  // Bound the angles to the limits
  if (m_panSetpointAngle < PAN_ANGLE_MIN) m_panSetpointAngle = PAN_ANGLE_MIN;
  else if (m_panSetpointAngle > PAN_ANGLE_MAX) m_panSetpointAngle = PAN_ANGLE_MAX;
  
  if (m_tiltSetpointAngle < TILT_ANGLE_MIN) m_tiltSetpointAngle = TILT_ANGLE_MIN;
  else if (m_tiltSetpointAngle > TILT_ANGLE_MAX) m_tiltSetpointAngle = TILT_ANGLE_MAX;

  // Send the relative setpoint angles to the microcontroller
  SendSetpointUART(Angle::Pan, relPanAngle, true);
  SendSetpointUART(Angle::Tilt, relTiltAngle, true);
}

std::string AngleHandler::FindSerialDevice() const {
  std::string device{};

  #if defined(__APPLE__)
    std::vector<std::string> devices;
    std::string path = "/dev/";
    // Find all serial devices
    for (const auto &entry : std::filesystem::directory_iterator(path))
      if (entry.path().string().find("tty.") != std::string::npos)
        devices.push_back(entry.path().string());

    // Select the serial device
    for (size_t i = 0; i < devices.size(); i++) {
      fmt::print("Device {}: {}\n", i, devices[i]);
    }
    size_t deviceIndex;
    std::cin >> deviceIndex;
    assert(!std::cin.fail() && deviceIndex < devices.size() &&
          "Invalid device index");
    device = devices[deviceIndex];
    fmt::print("Selected device: {}\n", devices[deviceIndex]);
  #endif

  return device;
}

void AngleHandler::SendSetpointUART(Angle axis, int16_t setpoint, bool relative) const {
  // Format:
  // ABRSDDDD
  // A: Angle (1 = Pan, 0 = Tilt)
  // B: Bits  (1 = High bits, 0 = Low bits)
  // R: Relative (Relative: R=1, Absolute: R=0)
  // S: Sign  (Data: B=0, Sign: B=1)
  // D: Data  (Always)
  uint8_t lowByte  = 0;
  uint8_t highByte = (1 << 6);

  // Set type bit (Pan = 1, Tilt = 0)
  if (axis == Angle::Pan) {
    lowByte  |= (1 << 7);
    highByte |= (1 << 7);
  }

  // Set relative bits
  if (relative) {
    lowByte  |= (1 << 5);
    highByte |= (1 << 5);
  }

  // Set sign bit
  if (setpoint < 0) {
    setpoint = -setpoint;
    // Set sign bit
    highByte |= (1 << 4);
  }

  // Set data bits
  lowByte  |= setpoint & 0b00011111;
  highByte |= (setpoint >> 5) & 0b00001111;

  // Send data (Order is not important)
  m_serialConnection.writeChar(lowByte);
  m_serialConnection.writeChar(highByte);
}

void AngleHandler::ReceiveAnglesUART() {
  // Read the lastest data from the serial port
  uint8_t receivedBytes[MAX_DATA_LENGTH];
  int nBytes =
      m_serialConnection.readBytes(receivedBytes, MAX_DATA_LENGTH, 100);
  if (nBytes < 0) {
    fmt::print("Error while reading bytes. Error code: {}\n", nBytes);
    return;
  } else if (nBytes == 0) {
    // No data received
    return;
  }
  for (int i = 0; i < nBytes; i++) {
    uint8_t data = receivedBytes[i];
    int16_t *angle;
    if (data & (1 << 7)) {
      angle = &m_panAngle;
    } else {
      angle = &m_tiltAngle;
    }

    bool isNegative = *angle < 0;
    if (isNegative) {
      *angle = -(*angle);
    }

    if (data & (1 << 6)) {   // High bits
      // Remove old high bits data
      *angle &= 0x1F;
      // Set new high bits data
      *angle |= (data & 0x0F) << 5;
      // Set the sign bit
      isNegative = data & (1 << 4);
    } else { // Low bits
      // Remove old low bits data
      *angle &= 0xFFE0;
      // Set new low bits data
      *angle |= data & 0x1F;
    }

    // Restore the sign
    if (isNegative) {
      *angle = -(*angle);
    }

    // Only absolute angles are received
    if (data & (1 << 5)) {
      fmt::print("Received relative angle. Only absolute angles are supported.\n");
    }
  }
  fmt::print("Pan: {:<4} Tilt: {:<4}\n", m_panAngle, m_tiltAngle);
}
