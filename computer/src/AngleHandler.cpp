#include "AngleHandler.h"
#include "serialib.h"
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fmt/core.h>
#include <string>
#include <vector>

#define PAN_ANGLE_MIN -180
#define PAN_ANGLE_MAX 180
#define TILT_ANGLE_MIN -180
#define TILT_ANGLE_MAX 180
#define MAX_DATA_LENGTH 255

// Uart Protocol
// Format:
// Header: 2 bytes (CommandType, Data length)
// Data: 0-255 bytes
// Checksum: 1 byte

// CommandType:
// 0: Setpoint Pan Absolute (SendSetpoint)
// 1: Setpoint Pan Relative (SendSetpoint)
// 2: Angle Pan (ReceiveAngles)
// 3: Setpoint Tilt Absolute (SendSetpoint)
// 4: Setpoint Tilt Relative (SendSetpoint)
// 5: Angle Tilt (ReceiveAngles)
// 6: Debug string (ReceiveDebugString)

AngleHandler::AngleHandler() {
  // Connection to serial port
  int err = m_serialConnection.openDevice(FindSerialDevice().c_str(), 115200,
                                          SERIAL_DATABITS_8, SERIAL_PARITY_EVEN,
                                          SERIAL_STOPBITS_1);
  if (err < 1)
    fmt::print("Serial error: Error code: {}\n", err);

  m_serialConnection.flushReceiver();
  m_RxState = RxState::Type;
  m_panAngle = 0;
  m_tiltAngle = 0;
  m_panVoltage = 0;
  m_tiltVoltage = 0;
  m_panSetpointAngle = 0;
  m_tiltSetpointAngle = 0;
}

AngleHandler::~AngleHandler() {
  if (m_serialConnection.isDeviceOpen()) {
    m_serialConnection.closeDevice();
  }
}

std::pair<float, float> AngleHandler::GetAngles() const {
  return std::make_pair(m_panAngle, m_tiltAngle);
}

std::pair<float, float> AngleHandler::GetSetpoints() const {
  return std::make_pair(m_panSetpointAngle, m_tiltSetpointAngle);
}

std::pair<float, float> AngleHandler::GetVoltage() const {
  return std::make_pair(m_panVoltage, m_tiltVoltage);
}

void AngleHandler::SetAbsoluteAngles(float absPanAngle, float absTiltAngle) {
  // Set the setpoint angles
  m_panSetpointAngle = absPanAngle;
  m_tiltSetpointAngle = absTiltAngle;

  // Bound the angles to the limits
  if (m_panSetpointAngle < PAN_ANGLE_MIN)
    m_panSetpointAngle = PAN_ANGLE_MIN;
  else if (m_panSetpointAngle > PAN_ANGLE_MAX)
    m_panSetpointAngle = PAN_ANGLE_MAX;

  if (m_tiltSetpointAngle < TILT_ANGLE_MIN)
    m_tiltSetpointAngle = TILT_ANGLE_MIN;
  else if (m_tiltSetpointAngle > TILT_ANGLE_MAX)
    m_tiltSetpointAngle = TILT_ANGLE_MAX;

  // Send the absolute setpoint angles to the microcontroller
  SendSetpointUART(Angle::Pan, absPanAngle, false);
  SendSetpointUART(Angle::Tilt, absTiltAngle, false);
}

void AngleHandler::SetRelativeAngles(float relPanAngle, float relTiltAngle) {
  // Set the setpoint angles
  m_panSetpointAngle = m_panAngle + relPanAngle;
  m_tiltSetpointAngle = m_tiltAngle + relTiltAngle;

  // Bound the angles to the limits
  if (m_panSetpointAngle < PAN_ANGLE_MIN)
    m_panSetpointAngle = PAN_ANGLE_MIN;
  else if (m_panSetpointAngle > PAN_ANGLE_MAX)
    m_panSetpointAngle = PAN_ANGLE_MAX;

  if (m_tiltSetpointAngle < TILT_ANGLE_MIN)
    m_tiltSetpointAngle = TILT_ANGLE_MIN;
  else if (m_tiltSetpointAngle > TILT_ANGLE_MAX)
    m_tiltSetpointAngle = TILT_ANGLE_MAX;

  // Send the relative setpoint angles to the microcontroller
  SendSetpointUART(Angle::Pan, relPanAngle, true);
  SendSetpointUART(Angle::Tilt, relTiltAngle, true);
}

void AngleHandler::SendSetpointUART(Angle type, float setpoint,
                                    bool relative) const {
  // Store the setpoint angle in a union to access the bytes
  union {
    float angle;
    uint8_t data[4];
  } u;

  u.angle = setpoint;

  uint8_t header[2] = {0, 4};
  if (type == Angle::Pan) {
    if (relative) {
      header[0] = (uint8_t)CommandType::SetpointPanRelative;
    } else {
      header[0] = (uint8_t)CommandType::SetpointPanAbsolute;
    }
  } else if (type == Angle::Tilt) {
    if (relative) {
      header[0] = (uint8_t)CommandType::SetpointTiltRelative;
    } else {
      header[0] = (uint8_t)CommandType::SetpointTiltAbsolute;
    }
  }

  // Calculate the checksum
  uint8_t checksum = 0;
  CheckSum(header, 2, checksum);
  CheckSum(u.data, 4, checksum);

  // Send the data length
  m_serialConnection.writeBytes(header, 2);

  // Send the setpoint angle
  m_serialConnection.writeBytes(u.data, 4);

  // Send the checksum
  m_serialConnection.writeBytes(&checksum, 1);
}

void AngleHandler::ReceiveUART() {
  // Read the lastest data from the serial port
  uint8_t receivedBytes[MAX_DATA_LENGTH];
  int nBytes = m_serialConnection.readBytes(receivedBytes, MAX_DATA_LENGTH, 1);
  if (nBytes < 0) {
    fmt::print("Error while reading bytes. Error code: {}\n", nBytes);
    return;
  } else if (nBytes == 0) {
    // No data received
    return;
  }
  for (int i = 0; i < nBytes; i++) {
    m_RxDataQueue.push(receivedBytes[i]);
  }
  static uint8_t dataLength = 0;
  static CommandType type;
  std::vector<uint8_t> data;
  while (!m_RxDataQueue.empty()) {
    switch (m_RxState) {
    case RxState::Type: {
      if (m_RxDataQueue.size() >= 1) {
        type = static_cast<CommandType>(m_RxDataQueue.front());
        m_RxDataQueue.pop();
        if ((uint8_t)type > 8) {
          fmt::print("Received unknown type\n");
        } else {
          m_RxState = RxState::Length;
        }
      }
      break;
    }
    case RxState::Length: {
      if (m_RxDataQueue.size() >= 1) {
        dataLength = m_RxDataQueue.front();
        m_RxDataQueue.pop();
        if (dataLength == 0) {
          // Error, no data
          m_RxState = RxState::Type;
        } else {
          // Next state to get the data
          m_RxState = RxState::Data;
          data.clear();
        }
      }
      break;
    }
    case RxState::Data: {
      // Check if there is enough data
      if (m_RxDataQueue.size() < dataLength) {
        // Not enough data, wait for the rest
        return;
      }

      for (int i = 0; i < dataLength; i++) {
        data.push_back(m_RxDataQueue.front());
        m_RxDataQueue.pop();
      }

      // All data received, go to the next state after parsing the data
      m_RxState = RxState::Checksum;
      break;
    }
    case RxState::Checksum: {
      if (m_RxDataQueue.size() < 1) {
        // Not enough data, wait for the checksum
        return;
      }
      uint8_t checksum = 0;
      uint8_t rxtype = (uint8_t)type;
      CheckSum(&rxtype, 1, checksum);
      CheckSum(&dataLength, 1, checksum);
      CheckSum(data.data(), data.size(), checksum);
      if (checksum != m_RxDataQueue.front()) {
        fmt::print("Checksum error\n");
        m_RxState = RxState::Type;
        m_RxDataQueue.pop();
        break;
      }
      m_RxDataQueue.pop();
      // Parse the data
      switch (type) {
      case CommandType::AnglePan: {
        if (dataLength > 4 || data.size() < 4) {
          break;
        }
        union {
          float angle;
          uint8_t data[4];
        } u;
        u.data[0] = data.at(0);
        u.data[1] = data.at(1);
        u.data[2] = data.at(2);
        u.data[3] = data.at(3);
        m_panAngle = u.angle;
        break;
      }
      case CommandType::AngleTilt: {
        if (dataLength > 4 || data.size() < 4) {
          break;
        }
        union {
          float angle;
          uint8_t data[4];
        } u;
        u.data[0] = data.at(0);
        u.data[1] = data.at(1);
        u.data[2] = data.at(2);
        u.data[3] = data.at(3);
        m_tiltAngle = u.angle;
      } break;
      case CommandType::VoltagePan: {
        if (dataLength > 4 || data.size() < 4) {
          break;
        }
        union {
          float voltage;
          uint8_t data[4];
        } u;
        u.data[0] = data.at(0);
        u.data[1] = data.at(1);
        u.data[2] = data.at(2);
        u.data[3] = data.at(3);
        m_panVoltage = u.voltage;
      } break;
      case CommandType::VoltageTilt: {
        if (dataLength > 4 || data.size() < 4) {
          break;
        }
        union {
          float voltage;
          uint8_t data[4];
        } u;
        u.data[0] = data.at(0);
        u.data[1] = data.at(1);
        u.data[2] = data.at(2);
        u.data[3] = data.at(3);
        m_tiltVoltage = u.voltage;
      } break;
      case CommandType::DebugString: {
        // Print the debug string
        for (auto d : data) {
          fmt::print("{:c}", d);
        }
        fmt::print("\n");
        break;
      }
      default:
        fmt::print("Received unsupported Rx type: {}\n", (uint8_t)type);
        break;
      }
      m_RxState = RxState::Type;
      break;
    }
    }
  }
}

void AngleHandler::CheckSum(uint8_t *data, size_t length,
                            uint8_t &checksum) const {
  for (size_t i = 0; i < length; i++) {
    checksum ^= data[i];
  }
}

std::string AngleHandler::FindSerialDevice() const {
  // Find the serial devices connected to the computer
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
  fmt::print("Select the device index: ");
  size_t deviceIndex;
  std::cin >> deviceIndex;
  assert(!std::cin.fail() && deviceIndex < devices.size() &&
         "Invalid device index");
  device = devices[deviceIndex];
  fmt::print("Selected device: {}\n", devices[deviceIndex]);
#endif

  return device;
}
