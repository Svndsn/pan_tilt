#include "UARTAngleHandler.h"
#include "serialib.h"
#include <cassert>
#include <filesystem>
#include <fmt/core.h>

#define PAN_ANGLE_MIN -90
#define PAN_ANGLE_MAX 90

// TODO: Define the max velocity for the angles
#define TILT_ANGLE_MAX_VELOCITY 10
#define PAN_ANGLE_MAX_VELOCITY 10

UARTAngleHandler::UARTAngleHandler() {
  // Connection to serial port
  m_serialConnection.openDevice(FindSerialDevice().c_str(), 115200,
                                SERIAL_DATABITS_8, SERIAL_PARITY_NONE,
                                SERIAL_STOPBITS_1);
}

UARTAngleHandler::~UARTAngleHandler() {
  if (m_serialConnection.isDeviceOpen()) {
    m_serialConnection.closeDevice();
  }
}

void UARTAngleHandler::ReceiveAngles() {
  int16_t panAngle{}, tiltAngle{};

  // TODO: Implement the method to receive the angles from the UART

  m_angleMutex.lock();
  m_panAngle = panAngle;
  m_tiltAngle = tiltAngle;
  m_angleMutex.unlock();
}

void UARTAngleHandler::SendAngles() const {
  m_angleMutex.lock();
  int16_t panAngle = m_panAngle;
  int16_t tiltAngle = m_tiltAngle;
  m_angleMutex.unlock();

  // TODO: Implement the method to send the angles to the UART
  (void)panAngle;
  (void)tiltAngle;
}

std::pair<int16_t, int16_t> UARTAngleHandler::GetAngles() const {
  m_angleMutex.lock();
  int16_t panAngle = m_panAngle;
  int16_t tiltAngle = m_tiltAngle;
  m_angleMutex.unlock();
  return std::make_pair(panAngle, tiltAngle);
}

void UARTAngleHandler::SetAngles(int16_t targetPanAngle,
                                 int16_t targetTiltAngle) {
  m_angleMutex.lock();
  m_panAngle = targetPanAngle;
  m_tiltAngle = targetTiltAngle;
  m_angleMutex.unlock();
}

std::string UARTAngleHandler::FindSerialDevice() {
  std::string device{};

#if defined(__linux__) || defined(__APPLE__)
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
