#pragma once
#include "SDL_gamepad.h"
#include "SensorFusion.h"
#include <utility>

enum class Tracking { NONE, JOYSTICK, MOVEMENT, TOUCH, VISION };

class Controller {
public:
  // Constructor
  Controller();

  // Destructor
  ~Controller();

  // Is controller open
  bool IsOpen();

  // Get the current values of the left joystick from -1 to 1
  std::pair<float, float> GetLeftAxis() const;

  // Get the current values of the right joystick from -1 to 1
  std::pair<float, float> GetRightAxis() const;

  // Get the current values of the right joystick from -1 to 1
  std::pair<float, float> GetTouchAxis() const;

  // Get the current pan and tilt angles of the controller (gyro + accel or
  // touchpad)
  std::pair<float, float> GetAngles() const;

  // Update the controller state
  void Update();

  // Close requested
  bool CloseRequested() const;

  // Get tracking of systen
  Tracking GetTracking() const;
  
  // Is comfirm button pressed
  bool IsConfirmPressed() const;

  // Toggle tracking of systen
  void SetLED(int red, int green, int blue) const;

  // Reset the LED based on the tracking
  void ResetLED() const;

  // Rumble the controller from 0 to 0xffff in N milliseconds
  void SetRumble(uint16_t lowFreq, uint16_t highFreq, uint32_t timeMS) const;

private:
  std::pair<float, float> NomalizeAxis(int16_t &x, int16_t &y) const;

  // Used to disable sensors when not in use
  void DisableSensors() const;
  void EnableSensors() const;

  SDL_Gamepad *m_gamepad;
  SensorFusion m_sensorFusion;
  Tracking m_tracking;

  float m_panAngle, m_tiltAngle = 0.0f;
  float m_axisRightX, m_axisRightY = 0.0f;
  float m_axisLeftX, m_axisLeftY = 0.0f;
  float m_axisTouchX, m_axisTouchY = 0.0f;
  bool m_closeRequested; // Triangle 
  bool m_isConnected;
};
