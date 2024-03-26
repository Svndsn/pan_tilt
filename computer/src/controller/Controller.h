#pragma once
#include "SDL_joystick.h"
#include <utility>

enum class ControllerButtons {
  CROSS = 0,
  CIRCLE = 1,
  SQUARE = 2,
  TRIANGLE = 3,
  L1 = 4,
  R1 = 5,
  L2 = 6,
  R2 = 7,
  SHARE = 8,
  OPTIONS = 9,
  PS = 10,
  L3 = 11,
  R3 = 12,
  TOUCHPAD = 13
};

enum class ControllerAxis { LEFT_X = 0, LEFT_Y = 1, RIGHT_X = 2, RIGHT_Y = 3 };

enum class Target { NONE, MANUAL, AUTO };

class Controller {
public:
  // Constructor
  Controller();

  // Destructor
  ~Controller();

  // Open controller
  bool Open();

  // Get the current values of the left joystick from -1 to 1
  std::pair<float, float> GetAxis() const;

  // Update the controller state
  void Update();

  // Close requested
  bool CloseRequested() const;

  // Toggle tracking of systen
  Target Tracking();

  // Toggle tracking of systen
  void SetLED(int red, int green, int blue) const;

private:
  std::pair<float, float> NomalizeAxis(int16_t &x, int16_t &y) const;

  SDL_Joystick *m_joystick;
  float m_axisX, m_axisY = 0.0f;
  bool m_closeRequested = false;
  bool m_isConnected = false;
  Target m_tracking = Target::NONE;
};
