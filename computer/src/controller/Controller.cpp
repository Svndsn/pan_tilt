#include "Controller.h"
#include "SDL_events.h"
#include "SDL_init.h"
#include "SDL_joystick.h"
#include "fmt/core.h"
#include <cstdint>

#define CONTROLLER_AXIS_THRESHOLD 0.1

Controller::Controller() {
  // Initialize SDL m_joystick/Controller
  if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_TIMER) != 0) {
    fmt::print("Error initializing SDL\n{}\n", SDL_GetError());
    return;
  }
  SDL_SetJoystickEventsEnabled(true);

  // Check for connected controllers
  int numJoysticks;
  SDL_JoystickID *id = SDL_GetJoysticks(&numJoysticks);
  if (numJoysticks < 1) {
    fmt::print("No joysticks available\n");
  } else {
    m_joystick = SDL_OpenJoystick(*id);
    if (m_joystick == nullptr) {
      fmt::print("Error opening m_joystick: {}\n", SDL_GetError());
      m_isConnected = false;
    } else {

      // Print m_joystick name
      fmt::print("Joystick name: {}\n", SDL_GetJoystickName(m_joystick));
    }
    unsigned int props = SDL_GetJoystickProperties(m_joystick);
    if (SDL_GetBooleanProperty(props, SDL_PROP_JOYSTICK_CAP_RUMBLE_BOOLEAN,
                               SDL_FALSE)) {
      fmt::print("Joystick supports rumble and LED\n");
      SetLED(0, 0, 0);
    } else {
      fmt::print("Configure Joystick in settings\n");
    }
  }
}

Controller::~Controller() {
  if (m_joystick != nullptr) {
    SDL_CloseJoystick(m_joystick);
  }
}

std::pair<float, float> Controller::GetAxis() const {
  return std::make_pair(m_axisX, m_axisY);
}

void Controller::Update() {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (event.type == SDL_EVENT_JOYSTICK_BUTTON_DOWN) {
      if (event.jbutton.button == 0) { // ⛌ Button
        // Toggle tracking mode
        switch (m_tracking) {
        case Target::NONE:
          SetLED(0, 255, 0);
          m_tracking = Target::MANUAL;
          break;
        case Target::MANUAL:
          SetLED(0, 0, 255);
          m_tracking = Target::AUTO;
          break;
        case Target::AUTO:
          SetLED(0, 255, 0);
          m_tracking = Target::MANUAL;
        default:
          break;
        }
      } else if (event.jbutton.button == 1) { // ○ Button
        // Reset the camera position or opencv tracking
        fmt::print("Reset Camera Position\n");
      } else if (event.jbutton.button == 3) { // △ Button
        // close the application
        m_closeRequested = true;
      }
    } else if (event.type == SDL_EVENT_JOYSTICK_REMOVED) {
      m_isConnected = false;
      fmt::print("Joystick disconnected\n");
    }
  }

  int16_t x = SDL_GetJoystickAxis(m_joystick, (int)ControllerAxis::LEFT_X);
  int16_t y = SDL_GetJoystickAxis(m_joystick, (int)ControllerAxis::LEFT_Y);
  std::tie(m_axisX, m_axisY) = NomalizeAxis(x, y);
}

bool Controller::CloseRequested() const { return m_closeRequested; }

Target Controller::Tracking() { return m_tracking; }

std::pair<float, float> Controller::NomalizeAxis(int16_t &x, int16_t &y) const {
  float fx = x;
  float fy = y;
  if (fx < 0) {
    fx /= (float)SDL_JOYSTICK_AXIS_MIN * -1;
    fx = fx < -CONTROLLER_AXIS_THRESHOLD ? fx : 0;
  } else if (fx > 0) {
    fx /= (float)SDL_JOYSTICK_AXIS_MAX;
    fx = fx > CONTROLLER_AXIS_THRESHOLD ? fx : 0;
  }

  if (fy < 0) {
    fy /= (float)SDL_JOYSTICK_AXIS_MIN * -1;
    fy = fy < -CONTROLLER_AXIS_THRESHOLD ? fy : 0;
  } else if (fy > 0) {
    fy /= (float)SDL_JOYSTICK_AXIS_MAX;
    fy = fy > CONTROLLER_AXIS_THRESHOLD ? fy : 0;
  }

  return std::make_pair(fx, fy);
}

void Controller::SetLED(int red, int green, int blue) const {
  // Set LED color
  SDL_SetJoystickLED(m_joystick, red, green, blue);
}
