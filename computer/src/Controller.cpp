#include "Controller.h"
#include "SDL_events.h"
#include "SDL_gamepad.h"
#include "SDL_init.h"
#include "fmt/core.h"
#include <cstdint>

#define CONTROLLER_AXIS_THRESHOLD 0.1
#define NS_TO_SEC 0.000000001

Controller::Controller()
    : m_sensorFusion(SensorFusion(0.99f, 0.003f, 0.f)),
      m_tracking(Tracking::NONE), m_closeRequested(false), m_isConnected(false) {
  // Initialize SDL m_gamepad/Controller
  if (SDL_Init(SDL_INIT_GAMEPAD | SDL_INIT_TIMER) != 0) {
    fmt::print("Error initializing SDL\n{}\n", SDL_GetError());
    return;
  }
  SDL_SetGamepadEventsEnabled(true);

  // Check for connected controllers
  int numGamepads;
  SDL_JoystickID *id = SDL_GetGamepads(&numGamepads);
  if (numGamepads < 1) {
    fmt::print("No joysticks available\n");
  } else {
    m_gamepad = SDL_OpenGamepad(*id);
    if (m_gamepad == nullptr) {
      fmt::print("Error opening m_gamepad: {}\n", SDL_GetError());
      m_isConnected = false;
    } else {
      // Print m_gamepad name
      m_isConnected = true;
      fmt::print("Gamepad name: {}\n", SDL_GetGamepadName(m_gamepad));
    }
    unsigned int props = SDL_GetGamepadProperties(m_gamepad);
    if (SDL_GetBooleanProperty(props, SDL_PROP_GAMEPAD_CAP_RUMBLE_BOOLEAN,
                               false)) {
      fmt::print("Joystick supports rumble and LED\n");
      SetLED(0, 0, 0);
    } else {
      fmt::print("Configure Joystick in settings\n");
    }
  }
}

Controller::~Controller() {
  if (m_gamepad != nullptr) {
    SDL_CloseGamepad(m_gamepad);
  }
}

std::pair<float, float> Controller::GetLeftAxis() const {
  return std::make_pair(m_axisLeftX, m_axisLeftY);
}

std::pair<float, float> Controller::GetRightAxis() const {
  return std::make_pair(m_axisRightX, m_axisRightY);
}

std::pair<float, float> Controller::GetTouchAxis() const {
  return std::make_pair(m_axisTouchX, m_axisTouchY);
}

std::pair<float, float> Controller::GetAngles() const {
  return std::make_pair(m_panAngle, m_tiltAngle);
}

void Controller::Update() {
  Axis gyroData;
  Axis accelData;
  Axis angles;
  static int16_t leftX, leftY = 0;
  static int16_t rightX, rightY = 0;

  static int64_t lastTime = 0;

  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    switch (event.type) {
    case SDL_EVENT_GAMEPAD_REMOVED:
      fmt::print("Joystick disconnected\n");
      m_isConnected = false;
      break;

    case SDL_EVENT_GAMEPAD_ADDED:
      fmt::print("Joystick connected\n");
      m_isConnected = true;
      m_gamepad = SDL_OpenGamepad(event.gdevice.which);
      break;

    case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
      switch (event.gbutton.button) {
      case SDL_GAMEPAD_BUTTON_DPAD_UP:
        fmt::print("Tracking: NONE\n");
        m_tracking = Tracking::NONE;
        DisableSensors();
        ResetLED();
        break;
      case SDL_GAMEPAD_BUTTON_DPAD_DOWN:
        fmt::print("Tracking: JOYSTICK\n");
        m_tracking = Tracking::JOYSTICK;
        DisableSensors();
        ResetLED();
        break;
      case SDL_GAMEPAD_BUTTON_DPAD_RIGHT:
        fmt::print("Tracking: MOVEMENT\n");
        m_tracking = Tracking::MOVEMENT;
        EnableSensors();
        ResetLED();
        break;
      case SDL_GAMEPAD_BUTTON_DPAD_LEFT:
        fmt::print("Tracking: VISION\n");
        m_tracking = Tracking::VISION;
        DisableSensors();
        ResetLED();
        break;
      case SDL_GAMEPAD_BUTTON_TOUCHPAD:
        fmt::print("Tracking: TOUCH\n");
        m_tracking = Tracking::TOUCH;
        DisableSensors();
        ResetLED();
        break;
      case SDL_GAMEPAD_BUTTON_NORTH:
        fmt::print("Close requested\n");
        m_closeRequested = true;
        break;
      case SDL_GAMEPAD_BUTTON_SOUTH:
        m_confirmPressed = true;
        break;
      case SDL_GAMEPAD_BUTTON_WEST:
        if (m_tracking == Tracking::MOVEMENT) {
          fmt::print("Reset sensor reference\n");
          m_sensorFusion.ResetAbsoluteAngles();
        }
        break;
      case SDL_GAMEPAD_BUTTON_EAST:
        fmt::print("Home\n");
        m_homePressed = true;
        break;
      default:
        break;
      }
      break;

    case SDL_EVENT_GAMEPAD_TOUCHPAD_MOTION:
      if (m_tracking == Tracking::TOUCH) {
        m_axisTouchX = (event.gtouchpad.x - 0.5) * 2;
        m_axisTouchY = (event.gtouchpad.y - 0.5) * 2;
      }

      break;

    case SDL_EVENT_GAMEPAD_AXIS_MOTION:
      switch (event.gaxis.axis) {
      case SDL_GAMEPAD_AXIS_LEFTX:
        leftX = event.gaxis.value;
        break;
      case SDL_GAMEPAD_AXIS_LEFTY:
        leftY = event.gaxis.value;
        break;
      case SDL_GAMEPAD_AXIS_RIGHTX:
        rightX = event.gaxis.value;
        break;
      case SDL_GAMEPAD_AXIS_RIGHTY:
        rightY = event.gaxis.value;
        break;
      default:
        break;
      }
      break;
    case SDL_EVENT_GAMEPAD_SENSOR_UPDATE:
      if (event.gsensor.sensor == SDL_SENSOR_GYRO) {
        // Get the gyro data
        gyroData.pitch = event.gsensor.data[0];
        gyroData.yaw = event.gsensor.data[1];
        gyroData.roll = event.gsensor.data[2];
      } else if (event.gsensor.sensor == SDL_SENSOR_ACCEL) {
        // Get the accelerometer data
        accelData.pitch = event.gsensor.data[0];
        accelData.yaw = event.gsensor.data[1];
        accelData.roll = event.gsensor.data[2];
        m_sensorFusion.getAngles(
            accelData, gyroData,
            (float)(event.gsensor.sensor_timestamp - lastTime) * NS_TO_SEC,
            &angles);
        m_panAngle = -angles.yaw;
        m_tiltAngle = angles.pitch;
        lastTime = event.gsensor.sensor_timestamp;
      }
      break;

    default:
      break;
    }
  }
  if (!m_isConnected) {
    return;
  }

  // Handle axis inputs
  if (m_tracking == Tracking::JOYSTICK || m_tracking == Tracking::VISION) {
    std::tie(m_axisLeftX, m_axisLeftY) = NomalizeAxis(leftX, leftY);
    std::tie(m_axisRightX, m_axisRightY) = NomalizeAxis(rightX, rightY);
  }
}

bool Controller::CloseRequested() const { return m_closeRequested; }

bool Controller::IsConfirmPressed() {
  if(m_confirmPressed){
    m_confirmPressed = false;
    return true;
  }
  return false;
}

bool Controller::IsHomePressed(){
  if(m_homePressed){
    m_homePressed = false;
    return true;
  }
  return false;
}

Tracking Controller::GetTracking() const { return m_tracking; }

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

void Controller::ResetLED() const {
  switch (m_tracking) {
  case Tracking::NONE:
    SetLED(0, 0, 0);
    SetRumble(0xffff, 0, 1000);
    break;
  case Tracking::JOYSTICK:
    SetLED(0, 255, 0);
    break;
  case Tracking::MOVEMENT:
    SetLED(0, 0, 255);
    break;
  case Tracking::VISION:
    SetLED(255, 255, 255);
    break;
  case Tracking::TOUCH:
    SetLED(255, 255, 0);
  }
}

void Controller::SetLED(int red, int green, int blue) const {
  // Set LED color
  SDL_SetGamepadLED(m_gamepad, red, green, blue);
}

void Controller::SetRumble(uint16_t lowFreq, uint16_t highFreq,
                           uint32_t timeMS) const {
  // Set rumble
  SDL_RumbleGamepad(m_gamepad, lowFreq, highFreq, timeMS);
}

void Controller::DisableSensors() const {
  // Disable sensors
  SDL_SetGamepadSensorEnabled(m_gamepad, SDL_SENSOR_GYRO, false);
  SDL_SetGamepadSensorEnabled(m_gamepad, SDL_SENSOR_ACCEL, false);
}

void Controller::EnableSensors() const {
  // Enable sensors
  SDL_SetGamepadSensorEnabled(m_gamepad, SDL_SENSOR_GYRO, true);
  SDL_SetGamepadSensorEnabled(m_gamepad, SDL_SENSOR_ACCEL, true);
}
