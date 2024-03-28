#include "SDL_events.h"
#include "SDL_gamepad.h"
#include "SDL_init.h"
#include "SDL_joystick.h"
#include "SDL_timer.h"
#include "SensorFusion.h"
#include "fmt/core.h"
#include "math.h"
#include <cstdint>

#define FPS 5
#define FRAME_TIME 1000 / FPS
#define NS_TO_SEC 0.000000001

int main() {
  SDL_Init(SDL_INIT_GAMEPAD | SDL_INIT_TIMER);
  int numJoysticks;
  SDL_JoystickID *id = SDL_GetJoysticks(&numJoysticks);
  SDL_Gamepad *gamepad = SDL_OpenGamepad(*id);
  SDL_SetGamepadEventsEnabled(true);
  SDL_SetGamepadSensorEnabled(gamepad, SDL_SENSOR_GYRO, true);
  SDL_SetGamepadSensorEnabled(gamepad, SDL_SENSOR_ACCEL, true);

  SensorFusion fusion(0.95, 0.f, 0.f, 0.f);

  ThreeAxis gyroData;
  ThreeAxis accelData;
  ThreeAxis angles;

  Uint64 lastTime = 0;
  bool firstRun = true;

  bool isRunning = true;
  // Main loop
  while (isRunning) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      switch (event.type) {
      case SDL_EVENT_GAMEPAD_SENSOR_UPDATE:
        if (event.gsensor.sensor == SDL_SENSOR_GYRO) {
          // Get the gyro data
          gyroData.pitch = event.gsensor.data[0];
          gyroData.yaw = event.gsensor.data[1];
          gyroData.roll = event.gsensor.data[2];
        } else if (event.gsensor.sensor == SDL_SENSOR_ACCEL) {
          if (firstRun) {
            // Get the reference time
            lastTime = event.gsensor.timestamp;
            firstRun = false;
          } else {
            // Get the accelerometer data
            accelData.pitch = event.gsensor.data[0];
            accelData.yaw = event.gsensor.data[1];
            accelData.roll = event.gsensor.data[2];
            fusion.getAngles(accelData, gyroData,
                             (float)(event.gsensor.timestamp - lastTime) *
                                 NS_TO_SEC,
                             &angles);
            fmt::print("Pitch: {:8.5}, Roll: {:8.5}, Yaw: {:8.5}\n",
                       angles.pitch, angles.roll, angles.yaw);
            lastTime = event.gsensor.timestamp;
          }
        }
        break;
      case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
        if (event.gbutton.button == SDL_GAMEPAD_BUTTON_NORTH) {
          isRunning = false;
        } else if (event.gbutton.button == SDL_GAMEPAD_BUTTON_WEST) {
          fusion.ResetAngles();
        }
      default:
        break;
      }
    }
  }
  return 0;
}
