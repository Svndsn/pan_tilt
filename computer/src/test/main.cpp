#include "SDL_events.h"
#include "SDL_gamepad.h"
#include "SDL_init.h"
#include "SDL_joystick.h"
#include "SDL_timer.h"
#include "fmt/core.h"

#define FPS 5
#define FRAME_TIME 1000 / FPS

int main() {
  SDL_Init(SDL_INIT_GAMEPAD | SDL_INIT_TIMER);
  int numJoysticks;
  SDL_JoystickID *id = SDL_GetJoysticks(&numJoysticks);
  SDL_Gamepad *gamepad = SDL_OpenGamepad(*id);
  SDL_SetGamepadEventsEnabled(true);
  SDL_SetGamepadSensorEnabled(gamepad, SDL_SENSOR_GYRO, true);
  // SDL_SetGamepadSensorEnabled(gamepad, SDL_SENSOR_ACCEL, true);
  // for (int i = 0; i < 7; i++) {
  //   if (SDL_GamepadHasSensor(gamepad, (SDL_SensorType)i)) {
  //     fmt::print("Gamepad has sensor type {}\n", i);
  //   }
  // }
  bool isRunning = true;
  // Main loop
  while (isRunning) {
    auto startMS = SDL_GetTicks();
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      switch (event.type) {
      case SDL_EVENT_GAMEPAD_SENSOR_UPDATE:
        fmt::print("Gyro X: {:5.3}, Y: {:5.3}, Z: {:5.3}\n",
                   event.sensor.data[1], event.sensor.data[2],
                   event.sensor.data[3]);
        break;
      default:
        break;
      }
    }

    auto endMS = SDL_GetTicks();
    if (endMS - startMS < FRAME_TIME) {
      SDL_Delay(FRAME_TIME - (endMS - startMS));
    }
  }
  return 0;
}
