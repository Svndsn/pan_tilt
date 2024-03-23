#include "SDL_init.h"
#include "SDL_joystick.h"
#include "SDL_stdinc.h"
#include "SDL_timer.h"
#include "fmt/core.h"

#define AXIS_DEAD_ZONE 5000

int main() {
  // Initialize SDL joystick
  if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_TIMER) != 0) {
    fmt::print("Error initializing SDL Joystick\n{}\n", SDL_GetError());
    return 1;
  }

  int numJoysticks;
  SDL_JoystickID *id = SDL_GetJoysticks(&numJoysticks);
  if (numJoysticks < 1) {
    fmt::print("No joysticks available\n");
    return 0;
  } else {
    fmt::print("Joystick available: {}\n", numJoysticks);
  }

  // Open joystick
  SDL_Joystick *joystick = SDL_OpenJoystick(*id);
  if (joystick == nullptr) {
    fmt::print("Error opening joystick: {}\n", SDL_GetError());
    return 1;
  } else {
    fmt::print("Joystick opened\n");
    // Print joystick name
    fmt::print("Joystick name: {}\n", SDL_GetJoystickName(joystick));
  }

  // Print number of axes
  fmt::print("Number of axes: {}\n", SDL_GetNumJoystickAxes(joystick));
  // Print number of buttons
  fmt::print("Number of buttons: {}\n", SDL_GetNumJoystickButtons(joystick));
  // Print number of balls
  fmt::print("Number of balls: {}\n", SDL_GetNumJoystickBalls(joystick));
  // Print number of hats
  fmt::print("Number of hats: {}\n", SDL_GetNumJoystickHats(joystick));
  // Get data from joystick
  bool running = true;
  while (running) {
    SDL_UpdateJoysticks();
    if (SDL_GetJoystickButton(joystick, 3) == 1) {
      fmt::print("Stoping\n");
      running = false;
      break;
    }
    if (SDL_GetJoystickButton(joystick, 0) == 1) {
      fmt::print("Button X Pressed\n");
    }
    Sint16 x_axis = SDL_GetJoystickAxis(joystick, 0);
    if (x_axis < AXIS_DEAD_ZONE && x_axis > -AXIS_DEAD_ZONE) {
      x_axis = 0;
    }
    Sint16 y_axis = SDL_GetJoystickAxis(joystick, 1);
    if (y_axis < AXIS_DEAD_ZONE && y_axis > -AXIS_DEAD_ZONE) {
      y_axis = 0;
    }
    fmt::print("X: {:>6}, Y: {:>6}\n", x_axis, y_axis);
    SDL_Delay(100);
  }

  // Close joystick
  SDL_CloseJoystick(joystick);
  return 0;
}
