#include "AngleHandler.h"
#include "Controller.h"
#include "SDL_timer.h"
#include "fmt/core.h"

#define FPS 15
#define FRAME_TIME 1000 / FPS

int main() {
  // Initialize SDL and controller
  Controller controller;

  AngleHandler angleHandler;
  // angleHandler.GetAngles();

  bool isRunning = true;
  auto startMS = SDL_GetTicks();
  // Main loop
  while (isRunning) {

    // Update controller inputs
    controller.Update();

    // Check if the user has requested to close the application
    if (controller.CloseRequested()) {
      isRunning = false;
      break;
    }

    // Handle the timing of the loop
    auto endMS = SDL_GetTicks();
    if (endMS - startMS > FRAME_TIME) {
      startMS = endMS;
    } else {
      continue;
    }

    // This section of code will run at FPS
    // Handle tracking
    switch (controller.GetTracking()) {
    case Target::NONE:
      break;
    case Target::MANUAL: {
      fmt::print("Joystick Right Tracking X: {:6.3} Y: {:6.3}\n",
                 controller.GetRightAxis().first,
                 controller.GetRightAxis().second);
      fmt::print("Joystick Left Tracking X: {:6.3} Y: {:6.3}\n",
                 controller.GetLeftAxis().first,
                 controller.GetLeftAxis().second);

      break;
    }
    case Target::MOVEMENT: {
      const auto [yaw, pitch] = controller.GetAngles();
      fmt::print("Movement Tracking X: {:6.3} Y: {:6.3}\n", yaw, pitch);
      break;
    }
    case Target::TOUCH: {
      const auto [yaw, pitch] = controller.GetTouchAxis();
      fmt::print("Touch Tracking X: {:6.3} Y: {:6.3}\n", yaw, pitch);
      break;
    }
    case Target::VISION: {
      // OpenCV code here
      break;
    default:
      break;
    }
    }
  }

  return 0;
}
