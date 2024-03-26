#include "Controller.h"
#include "SDL_timer.h"
#include "UARTAngleHandler.h"
#include "fmt/core.h"

#define FPS 10
#define FRAME_TIME 1000 / FPS

int main() {
  // Initialize SDL controller
  Controller controller;
  UARTAngleHandler uartAngleHandler;

  bool isRunning = true;
  // Main loop
  while (isRunning) {
    auto startMS = SDL_GetTicks();

    // Update controller inputs
    controller.Update();

    // Check if the user has requested to close the application
    if (controller.CloseRequested()) {
      isRunning = false;
      break;
    }

    // Handle tracking
    switch (controller.Tracking()) {
    case Target::NONE:
      break;
    case Target::AUTO:
      controller.SetLED(0, 0, 255);
      break;
    case Target::MANUAL:
      controller.SetLED(0, 255, 0);
      const auto [axisX, axisY] = controller.GetAxis();
      fmt::print("Manual Tracking X: {:6.3} Y: {:6.3}\n", axisX, axisY);
      break;
    }

    // Handle the timing of the loop
    auto endMS = SDL_GetTicks();
    // fmt::print("Frame time: {}\n", endMS - startMS);
    if (endMS - startMS < FRAME_TIME)
      SDL_Delay(FRAME_TIME - (endMS - startMS));
  }

  return 0;
}
