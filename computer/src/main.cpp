// Libraries
#include <SDL_timer.h>
#include <fmt/core.h>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

// Project
#include "AngleHandler.h"
#include "Controller.h"
#include "Vision.h"

#define FPS 30
#define FRAME_TIME 1000 / FPS

int main() {
  // Initialize SDL and controller
  Controller controller;

  // Initialize communication with the microcontroller
  AngleHandler angleHandler;

  // Initialize the vision tracking and video capture
  Vision vision("Tracking", 1);

  // For timings
  auto startMS = SDL_GetTicks();
  auto lastConfirmed = SDL_GetTicks();

  // Main loop
  while (!controller.CloseRequested()) {

    // Update controller inputs
    controller.Update();

    // Handle the timing of the loop
    auto endMS = SDL_GetTicks();
    if (endMS - startMS > FRAME_TIME) {
      startMS = endMS;
    } else {
      continue;
    }

    // This section of code will run at FPS
    ///////////////////////////////////////

    // Get the current frame from the camera
    vision.UpdateCamera();

    // Handle tracking
    switch (controller.GetTracking()) {
    case Tracking::NONE:
        angleHandler.SetAbsoluteAngles(0, 0);
      break;
    case Tracking::JOYSTICK: {
        const auto [pan, tilt] = controller.GetLeftAxis();
        angleHandler.SetRelativeAngles(pan * 5, tilt * 5);
      break;
    }
    case Tracking::MOVEMENT: {
      const auto [pan, tilt] = controller.GetAngles();
      angleHandler.SetAbsoluteAngles(pan, tilt);
      break;
    }
    case Tracking::TOUCH: {
      const auto [pan, tilt] = controller.GetTouchAxis();
      angleHandler.SetRelativeAngles(pan * 5, tilt * 5);
      break;
    }
    case Tracking::VISION: {
      if (!vision.IsTrackingActive()) {
        // Select the tracking box
        const auto [posX, posY] = controller.GetRightAxis();
        const auto [width, height] = controller.GetLeftAxis();

        vision.UpdateTrackingBox(posX * 8, posY * 8, width * 5, height * 5);

        // Press the confirm button (X) to start tracking
        if (controller.IsConfirmPressed()) {
          // Only confirm if the last confirmation was minimum 500ms ago
          if (lastConfirmed + 500 < SDL_GetTicks()) {
            lastConfirmed = SDL_GetTicks();
            vision.SetTrackingActive(true);
          }
        }
      } else {
        // Tracking active
        vision.UpdateTracker();

        // Calculate the angles based on
        const auto [pan2Center, tilt2Center] = vision.GetAngles();
        angleHandler.SetRelativeAngles(pan2Center, tilt2Center);

        // Press the confirm button (X) to stop tracking
        if (controller.IsConfirmPressed()) {
          if (lastConfirmed + 500 < SDL_GetTicks()) {
            lastConfirmed = SDL_GetTicks();
            vision.SetTrackingActive(false);
          }
        }
      }
      if (vision.IsTrackingActive()) {
        vision.PutText("Tracking", 10, 20, 0.5, cv::Scalar(0, 255, 0), 1);
      } else {
        vision.PutText("Not Tracking", 10, 20, 0.5, cv::Scalar(0, 0, 255), 1);
      }
      break;
    }
    default:
      fmt::print("Unknown tracking method: {}\n", (int)controller.GetTracking());
      break;
    }
    // The current angle of pan and tilt
    vision.PutText(fmt::format("Angle: Pan: {:<4} Tilt: {:<4}",
                               angleHandler.GetAngles().first,
                               angleHandler.GetAngles().second),
                   10, 40, 0.5, cv::Scalar(255, 255, 255), 1);

    // The setpoint angle of pan and tilt
    vision.PutText(fmt::format("Setpoint: Pan: {:<4} Tilt: {:<4}",
                               angleHandler.GetSetpoints().first,
                               angleHandler.GetSetpoints().second),
                   10, 60, 0.5, cv::Scalar(255, 255, 255), 1);

    // Update the window with the new frame
    vision.UpdateWindow();
  }

  return 0;
}
