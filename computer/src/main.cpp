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

    angleHandler.ReceiveUART();

    if (controller.IsHomePressed()) {
      fmt::print("Homing\n");
      for (int i = 0; i < 10; ++i) {
        angleHandler.SetAbsoluteAngles(0.f, 0.f);
      }
      controller.SetTracking(Tracking::NONE);
    }

    // Handle tracking
    switch (controller.GetTracking()) {
    case Tracking::NONE:
        vision.PutText("No Tracking", 10, 20, 0.5, cv::Scalar(0, 0, 255), 1);
      break;
    case Tracking::JOYSTICK: {
        const auto [pan, tilt] = controller.GetLeftAxis();
        angleHandler.SetRelativeAngles(pan * 10, -tilt * 10);
        vision.PutText("Joystick Tracking", 10, 20, 0.5, cv::Scalar(0, 255, 0), 1);
      break;
    }
    case Tracking::MOVEMENT: {
      const auto [pan, tilt] = controller.GetAngles();
      angleHandler.SetAbsoluteAngles(pan, tilt);
      vision.PutText("Movement Tracking", 10, 20, 0.5, cv::Scalar(0, 255, 0), 1);
      break;
    }
    case Tracking::TOUCH: {
      const auto [pan, tilt] = controller.GetTouchAxis();
      angleHandler.SetRelativeAngles(pan * 2, tilt * 2);
      vision.PutText("Touch Tracking", 10, 20, 0.5, cv::Scalar(0, 255, 0), 1);
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
          vision.SetTrackingActive(true);
        }
      } else {
        // Tracking active
        vision.UpdateTracker();

        // Calculate the angles based on
        const auto [pan2Center, tilt2Center] = vision.GetAngles();
        const auto [pan, tilt] = angleHandler.GetAngles();
        angleHandler.SetAbsoluteAngles(pan + pan2Center, tilt - tilt2Center);
        // angleHandler.SetRelativeAngles(pan2Center, -tilt2Center);

        // Press the confirm button (X) to stop tracking
        if (controller.IsConfirmPressed()) {
          vision.SetTrackingActive(false);
        }
      }
      if (vision.IsTrackingActive()) {
        vision.PutText("Tracking", 10, 20, 0.5, cv::Scalar(0, 255, 0), 1);
      } else {
        vision.PutText("Set Tracking Box", 10, 20, 0.5, cv::Scalar(0, 0, 255), 1);
      }
      break;
    }
    default:
      fmt::print("Unknown tracking method: {}\n", (int)controller.GetTracking());
      break;
    }
    // The current voltage of pan and tilt
    vision.PutText(fmt::format("Voltage:  Pan: {:<5.4} Tilt: {:<5.4}",
                               angleHandler.GetVoltage().first,
                               angleHandler.GetVoltage().second),
                   10, 40, 0.5, cv::Scalar(255, 255, 255), 1);
    // The current angle of pan and tilt
    vision.PutText(fmt::format("Angle:    Pan: {:<5.4} Tilt: {:<5.4}",
                               angleHandler.GetAngles().first,
                               angleHandler.GetAngles().second),
                   10, 60, 0.5, cv::Scalar(255, 255, 255), 1);

    // The setpoint angle of pan and tilt
    vision.PutText(fmt::format("Setpoint: Pan: {:<5.4} Tilt: {:<5.4}",
                               angleHandler.GetSetpoints().first,
                               angleHandler.GetSetpoints().second),
                   10, 80, 0.5, cv::Scalar(255, 255, 255), 1);

    // Update the window with the new frame
    vision.UpdateWindow();
  }

  return 0;
}
