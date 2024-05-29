// Libraries
#include <SDL_timer.h>
#include <array>
#include <fmt/core.h>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include "fstream"
#include "chrono"

// Project
#include "AngleHandler.h"
#include "Controller.h"

#define FPS 30
#define FRAME_TIME 1000 / FPS

int main() {
  std::array<float, 2> SetAngles = {0.f, 90.f};

  // Initialize SDL and controller
  Controller controller;

  // Initialize communication with the microcontroller
  AngleHandler angleHandler;

  // Generate file for logging
  // Get time as file name
  std::string filename = fmt::format("{}.csv", std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
  fmt::print("File: {}\n", filename);
  std::ofstream file(filename, std::ios::out);
      file << fmt::format( "{},{},{},{},{},{},{},{}\n", "TestN",
                          "panSetAngles", "tiltSetAngles", 
                          "currentTimeMS","panAngle", "tiltAngle", 
                          "panVoltage", "tiltVoltage");
  // For timings
  auto startTimeMS = SDL_GetTicks();

  int count = 0;
  for (int i = 0; i < 60; ++i) {
    while (1) {
      angleHandler.SetAbsoluteAngles(SetAngles[i%2], SetAngles[i%2]);
      angleHandler.ReceiveUART();
      // Current time
      auto currentTimeMS = SDL_GetTicks() - startTimeMS;
      auto [panAngle, tiltAngle] = angleHandler.GetAngles();
      auto [panVoltage, tiltVoltage] = angleHandler.GetVoltage();
      file << fmt::format( "{},{},{},{},{},{},{},{}\n", i/2,
                          SetAngles[i%2], SetAngles[i%2], 
                          currentTimeMS,panAngle, tiltAngle, 
                          panVoltage, tiltVoltage);
      fmt::print ("{},{},{}\n",i/2, panAngle, tiltAngle);
      SDL_Delay(5);

      if (SetAngles[i % 2] - 1.3 < panAngle &&
          panAngle < SetAngles[i % 2] + 1.3 &&
          SetAngles[i % 2] - 0.7 < tiltAngle &&
          tiltAngle < SetAngles[i % 2] + 0.7) {
        count++;
        if (count > 100) {
          SDL_Delay(500);
          count = 0;
          break;
        }
      }
    }
  }
  // Save file
  file.close();
  
  return 0;
}
