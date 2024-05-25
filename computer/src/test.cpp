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
  std::array<float, 5> panSetAngles = {30, 0, 90, 0, 100};
  std::array<float, 5> tiltSetAngles = {30, 0, 90, 0, 180};

  // Initialize SDL and controller
  Controller controller;

  // Initialize communication with the microcontroller
  AngleHandler angleHandler;

  // Generate file for logging
  // Get time as file name
  std::string filename = fmt::format("data/{}.csv", std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
  fmt::print("File: {}\n", filename);
  std::ofstream file(filename, std::ios::out);
      file << fmt::format( "{},{},{},{},{},{},{}\n", 
                          "panSetAngles", "tiltSetAngles", 
                          "currentTimeMS","panAngle", "tiltAngle", 
                          "panVoltage", "tiltVoltage");
  // For timings
  auto startTimeMS = SDL_GetTicks();

  for (int i = 0; i < 5; ++i) {
    while (1) {
      angleHandler.SetAbsoluteAngles(panSetAngles[i], tiltSetAngles[i]);
      angleHandler.ReceiveUART();
      // Current time
      auto currentTimeMS = SDL_GetTicks() - startTimeMS;
      auto [panAngle, tiltAngle] = angleHandler.GetAngles();
      auto [panVoltage, tiltVoltage] = angleHandler.GetVoltage();
      file << fmt::format( "{},{},{},{},{},{},{}\n", 
                          panSetAngles[i], tiltSetAngles[i], 
                          currentTimeMS,panAngle, tiltAngle, 
                          panVoltage, tiltVoltage);
      fmt::print ("{},{},{},{},{},{}\n", panSetAngles[i], tiltSetAngles[i], panAngle, tiltAngle, panVoltage, tiltVoltage);
      SDL_Delay(5);

      if(panSetAngles[i]-1.3 < panAngle && panAngle < panSetAngles[i]+1.3 && tiltSetAngles[i]-0.7 < tiltAngle && tiltAngle < tiltSetAngles[i]+0.7){
        SDL_Delay(2000);
        break;
      }
    }
  }
  // Save file
  file.close();
  
  return 0;
}
