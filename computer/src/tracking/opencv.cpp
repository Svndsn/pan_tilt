#include "fmt/core.h"
#include <opencv2/opencv.hpp>

int main() {
  // Create a VideoCapture object to capture video from the default camera
  cv::VideoCapture cap(1);

  // Check if the camera is opened successfully
  if (!cap.isOpened()) {
    fmt::print(stderr, "Error: Unable to open the camera\n");
    return -1;
  }

  // Create a window to display the captured video
  cv::namedWindow("Video", cv::WINDOW_NORMAL);

  // Main loop to capture and display video
  while (true) {
    // Capture frame-by-frame
    cv::Mat frame;
    cap >> frame;

    // Check if the frame is empty (end of video)
    if (frame.empty()) {
      fmt::print(stderr, "Error: Unable to capture frame\n");
      break;
    }

    // Display the captured frame
    cv::imshow("Video", frame);

    // Check for user input to exit the loop (press 'q')
    if (cv::waitKey(1) == 'q')
      break;
  }

  // Release the VideoCapture object and close the OpenCV window
  cap.release();
  cv::destroyAllWindows();

  return 0;
}
