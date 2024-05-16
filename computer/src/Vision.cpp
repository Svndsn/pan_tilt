// Libraries
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <fmt/core.h>

// Project
#include "Vision.h"

Vision::Vision(std::string windowName, int cameraIndex) {
  // Set the window name
  m_windowName = windowName;

  // Create a VideoCapture object to capture video from the default camera
  m_cap = cv::VideoCapture(cameraIndex);

  // Check if the camera is opened successfully
  if (!m_cap.isOpened()) {
    fmt::print(stderr, "Error: Unable to open the camera\n");
  }

  // Create a window to display the captured video
  cv::namedWindow(windowName, cv::WINDOW_NORMAL);

  // Set the tracking box
  SetTrackingBox(400, 200, 100, 100);

  // Set the tracking state
  m_trackingActive = false;

  // Fill the frame with black color
  m_frame = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
}

Vision::~Vision() {
  // Release the VideoCapture object
  m_cap.release();
  // Destroy the window
  cv::destroyAllWindows();
}

bool Vision::IsTrackingActive() const {
  // Return the tracking state
  return m_trackingActive;
}

void Vision::SetTrackingActive(bool active) {
  // Set the tracking state
  if (active) {
    fmt::print("Vision Tracking: Start Tracking\n");
    InitializeTracker();
  } else {
    fmt::print("Vision Tracking: Stop Tracking\n");
  }
  m_trackingActive = active;
}

void Vision::SetTrackingBox(int x, int y, int width, int height) {
  // Set the tracking box
  m_trackingBox = cv::Rect(x, y, width, height);
}

void Vision::UpdateTrackingBox(int dx, int dy, int dwidth, int dheight){
  // Update the tracking box
  m_trackingBox.x += dx - dwidth / 2;
  m_trackingBox.y += dy - dheight / 2;
  m_trackingBox.width += dwidth;
  m_trackingBox.height += dheight;
  cv::rectangle(m_frame, m_trackingBox, cv::Scalar(255, 0, 0), 2);
}

void Vision::InitializeTracker() {
  // Create a new tracker object
  m_tracker = cv::TrackerKCF::create();
  // Initialize the tracker with the tracking box and the current frame
  m_tracker->init(m_frame, m_trackingBox);
}

void Vision::UpdateTracker() {
  // Error count to refind the target if lost
  static int errorCount = 0;
  // Update the tracker with the current frame
  if(m_tracker->update(m_frame, m_trackingBox)){
    // Add line to center
    int xMid = m_frame.cols / 2;
    int yMid = m_frame.rows / 2;
    int xBoxMid = m_trackingBox.x + m_trackingBox.width / 2;
    int yBoxMid = m_trackingBox.y + m_trackingBox.height / 2;
    m_panAngle2Center = xBoxMid - xMid;
    m_tiltAngle2Center = yBoxMid - yMid;
    cv::line(m_frame, cv::Point(xMid, yMid), 
             cv::Point(xBoxMid, yBoxMid),
             cv::Scalar(0, 255, 0),1, cv::LINE_AA);
    cv::rectangle(m_frame, m_trackingBox, cv::Scalar(0, 255, 255), 2);
  } else if(errorCount > 60){
    fmt::print("Vision Tracking: Lost Target\n");
    fmt::print("Vision Tracking: Set New Target\n");
    m_trackingActive = false;
    errorCount = 0;
  } else {
    fmt::print("Vision Tracking: Error\n");
    errorCount++;
  }
}

void Vision::UpdateWindow() {
  // Display the frame
  if (!m_frame.empty()) {
    cv::imshow(m_windowName, m_frame);
    // Wait for OpenCV to display the frame
  }
  cv::waitKey(1);
}

void Vision::UpdateCamera() {
  // Capture frame-by-frame
  cv::Mat videoFrame;
  m_cap >> videoFrame;
  // Check if the frame is empty (end of video)
  if (videoFrame.empty()) {
    fmt::print(stderr, "Error: Unable to capture frame\n");
    // Fill the frame with black color
  } else {
    // Scale the frame down to get better performance
    m_frame.release();
    cv::resize(videoFrame, m_frame,
               cv::Size(videoFrame.cols * VIDEO_SCALE, videoFrame.rows * VIDEO_SCALE), 0, 0,
               cv::INTER_LINEAR);
  }
}

std::pair<float, float> Vision::GetAngles() const {
  float panAngle = m_panAngle2Center / (IPHONE_13_PRO_WIDTH * VIDEO_SCALE) *
                     IPHONE_13_PRO_FOV_H;
  float tiltAngle = m_tiltAngle2Center / (IPHONE_13_PRO_HEIGHT * VIDEO_SCALE) *
                      IPHONE_13_PRO_FOV_V;
  return std::make_pair(panAngle, tiltAngle);
}

void Vision::PutText(const std::string &text, int x, int y, double fontScale,
                     cv::Scalar color, int thickness) {
  // Put text on the frame
  cv::putText(m_frame, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX,
              fontScale, color, thickness);
}
