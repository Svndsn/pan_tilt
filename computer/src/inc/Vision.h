#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <string>


#define IPHONE_13_PRO_FOV_H 69.39
#define IPHONE_13_PRO_FOV_V 40.82
#define IPHONE_13_PRO_WIDTH 1920
#define IPHONE_13_PRO_HEIGHT 1080
// #define 
#define VIDEO_SCALE 0.5

class Vision {
public:
  // Constructor
  Vision(std::string windowName, int cameraIndex);

  // Destructor
  ~Vision();

  // Get the state of the tracking
  bool IsTrackingActive() const;

  // Set the current state of the tracking
  void SetTrackingActive(bool active);

  // Set the tracking box
  void SetTrackingBox(int x, int y, int width, int height);

  // Update the tracking box
  void UpdateTrackingBox(int dx, int dy, int dwidth, int dheight);

  // Initialize the tracker
  void InitializeTracker();

  // Update the tracker
  void UpdateTracker();

  // Update the window
  void UpdateWindow();

  // Update the camera
  void UpdateCamera();

  // Get the angles to make the object in the center of the frame
  // Returns a pair of pan and tilt angles
  std::pair<float, float> GetAngles() const;

  // Put text on the frame
  void PutText(const std::string &text, int x, int y, double fontScale,
               cv::Scalar color, int thickness);

private:
  std::string m_windowName;
  cv::VideoCapture m_cap;
  cv::Mat m_frame;
  cv::Rect m_trackingBox;
  cv::Ptr<cv::Tracker> m_tracker;

  bool m_trackingActive;
  int16_t m_panAngle2Center;
  int16_t m_tiltAngle2Center;
};
