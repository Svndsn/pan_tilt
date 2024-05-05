#include "auto_tracking.h"
#include "manuel_tracking.h"
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

void Auto() {

  cv::VideoCapture video(1);
  cv::Mat frame;
  auto_tracking track;

  while (video.read(frame)) {
    frame.resize(64);
    std::vector<cv::Rect> found = track.detect(frame);
    for (std::vector<cv::Rect>::iterator i = found.begin(); i != found.end();
         ++i) {
      cv::Rect &r = *i;
      track.ResizeBoxes(r);
      cv::rectangle(frame, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
    }
    cv::imshow("Auto tracking", frame);
    if (cv::waitKey(1) >= 0) {
      break;
    }
  }
  video.release();
  cv::destroyAllWindows();
}

void manuel() {
  cv::VideoCapture video(1);

  cv::Mat frame;
  video.read(frame);
  manuel_tracking track(cv::TrackerCSRT::create());
  track.settrackingbox(frame, false);
  track.gettracker()->init(frame, track.gettrackingbox());
  cv::Rect trackingbox = track.gettrackingbox();

  while (video.read(frame)) {

    if (track.gettracker()->update(frame, trackingbox)) {
      cv::rectangle(frame, trackingbox, cv::Scalar(0, 0, 255), 2, 8);
    }
    cv::imshow("Video feed", frame);
    if (cv::waitKey(1) >= 0) {
      break;
    }
  }

  video.release();
  cv::destroyAllWindows();
}

int main() {
  int value;
  std::cin >> value;

  switch (value) {
  case 1:
    manuel();
    break;
  case 2:
    Auto();
    break;
  default:
    break;
  }
}
