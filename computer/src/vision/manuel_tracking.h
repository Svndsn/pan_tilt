#ifndef MANUEL_TRACKING_H
#define MANUEL_TRACKING_H

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

class manuel_tracking {
public:
  manuel_tracking(cv::Ptr<cv::Tracker> tracker);

  void settracker(cv::Ptr<cv::Tracker> t);

  cv::Ptr<cv::Tracker> gettracker();

  void settrackingbox(cv::Mat frame, bool f);

  cv::Rect gettrackingbox();

private:
  cv::Ptr<cv::Tracker> _tracker;
  cv::Rect _trackingBox;
};

void manuel_tracking::settracker(cv::Ptr<cv::Tracker> t) { _tracker = t; }

cv::Ptr<cv::Tracker> manuel_tracking::gettracker() { return _tracker; }

void manuel_tracking::settrackingbox(cv::Mat frame, bool f) {
  _trackingBox = cv::selectROI(frame, f);
}

cv::Rect manuel_tracking::gettrackingbox() { return _trackingBox; }

manuel_tracking::manuel_tracking(cv::Ptr<cv::Tracker> tracker)
    : _tracker(tracker) {}

#endif // MANUEL_TRACKING_H
