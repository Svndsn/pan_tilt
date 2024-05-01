#ifndef AUTO_TRACKING_H
#define AUTO_TRACKING_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>

class auto_tracking
{
private:
    cv::HOGDescriptor _hog;
public:
    auto_tracking();
    std::vector<cv::Rect>detect(cv::InputArray img);
    void ResizeBoxes(cv::Rect& box);
};

auto_tracking::auto_tracking(): _hog()
{
    _hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
}
std::vector<cv::Rect> auto_tracking::detect(cv::InputArray img){
    std::vector<cv::Rect> found;
    _hog.detectMultiScale(img,found, 0 , cv::Size(8,8),cv::Size(),1.05,2,false);
    return found;
}
void auto_tracking::ResizeBoxes(cv::Rect& box){
    box.x += cvRound(box.width*0.1);
    box.width = cvRound(box.width*0.8);
    box.y += cvRound(box.height*0.07);
    box.height = cvRound(box.height*0.8);
}



#endif //AUTO_TRACKING_H