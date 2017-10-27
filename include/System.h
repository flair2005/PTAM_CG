#pragma once
#ifndef __System_H
#define __System_H

#include <opencv2/opencv.hpp>



class GLWindowPangolin;
class VideoSource;
class Tracker;

class System
{
public:
    GLWindowPangolin *mpPangolinWindow;
    VideoSource *mpVideoSource;
    System();
    ~System();
    void Run();
    void Update(cv::Mat imgBW, cv::Mat imgRGB);

private:
    Tracker *mpTracker;

    bool mbDone;
};
#endif
