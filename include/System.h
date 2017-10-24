#pragma once

#include <opencv2/opencv.hpp>

#include "Common.h"

class Tracker;

class System
{
public:
    System();
    void Run();
    void Update(cv::Mat imgBW, cv::Mat imgRGB);

private:
    Tracker *mpTracker;

    bool mbDone;
};
