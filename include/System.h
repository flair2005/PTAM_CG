#pragma once
#ifndef __System_H
#define __System_H

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
#endif
