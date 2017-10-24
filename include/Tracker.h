#pragma once

#include "KeyFrame.h"

class Tracker
{
public:
    inline Tracker()
    {
        mCurrentKF.bFixed = false;
    }
    ~Tracker(){}
    void TrackFrame(cv::Mat imgBW, bool bDraw);

protected:
    KeyFrame mCurrentKF;
};
