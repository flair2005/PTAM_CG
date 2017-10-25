#pragma once
#ifndef __Tracker_H
#define __Tracker_H

#include "MiniPatch.h"
#include "KeyFrame.h"

struct Trail    // This struct is used for initial correspondences of the first stereo pair.
{
    MiniPatch mPatch;
    cv::Point2i ptCurrentPos;
    cv::Point2i ptInitialPos;
};

class Tracker
{
public:
    Tracker();
    ~Tracker(){}

    void TrackFrame(cv::Mat imgBW, bool bDraw);
    void TrackForInitialMap();
    void TrailTracking_Start();
    int TrailTracking_Advance();

protected:
    bool mbDraw;
    int mnFrame;
    KeyFrame mCurrentKF;
    KeyFrame mFirstKF;
    KeyFrame mPreviousFrameKF;
    std::list<Trail> mlTrails;
    enum{TRAIL_TRACKING_NOT_STARTED,TRAIL_TRACKING_STARTED,TRAIL_TRACKING_COMPLETE} mnInitialStage;

    void Reset();
};
#endif
