// This is a simple pixel-patch class, used for tracking small patches
// it's used by the tracker for building the initial map
#pragma once

#include <vector>

#include "ImgProc.h"

struct MiniPatch
{
    int SampleFromImage(cv::Mat &img, const cv::Point2f &ptPos);
    int SSDAtPoint(cv::Mat &img, const cv::Point2i &pt);
    bool FindPatch(cv::Point2f &ptPos,cv::Mat &img,int nRange,
                   std::vector<cv::Point2f> &vCorners,
                   std::vector<int> *pvRowLUT = NULL);

    cv::Mat mimOrigPatch;           // Original pixels
    static int mnHalfPatchSize;     // How big is the patch?
    static int mnRange;             // How far to search?
    static int mnMaxSSD;            // Max SSD for matches?    
};
