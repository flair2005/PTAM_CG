#pragma once
#ifndef __ImgProc_H
#define __ImgProc_H

#include "opencv2/opencv.hpp"

class ImgProc
{
public:
    ImgProc() {}
    ~ImgProc(){}

    static int DetectFASTCorners(const cv::Mat &srcImage, std::vector<cv::Point2f> &vecKeyPoints, const int &threshold, bool nonmaxSuppression);
    static double FindShiTomasiScoreAtPoint(const cv::Mat &image, cv::Point2i ptCenter, unsigned int nHalfBoxSize=3);
    static bool IsInImageWithBorder(const cv::Mat &image, const cv::Point2f &pt, int border=10);
};

// This is a simple pixel-patch class, used for tracking small patches
// it's used by the tracker for building the initial map
class MiniPatch : public ImgProc
{
public:
    cv::Mat mimOrigPatch;           // Original pixels
    static int mnHalfPatchSize;     // How big is the patch?
    static int mnRange;             // How far to search?
    static int mnMaxSSD;            // Max SSD for matches?

    int SampleFromImage(cv::Mat &img, const cv::Point2f &ptPos);
    int SSDAtPoint(cv::Mat &img, const cv::Point2i &pt);
    bool FindPatch(cv::Point2f &ptPos,cv::Mat &img,int nRange,
                   std::vector<cv::Point2f> &vCorners,
                   std::vector<int> *pvRowLUT = NULL);
};

#endif
