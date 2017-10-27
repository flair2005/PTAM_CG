#pragma once
#ifndef __ImgProc_H
#define __ImgProc_H

#include "opencv2/opencv.hpp"

class ImgProc
{
public:
    static bool IsInImageWithBorder(const cv::Mat &image, const cv::Point2f &pt, int border=10);
    static int HalfSample(const cv::Mat &imgSrc, cv::Mat &imgDst);
};

class Feature2dDetector : public ImgProc
{
public:
    static int DetectFASTCorners(const cv::Mat &srcImage, std::vector<cv::Point2f> &vecKeyPoints, const int &threshold, bool nonmaxSuppression);
    static int FindShiTomasiScoreAtPoint(const cv::Mat &image, cv::Point2i ptCenter, double &dScore, unsigned int nHalfBoxSize=3);
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

    int SampleFromImage(const cv::Mat &img, const cv::Point2f &ptPos);
    int SSDAtPoint(const cv::Mat &img, const cv::Point2i &pt, int &nSSD);
    bool FindPatch(cv::Point2f &ptPos,cv::Mat &img,int nRange,
                   std::vector<cv::Point2f> &vCorners,
                   std::vector<int> *pvRowLUT = NULL);
};

#endif
