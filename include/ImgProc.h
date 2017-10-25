#pragma once
#ifndef __ImgProc_H
#define __ImgProc_H

#include "opencv2/opencv.hpp"

class ImgProc
{
public:
    ImgProc() {}
    ~ImgProc(){}

    static int DetectFASTCorners10(const cv::Mat &srcImage, std::vector<cv::Point2f> &vecKeyPoints, const int &threshold=10);
    static double FindShiTomasiScoreAtPoint(const cv::Mat &image, cv::Point2i ptCenter, unsigned int nHalfBoxSize=3);
    static bool IsInImageWithBorder(const cv::Mat &image, const cv::Point2f &pt, int border=10);
};
#endif
