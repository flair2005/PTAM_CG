#include "gtest/gtest.h"

#include "ImgProc.h"

TEST(ImgProc,DetectFASTCorners10)
{
    cv::Mat srcImage = cv::imread("/home/gordon/Pictures/47.jpg");
    std::vector<cv::Point2f> vecKeyPoints;
    ImgProc::DetectFASTCorners10(srcImage, vecKeyPoints, 10);
}
