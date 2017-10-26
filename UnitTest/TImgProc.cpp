#include "gtest/gtest.h"

#include "ImgProc.h"

#include <iostream>

cv::Mat srcImage = cv::imread("/home/gordon/projects/SLAM/PTAM4AR/data/rgbd_dataset_freiburg1_xyz/rgb/1305031102.311267.png");

TEST(ImgProc,HalfSample)
{
    cv::Mat dstImg;
    cv::Mat imgBW;
    cv::cvtColor(srcImage,imgBW, CV_BGR2GRAY);
    ASSERT_EQ(ImgProc::HalfSample(imgBW,dstImg), 0);
    cv::imwrite("HalfSample.bmp",dstImg);
}

TEST(Feature2dDetector,DetectFASTCorners)
{
    std::vector<cv::Point2f> vecKeyPoints;
    Feature2dDetector::DetectFASTCorners(srcImage, vecKeyPoints, 10, true);
}

TEST(Feature2dDetector,FindShiTomasiScoreAtPoint)
{
    cv::Mat imgBW;
    cv::cvtColor(srcImage,imgBW, CV_BGR2GRAY);
    double score = Feature2dDetector::FindShiTomasiScoreAtPoint(imgBW, cv::Point2i(215,278), 3);
    std::cout << "score: " << score << std::endl;
}
