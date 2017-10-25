#include "gtest/gtest.h"

#include "ImgProc.h"

#include <iostream>

TEST(ImgProc,DetectFASTCorners10)
{
    cv::Mat srcImage = cv::imread("/home/gordon/Pictures/47.jpg");
    std::vector<cv::Point2f> vecKeyPoints;
    ImgProc::DetectFASTCorners(srcImage, vecKeyPoints, 10, true);
}

TEST(ImgProc,FindShiTomasiScoreAtPoint)
{
    cv::Mat srcImage = cv::imread("/home/gordon/projects/SLAM/PTAM4AR/data/rgbd_dataset_freiburg1_xyz/rgb/1305031102.311267.png");
    cv::Mat imgBW;
    cv::cvtColor(srcImage,imgBW, CV_BGR2GRAY);
    double score = ImgProc::FindShiTomasiScoreAtPoint(imgBW, cv::Point2i(215,278), 3);
    std::cout << "score: " << score << std::endl;
}
