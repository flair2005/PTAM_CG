#include <gtest/gtest.h>
#include <iostream>

#include "Common.h"
#include "ImgProc.h"

cv::Mat srcImage = cv::imread("/home/gordon/projects/PTAM4AR/data/rgbd_dataset_freiburg1_xyz/rgb/1305031102.311267.png");

TEST(ImgProc,HalfSample)
{
    cv::Mat dstImg;
    cv::Mat imgBW;
    cv::cvtColor(srcImage,imgBW, CV_BGR2GRAY);
    ASSERT_EQ(ImgProc::HalfSample(imgBW,dstImg), GS::RET_SUCESS);
    cv::imwrite("HalfSample.bmp",dstImg);
    ASSERT_EQ(imgBW.cols/2, dstImg.cols);
    ASSERT_EQ(imgBW.rows/2, dstImg.rows);
    const unsigned char *srcT = imgBW.data;
    const unsigned char *srcB = imgBW.data+imgBW.cols;
    const unsigned char *dstT = dstImg.data;
    ASSERT_EQ((srcT[0]+srcT[1]+srcB[0]+srcB[1])/4, dstT[0]);
}

TEST(Feature2dDetector,DetectFASTCorners)
{
    std::vector<cv::Point2f> vecKeyPoints;
    int ret = Feature2dDetector::DetectFASTCorners(srcImage, vecKeyPoints, 10, true);
    ASSERT_EQ(ret, GS::RET_SUCESS);
}

TEST(Feature2dDetector,FindShiTomasiScoreAtPoint)
{
    cv::Mat imgBW;
    cv::cvtColor(srcImage,imgBW, CV_BGR2GRAY);
    double score = 0;
    int ret = Feature2dDetector::FindShiTomasiScoreAtPoint(imgBW, cv::Point2i(215,278), score, 3);
    ASSERT_EQ(ret, GS::RET_SUCESS);
    std::cout << "score: " << score << std::endl;
}
