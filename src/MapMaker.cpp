#include "MapMaker.h"

#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>

MapMaker::MapMaker(const ATANCamera &cam):
    mCamera(cam)
{
}

bool MapMaker::InitFromStereo(
        KeyFrame &kFirst,
        KeyFrame &kSecond,
        std::vector<std::pair<cv::Point2i, cv::Point2i> > &vTrailMatches,
        Sophus::SE3 &se3CameraPos)
{
    std::vector<std::pair<cv::Point2i, cv::Point2i> >().swap(vTrailMatches);
    vTrailMatches.clear();

    vTrailMatches.push_back(std::pair<cv::Point2i, cv::Point2i>(cv::Point2i(491.913, 94.5141),cv::Point2i(446.78, 100.857)));
    vTrailMatches.push_back(std::pair<cv::Point2i, cv::Point2i>(cv::Point2i(205.123, 102.715),cv::Point2i(159.491, 109.362)));
    vTrailMatches.push_back(std::pair<cv::Point2i, cv::Point2i>(cv::Point2i(201.812, 309.027),cv::Point2i(157.146, 319.571)));
    vTrailMatches.push_back(std::pair<cv::Point2i, cv::Point2i>(cv::Point2i(506.192, 304.866),cv::Point2i(462.334, 308.977)));

    std::vector<cv::Point2f> srcPoints;
    std::vector<cv::Point2f> dstPoints;
    for(unsigned int i=0; i<vTrailMatches.size(); i++)
    {
        cv::Point2f v2CamPlaneFirst = mCamera.Project(vTrailMatches[i].first);
        cv::Point2f v2CamPlaneSecond = mCamera.Project(vTrailMatches[i].second);
        //Eigen::Matrix2f m2PixelProjectionJac = mCamera.GetProjectionDerivs();

        srcPoints.push_back(v2CamPlaneFirst);
        dstPoints.push_back(v2CamPlaneSecond);
    }

    cv::Mat mat = cv::findHomography(srcPoints,dstPoints,CV_RANSAC,1);

    std::cout << "findHomography: " << mat << std::endl;

    exit(1);
    return false;
}
