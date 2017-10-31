#include "MapMaker.h"

#include "Common.h"
#include "Homography.h"

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
    mCamera.SetImageSize(cv::Point2f(640,480));

    std::vector<Homography::HomographyMatch> vMatches;
    for(unsigned int i=0; i<vTrailMatches.size(); i++)
    {
        cv::Point2f pt2CamPlaneFirst = mCamera.UnProject(vTrailMatches[i].first);
        cv::Point2f pt2CamPlaneSecond = mCamera.UnProject(vTrailMatches[i].second);
        Eigen::Matrix2f m2PixelProjectionJac = mCamera.GetProjectionDerivs();

        Homography::HomographyMatch match;
        match.v2CamPlaneFirst[0]  = pt2CamPlaneFirst.x;
        match.v2CamPlaneFirst[1]  = pt2CamPlaneFirst.y;
        match.v2CamPlaneSecond[0] = pt2CamPlaneSecond.x;
        match.v2CamPlaneSecond[1] = pt2CamPlaneSecond.y;
        match.m2PixelProjectionJac = m2PixelProjectionJac;
        vMatches.push_back(match);
    }

    Homography homo;
    Eigen::Matrix3f m3Homography;
    if(GS::RET_SUCESS == homo.HomographyFromMatches(vMatches, m3Homography))
    {
    }

    return false;
}
