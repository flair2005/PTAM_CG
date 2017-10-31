#ifndef __Homography_H
#define __Homography_H

#include <Eigen/Core>

class Homography
{
public:
    // Homography matches are 2D-2D matches in a stereo pair, unprojected
    // to the Z=1 plane.
    struct HomographyMatch
    {
        // To be filled in by MapMaker:
        Eigen::Vector2f v2CamPlaneFirst;
        Eigen::Vector2f v2CamPlaneSecond;
        Eigen::Matrix2f m2PixelProjectionJac;
    };
public:
    Homography() {}
    ~Homography() {}

public:
    int HomographyFromMatches(std::vector<HomographyMatch> vMatches, Eigen::Matrix3f &m3Homography);
};

#endif
