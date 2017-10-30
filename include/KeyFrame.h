#pragma once
#ifndef __KeyFrame_H
#define __KeyFrame_H

#include <vector>
#include <map>
#include <opencv2/opencv.hpp>

const unsigned int LEVELS = 4;

struct Candidate
{
    cv::Point2i ptLevelPos;
    cv::Point2f ptRootPos;
    double dSTScore;
};

struct Measurement
{
    int nLevel;   // Which image level?
    bool bSubPix; // Has this measurement been refined to sub-pixel level?
    cv::Point2i pt2fRootPos;  // Position of the measurement, REFERED TO PYRAMID LEVEL ZERO
    enum {SRC_TRACKER, SRC_REFIND, SRC_ROOT, SRC_TRAIL, SRC_EPIPOLAR} Source; // Where has this measurement come frome?
};

struct Level
{
    inline Level()
    {
        bImplaneCornersCached = false;
    }

    cv::Mat im;                // The pyramid level pixels
    std::vector<cv::Point2i> vCorners;     // All FAST corners on this level
    std::vector<unsigned int> vCornerRowLUT;          // Row-index into the FAST corners, speeds up access
    std::vector<cv::Point2i> vMaxCorners;  // The maximal FAST corners
    // The keyframe struct is quite happy with default operator=, but Level needs its own
    // to override CVD's reference-counting behaviour.
//    Level& operator=(const Level &rhs);
//    {
//        // Operator= should physically copy pixels, not use CVD's reference-counting image copy.
//        im.resize(rhs.im.size());
//        copy(rhs.im, im);

//        vCorners = rhs.vCorners;
//        vMaxCorners = rhs.vMaxCorners;
//        vCornerRowLUT = rhs.vCornerRowLUT;
//        return *this;
//    }

    std::vector<Candidate> vCandidates;   // Potential locations of new map points

    bool bImplaneCornersCached;           // Also keep image-plane (z=1) positions of FAST corners to speed up epipolar search
    std::vector<cv::Point2f> vImplaneCorners; // Corner points un-projected into z=1-plane coordinates
};

struct KeyFrame
{
    inline KeyFrame()
    {
    }

    bool bFixed; //(only true for first KF!)
    Level aLevels[LEVELS];

    int MakeKeyFrame_Lite(const cv::Mat &img);
    int MakeKeyFrame_Rest();
};

#endif
