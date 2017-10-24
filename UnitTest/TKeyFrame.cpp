#include "gtest/gtest.h"

#include "KeyFrame.h"

#include <iostream>

TEST(KeyFrame,MakeKeyFrame_Lite)
{
    cv::Mat srcImage = cv::imread("/home/gordon/projects/SLAM/PTAM4AR/data/rgbd_dataset_freiburg1_xyz/rgb/1305031102.175304.png");
    KeyFrame kf;
    kf.MakeKeyFrame_Lite(srcImage);
    ASSERT_TRUE(0!=kf.aLevels[0].vCorners.size());
    std::cout << "kf.aLevels[0].vCorners.size(): " << kf.aLevels[0].vCorners.size() << std::endl;
    std::cout << "kf.aLevels[0].im.size: " << kf.aLevels[0].im.size << std::endl;
    std::cout << "kf.aLevels[1].im.size: " << kf.aLevels[1].im.size << std::endl;
    std::cout << "kf.aLevels[2].im.size: " << kf.aLevels[2].im.size << std::endl;
    std::cout << "kf.aLevels[3].im.size: " << kf.aLevels[3].im.size << std::endl;
}
