#include "VideoSource.h"

#include "gtest/gtest.h"

TEST(ImageDataSet,ReadImagesAssociationFile)
{
    ImageDataSet *datasetImg = new ImageDataSet("/home/gordon/projects/SLAM/PTAM4AR/data/rgbd_dataset_freiburg1_xyz",
                                                "/home/gordon/projects/SLAM/PTAM4AR/data/rgbd_dataset_freiburg1_xyz/associate.txt");
    std::vector<std::string> vstrImageFilenamesRGB;
    std::vector<std::string> vstrImageFilenamesD;
    std::vector<double> vTimestamps;
    datasetImg->ReadImagesAssociationFile(vstrImageFilenamesRGB,vstrImageFilenamesD,vTimestamps);
    if(datasetImg!=NULL)
    {
        delete datasetImg;
        datasetImg = NULL;
    }
    ASSERT_TRUE(!vstrImageFilenamesRGB.empty());
    ASSERT_EQ(vstrImageFilenamesD.size(),vstrImageFilenamesRGB.size());
}
