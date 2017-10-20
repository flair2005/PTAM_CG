#ifndef __VideoSource_H
#define __VideoSource_H

#include <vector>
#include <string>
using namespace std;

#include <opencv2/opencv.hpp>

class VideoSource
{
public:
    unsigned int mWidth;
    unsigned int mHeight;
    VideoSource() {}
    virtual int GetFrameRGB(cv::Mat &imRGB,cv::Mat &imBW){return 0;}

};

class ImageDataSet : public VideoSource
{
public:
    ImageDataSet(const std::string &strDatasetDir,const std::string &strAssociationFilePath):
        mStrDatasetDir(strDatasetDir),
        mStrAssociationFilePath(strAssociationFilePath)
    {
    }
    void ReadImagesAssociationFile(std::vector<std::string> &vstrImageFilenamesRGB,
                                   std::vector<std::string> &vstrImageFilenamesD,
                                   std::vector<double> &vTimestamps);
    //int GetFrameRGBAndBW(cv::Mat &imRGB, cv::Mat &imBW);

private:
    std::string mStrDatasetDir;
    std::string mStrAssociationFilePath;
};

#endif
