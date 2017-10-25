#pragma once
#ifndef __VideoSource_H
#define __VideoSource_H

#include <vector>
#include <string>
#include <sstream>
using namespace std;

class VideoSource
{
public:
    unsigned int mWidth;
    unsigned int mHeight;
    VideoSource(){}
    //virtual ~VideoSource(){}
    //virtual int GetFrameRGB(cv::Mat &imRGB,cv::Mat &imBW){return 0;}

};

class ImageDataSet : public VideoSource
{
public:
    ImageDataSet(const std::string &strDatasetDir,const std::string &strAssociationFilePath):
        mStrDatasetDir(strDatasetDir),
        mStrAssociationFilePath(strAssociationFilePath)
    {
    }
    ~ImageDataSet(){}
    void ReadImagesAssociationFile(std::vector<std::string> &vstrImageFilenamesRGB,
                                   std::vector<std::string> &vstrImageFilenamesD,
                                   std::vector<double> &vTimestamps);

private:
    std::string mStrDatasetDir;
    std::string mStrAssociationFilePath;
};
#endif
