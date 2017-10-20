#include "VideoSource.h"
#include <fstream>

void ImageDataSet::ReadImagesAssociationFile(std::vector<std::string> &vstrImageFilenamesRGB,
                                             std::vector<std::string> &vstrImageFilenamesD,
                                             std::vector<double> &vTimestamps)
{
    std::ifstream fAssociation;
    fAssociation.open(mStrAssociationFilePath.c_str());
    while(!fAssociation.eof())
    {
        std::string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            std::string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(mStrDatasetDir+"/"+sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(mStrDatasetDir+"/"+sD);
        }
    }
}
