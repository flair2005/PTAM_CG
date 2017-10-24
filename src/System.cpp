#include "System.h"

#include <iostream>

#include "VideoSource.h"
#include "GLWindowPangolin.h"
#include "Tracker.h"

System::System()
{
    mpTracker = new Tracker();
    mbDone = false;
}

void System::Run()
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
    if(vstrImageFilenamesRGB.empty())
    {
        std::cerr << "No images found in provided path." << endl;
        return;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        std::cerr << "Different number of images for rgb and depth." << endl;
        return;
    }

    GLWindowPangolin glwindowPangolin("PTAM-GS",GLWindowPangolin::Size(640,480));

    for(unsigned int ni=0; ni<vstrImageFilenamesRGB.size() && !pangolin::ShouldQuit(); ++ni)
    {
        std::cout << vstrImageFilenamesRGB[ni] << std::endl;
        cv::Mat imgBGR = cv::imread(vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        if(imgBGR.empty())
        {
            std::cerr << "Failed to load image at: " << vstrImageFilenamesRGB[ni] << endl;
            return;
        }
        cv::Mat imgBW,imgRGB;
        cv::cvtColor(imgBGR,imgRGB,CV_BGR2RGB );
        cv::cvtColor(imgBGR,imgBW, CV_BGR2GRAY);

        glwindowPangolin.SetupViewport();
        glwindowPangolin.SetupVideoOrtho();
        glwindowPangolin.SetupVideoRasterPosAndZoom();

        glwindowPangolin.Clear();
        glwindowPangolin.DrawTexture2DRGB(imgRGB);

        Update(imgBW,imgRGB);

        glwindowPangolin.RenderTextureToViewport();
        glwindowPangolin.EndFrame();

        usleep(200000);
        if(ni == vstrImageFilenamesRGB.size()-1)
        {
            ni=0;
        }
    }
}

void System::Update(cv::Mat imgBW, cv::Mat imgRGB)
{
    mpTracker->TrackFrame(imgBW, true);
}
