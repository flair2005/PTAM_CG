#include "Common.h"
#include "System.h"
#include "GLWindowPangolin.h"
#include "VideoSource.h"
#include "Tracker.h"

#include <unistd.h>
#include <iostream>

System::System()
{
    mpPangolinWindow = new GLWindowPangolin("PTAM-GS",GS::Size(640,480));
    mpVideoSource = new ImageDataSet("/home/gordon/projects/PTAM4AR/data/rgbd_dataset_freiburg1_xyz",
                                       "/home/gordon/projects/PTAM4AR/data/rgbd_dataset_freiburg1_xyz/associate.txt");
    mpTracker = new Tracker(mpPangolinWindow);
    mbDone = false;
}

void System::Run()
{
    cv::Mat imgRGB,imgBW;
    while(!mbDone)
    {
        if(GS::RET_FAILED == mpVideoSource->GetFrameRGBBW(imgRGB,imgBW))
            break;

        mpPangolinWindow->SetupViewport();
        mpPangolinWindow->SetupVideoOrtho();
        mpPangolinWindow->SetupVideoRasterPosAndZoom();

        mpPangolinWindow->Clear();
        mpPangolinWindow->DrawTexture2DGray(imgBW);

        Update(imgBW,imgRGB);

        mpPangolinWindow->RenderTextureToViewport();
        mpPangolinWindow->EndFrame();

        usleep(40000);
    }
}

void System::Update(cv::Mat imgBW, cv::Mat imgRGB)
{
    mpTracker->TrackFrame(imgBW, true);
}

System::~System()
{
    DELETE_NEW_OBJ(mpTracker)
    DELETE_NEW_OBJ(mpVideoSource)
    DELETE_NEW_OBJ(mpPangolinWindow)
}
