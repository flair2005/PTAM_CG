#include <iostream>
using namespace std;

#include "VideoSource.h"
#include "GLWindowPangolin.h"
#include "System.h"

int main()
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
        cerr << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    GLWindowPangolin glwindowPangolin("PTAM-GS",640,480);
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(640,480,420,420,320,240,-1.0,1.0),pangolin::ModelViewLookAt(-1,1,-1, 0,0,0, pangolin::AxisY));
    // Aspect ratio allows us to constrain width and height whilst fitting within specified
    // bounds. A positive aspect ratio makes a view 'shrink to fit' (introducing empty bars),
    // whilst a negative ratio makes the view 'grow to fit' (cropping the view).
    pangolin::View &d_cam = pangolin::Display("cam").SetBounds(0,1.0f,0,1.0f,-640/480.0).SetHandler(new pangolin::Handler3D(s_cam));
    // This view will take up no more than a third of the windows width or height, and it
    // will have a fixed aspect ratio to match the image that it will display. When fitting
    // within the specified bounds, push to the top-left (as specified by SetLock).
    pangolin::View &d_image = pangolin::Display("image").SetBounds(0.0f,1.0f,0,1.0f,640.0/480).SetLock(pangolin::LockLeft, pangolin::LockTop);
    pangolin::GlTexture imgTexture(640,480,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

    for(int ni=0; ni<vstrImageFilenamesRGB.size() && !pangolin::ShouldQuit(); ++ni)
    {
        std::cout << vstrImageFilenamesRGB[ni] << std::endl;
        cv::Mat imBGR = cv::imread(vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        if(imBGR.empty())
        {
            cerr << "Failed to load image at: " << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        cv::Mat imRGB;
        cvtColor(imBGR,imRGB,CV_BGR2RGB);

        glwindowPangolin.Clear();
        glwindowPangolin.DisplayImage(imRGB,imgTexture,s_cam,d_cam,d_image);

        usleep(50000);
        if(ni == vstrImageFilenamesRGB.size()-1)
        {
            ni=0;
        }
    }

    return 0;
}


