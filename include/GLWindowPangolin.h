#pragma once

#include <string>
#include <vector>

#include <GL/glew.h>
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

class GLWindowPangolin
{
public:
    struct RGB
    {
        float r;
        float g;
        float b;
        RGB(){}
        RGB(float red, float green, float blue):r(red),g(green),b(blue){}
    };

    struct Size
    {
        unsigned int width;
        unsigned int height;
        Size(){}
        Size(unsigned int w,unsigned int h):width(w),height(h){}
        Size(const Size &size):width(size.width),height(size.height){}
    };

public:
    GLWindowPangolin(){}
    GLWindowPangolin(const std::string title, Size sizeWindow);

    void SetupViewport();
    void SetupVideoOrtho();
    void SetupVideoRasterPosAndZoom();

    bool LoadTextures(cv::Mat matImg, GLuint texture);
    void DrawTexture2DRGB(const cv::Mat &imgRGB);
    void RenderTextureToViewport();

    void DrawPoints2f(const std::vector<cv::Point2f> &points, RGB rgb, float size);
    void DrawOrigeAxis();

    inline void Clear(){ glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); }
    inline void EndFrame(){ pangolin::FinishFrame(); }

private:
    Size mSizeWin;
    Size mSizeVideo;
    RGB mRGB;
    pangolin::GlTexture mTexture;
};
