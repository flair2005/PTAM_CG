#pragma once
#ifndef __GLWindowPangolin_H
#define __GLWindowPangolin_H

#include <string>
#include <vector>

#include <GL/glew.h>
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

#include "Common.h"

class GLWindowPangolin
{
public:
    GLWindowPangolin(){}
    GLWindowPangolin(const std::string title, GS::Size sizeWindow);

    void SetupViewport();
    void SetupVideoOrtho();
    void SetupVideoRasterPosAndZoom();

    bool LoadTextures(cv::Mat matImg, GLuint texture);
    void DrawTexture2DGray(const cv::Mat &imgGray);
    void DrawTexture2DRGB(const cv::Mat &imgRGB);
    void RenderTextureToViewport();

    void DrawPoints2f(const std::vector<cv::Point2f> &points, GS::RGB rgb, float size);
    void DrawLines(const cv::Point2f &ptStart, GS::RGB rgbStart,
                   const cv::Point2f &ptEnd  , GS::RGB rgbEnd,
                   float width);
    void DrawOrigeAxis();

    inline void Clear(){ glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); }
    inline void EndFrame(){ pangolin::FinishFrame(); }

private:
    GS::Size mSizeWin;
    GS::Size mSizeVideo;
    GS::RGB mRGB;
    pangolin::GlTexture mTexture;
};
#endif
