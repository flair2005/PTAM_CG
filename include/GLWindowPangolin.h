#ifndef __GLWindowPangolin_H
#define __GLWindowPangolin_H

#include <string>

#include <GL/glew.h>

#include <pangolin/pangolin.h>

#include <opencv2/opencv.hpp>

class GLWindowPangolin
{
public:
    unsigned int mWidth;
    unsigned int mHeight;

    GLWindowPangolin(const std::string title, unsigned int width, unsigned int height):
        mWidth(width),
        mHeight(height)
    {
        //Create OpenGL window in single line
        pangolin::CreateWindowAndBind(title,width,height);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);
    }

    bool LoadTextures(cv::Mat matImg, GLuint texture);

    void DisplayImage(const cv::Mat &image,
                      pangolin::GlTexture &imgTexture,
                      pangolin::OpenGlRenderState &s_cam,
                      pangolin::View &d_cam,
                      pangolin::View &d_image);

    void DrawOrigeAxis();

    void Clear()
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
};

#endif
