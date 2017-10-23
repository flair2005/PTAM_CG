#include "GLWindowPangolin.h"

bool GLWindowPangolin::LoadTextures(cv::Mat matImg, GLuint texture)
{
    int Status=false;
    if(!matImg.empty())
    {
        Status=true;
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(GL_TEXTURE_2D, 0, 3,
                     matImg.cols, matImg.rows,
                     0, GL_BGR, GL_UNSIGNED_BYTE, (unsigned char *)matImg.data);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    }
    return Status;
}

void GLWindowPangolin::DisplayImage(const cv::Mat &image,
                  pangolin::GlTexture &imgTexture,
                  pangolin::OpenGlRenderState &s_cam,
                  pangolin::View &d_cam,
                  pangolin::View &d_image)
{
    d_cam.Activate(s_cam);
    //Set some random image data and upload to GPU
    imgTexture.Upload(image.data,GL_RGB,GL_UNSIGNED_BYTE);
    //display the image
    d_image.Activate();
    glColor3f(1.0,1.0,1.0);
    imgTexture.RenderToViewport(true);
    pangolin::FinishFrame();
}

void GLWindowPangolin::DrawOrigeAxis()
{
    glLineWidth(3);
    glColor3f ( 0.8f,0.f,0.f );
    glBegin ( GL_LINES );
    glVertex3f( 0,0,0 );
    glVertex3f( 10,0,0 );
    glColor3f( 0.f,0.8f,0.f);
    glVertex3f( 0,0,0 );
    glVertex3f( 0,10,0 );
    glColor3f( 0.2f,0.2f,1.f);
    glVertex3f( 0,0,0 );
    glVertex3f( 0,0,10 );
    glEnd();
}


