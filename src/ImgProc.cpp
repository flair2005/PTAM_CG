#include "Common.h"
#include "ImgProc.h"

#include <vector>

bool ImgProc::IsInImageWithBorder(const cv::Mat &image, const cv::Point2i &pt, int border)
{
    return pt.x >=border && pt.y >=border && pt.x < image.cols - border && pt.y < image.rows - border;
}

//Mean Pyramid
int ImgProc::HalfSample(const cv::Mat &imgSrc, cv::Mat &imgDst)
{
    if(1 != imgSrc.channels())
        return GS::RET_FAILED;
    imgDst.create(imgSrc.rows/2,imgSrc.cols/2,CV_8UC1);
    const unsigned char *top = imgSrc.data;
    const unsigned char *bottom = top + imgSrc.cols;
    const unsigned char *end = top + imgSrc.step*imgSrc.rows;
    int ow = imgDst.cols;
    int skip = imgSrc.cols + (imgSrc.cols % 2);
    unsigned char *p = imgDst.data;
    while (bottom < end)
    {
        for (int j=0; j<ow; j++)
        {
            *p = static_cast<unsigned char>((top[0] + top[1] + bottom[0] + bottom[1])/4);
            p++;
            top += 2;
            bottom += 2;
        }
        top += skip;
        bottom += skip;
    }
    return GS::RET_SUCESS;
}
