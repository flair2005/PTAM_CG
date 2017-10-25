#include "ImgProc.h"

int MiniPatch::mnHalfPatchSize = 4;
int MiniPatch::mnRange = 10;
int MiniPatch::mnMaxSSD = 9999;

int MiniPatch::SampleFromImage(cv::Mat &img, const cv::Point2f &ptPos)
{
    if(!ImgProc::IsInImageWithBorder(img,ptPos,mnHalfPatchSize))
        return 1;

    cv::Rect rectROI;
    rectROI.width = 2*mnHalfPatchSize+1;
    rectROI.height = 2*mnHalfPatchSize+1;
    rectROI.x = ptPos.x-rectROI.width/2.f;
    rectROI.y = ptPos.y-rectROI.height/2.f;
    mimOrigPatch = img(rectROI);

    return 0;
}

int MiniPatch::SSDAtPoint(cv::Mat &img, const cv::Point2i &pt)
{
    if(!ImgProc::IsInImageWithBorder(img,pt,mnHalfPatchSize))
        return mnMaxSSD + 1;

    cv::Point2i ptImgBase(pt.x-mnHalfPatchSize, pt.y-mnHalfPatchSize);

    int nSumSqDiff = 0;
    unsigned int nRows = mimOrigPatch.rows;
    unsigned int nCols = mimOrigPatch.cols;
    for(unsigned int h=0; h<nRows; ++h)
    {
        unsigned char *pImage    = img.ptr<uchar>(ptImgBase.y+h);
        unsigned char *pTemplate = mimOrigPatch.ptr<uchar>(h);
        for(unsigned int w=0; w<nCols; ++w)
        {
            int nDiff = pImage[ptImgBase.x+w]-pTemplate[w];
            nSumSqDiff += nDiff * nDiff;
        }
    }

    return nSumSqDiff;
}

// Find a patch by searching at FAST corners in an input image
// If available, a row-corner LUT is used to speed up search through the
// FAST corners
bool MiniPatch::FindPatch(cv::Point2f &ptPos,
                          cv::Mat &img,
                          int nRange,
                          std::vector<cv::Point2f> &vCorners,
                          std::vector<int> *pvRowLUT)
{   
    cv::Point2f ptBBoxTL = ptPos - cv::Point2f(nRange, nRange);
    cv::Point2f ptBBoxBR = ptPos + cv::Point2f(nRange, nRange);
    std::vector<cv::Point2f>::iterator i;
    if(NULL == pvRowLUT)
    {
        for(i = vCorners.begin(); i!=vCorners.end(); i++)
        {
            if(i->y >= ptBBoxTL.y)
                break;
        }
    }
    else
    {
        int nTopRow = ptBBoxTL.y;
        if(nTopRow < 0)
            nTopRow = 0;
        if(nTopRow >= (int)pvRowLUT->size())
            nTopRow = (int)pvRowLUT->size()-1;
        i = vCorners.begin()+(*pvRowLUT)[nTopRow];
    }

    cv::Point2f ptBest;
    int nBestSSD = mnMaxSSD + 1;
    for(; i!=vCorners.end(); i++)
    {
        if(i->x < ptBBoxTL.x  || i->x > ptBBoxBR.x)
            continue;
        if(i->y > ptBBoxBR.y)
            break;

        int nSSD = SSDAtPoint(img, *i);
        if(nSSD < nBestSSD)
        {
            ptBest = *i;
            nBestSSD = nSSD;
        }
    }
    if(nBestSSD < mnMaxSSD)
    {
        ptPos = ptBest;
        return true;
    }
    else
        return false;
}












