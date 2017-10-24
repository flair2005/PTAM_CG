#include "KeyFrame.h"

#include "ImgProc.h"

void KeyFrame::MakeKeyFrame_Lite(cv::Mat &img)
{
    aLevels[0].im = img.clone();
    for(unsigned int i=0;i<LEVELS;++i)
    {
        Level &lev = aLevels[i];
        if(0 != i)
        {
            cv::Size sizeImg(aLevels[i-1].im.cols/2, aLevels[i-1].im.rows/2);
            cv::pyrDown(aLevels[i-1].im,lev.im,sizeImg);//Gaussian pyramid, default kernel 5*5
        }
        lev.vCorners.clear();
        lev.vCandidates.clear();
        lev.vMaxCorners.clear();
        if(i == 0)
        {
            ImgProc::DetectFASTCorners10(lev.im, lev.vCorners, 10);
            lev.vMaxCorners.resize(lev.vCorners.size());
            lev.vMaxCorners.assign(lev.vCorners.begin(),lev.vCorners.end());
        }
        if(i == 1)
        {
            ImgProc::DetectFASTCorners10(lev.im, lev.vCorners, 15);
            lev.vMaxCorners.resize(lev.vCorners.size());
            lev.vMaxCorners.assign(lev.vCorners.begin(),lev.vCorners.end());
        }
        if(i == 2)
        {
            ImgProc::DetectFASTCorners10(lev.im, lev.vCorners, 15);
            lev.vMaxCorners.resize(lev.vCorners.size());
            lev.vMaxCorners.assign(lev.vCorners.begin(),lev.vCorners.end());
        }
        if(i == 3)
        {
            ImgProc::DetectFASTCorners10(lev.im, lev.vCorners, 10);
            lev.vMaxCorners.resize(lev.vCorners.size());
            lev.vMaxCorners.assign(lev.vCorners.begin(),lev.vCorners.end());
        }
        //generate row LUT
        unsigned int v=0;
        lev.vCornerRowLUT.clear();
        for(int y=0; y<lev.im.rows; y++)
        {
            while(v < lev.vCorners.size() && y > lev.vCorners[v].y)
                v++;
            lev.vCornerRowLUT.push_back(v);
        }
    }
}

void KeyFrame::MakeKeyFrame_Rest()
{
    for(unsigned int l=0; l<LEVELS; ++l)
    {
        Level &lev = aLevels[l];
        for(std::vector<cv::Point2f>::iterator i=lev.vMaxCorners.begin(); i!=lev.vMaxCorners.end(); i++)
        {
            if(!ImgProc::IsInImageWithBorder(lev.im,*i, 10))
                continue;
            double dSTScore = ImgProc::FindShiTomasiScoreAtPoint(lev.im,*i);
            if(dSTScore > 70)
            {
                Candidate c;
                c.ptLevelPos = *i;
                c.dSTScore = dSTScore;
                lev.vCandidates.push_back(c);
            }
        }
    }
}
