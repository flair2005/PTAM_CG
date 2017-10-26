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
            ImgProc::HalfSample(aLevels[i-1].im,lev.im);
        }
        lev.vCorners.clear();
        lev.vCandidates.clear();
        lev.vMaxCorners.clear();
        if(i == 0)
        {
            Feature2dDetector::DetectFASTCorners(lev.im, lev.vCorners, 10, false);
        }
        if(i == 1)
        {
            Feature2dDetector::DetectFASTCorners(lev.im, lev.vCorners, 15, false);
        }
        if(i == 2)
        {
            Feature2dDetector::DetectFASTCorners(lev.im, lev.vCorners, 15, false);
        }
        if(i == 3)
        {
            Feature2dDetector::DetectFASTCorners(lev.im, lev.vCorners, 10, false);
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
        Feature2dDetector::DetectFASTCorners(lev.im, lev.vMaxCorners, 10, true);
        for(std::vector<cv::Point2f>::iterator i=lev.vMaxCorners.begin(); i!=lev.vMaxCorners.end(); i++)
        {
            if(!ImgProc::IsInImageWithBorder(lev.im,*i, 10))
                continue;
            double dSTScore = Feature2dDetector::FindShiTomasiScoreAtPoint(lev.im,*i,3);
            std::cout << "point,score: " << *i << ", " << dSTScore << std::endl;
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
