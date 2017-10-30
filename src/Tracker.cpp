#include "Tracker.h"

#include <utility>

#include "GLWindowPangolin.h"
#include "MapMaker.h"

Tracker::Tracker(GLWindowPangolin *pWindowPangolin, MapMaker &mapmaker):
    mpPangolinWindow(pWindowPangolin),
    mMapMaker(mapmaker)
{
    mCurrentKF.bFixed = false;
    Reset();
}

void Tracker::TrackFrame(const cv::Mat &imgBW, bool bDraw)
{
    mbDraw = bDraw;
    mCurrentKF.MakeKeyFrame_Lite(imgBW);

    mnFrame++;

    if(mbDraw)
    {
        mpPangolinWindow->DrawPoints2D(mCurrentKF.aLevels[0].vCorners, GS::RGB(1,0,1), 1.0f);
    }

    TrackForInitialMap();
}

void Tracker::TrackForInitialMap()
{
//    static gvar3<int> gvnMaxSSD("Tracker.MiniPatchMaxSSD", 100000, SILENT);
//    MiniPatch::mnMaxSSD = *gvnMaxSSD;
    MiniPatch::mnMaxSSD = 100000;

    if(mnInitialStage == TRAIL_TRACKING_NOT_STARTED)
    {
        if(mnFrame==5)
        {
            TrailTracking_Start();
            mnInitialStage = TRAIL_TRACKING_STARTED;
        }
        return;
    }
    if(mnInitialStage == TRAIL_TRACKING_STARTED)
    {
        int nGoodTrails = TrailTracking_Advance();
        if(nGoodTrails < 10)
        {
            Reset();
            return;
        }
        if(mnFrame==12)
        {
            std::vector<std::pair<cv::Point2i, cv::Point2i> > vMatches;
            for(std::list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end(); i++)
            {
                vMatches.push_back(std::pair<cv::Point2i, cv::Point2i>(i->ptInitialPos,i->ptCurrentPos));
            }
            mMapMaker.InitFromStereo(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld);
            mnInitialStage = TRAIL_TRACKING_COMPLETE;
        }
    }
}

bool sort_judge(const std::pair<double,cv::Point2i> a,const std::pair<double,cv::Point2i> b)
{
    return a.first > b.first;
}

void Tracker::TrailTracking_Start()
{
    mCurrentKF.MakeKeyFrame_Rest();
    mFirstKF = mCurrentKF;

    std::vector<std::pair<double,cv::Point2i> > vCornersAndSTScores;
    for(unsigned int i=0; i<mCurrentKF.aLevels[0].vCandidates.size(); i++)
    {                                                                     // so that we can choose the image corners with max ST score
        Candidate &c = mCurrentKF.aLevels[0].vCandidates[i];
        if(!ImgProc::IsInImageWithBorder(mCurrentKF.aLevels[0].im,c.ptLevelPos,MiniPatch::mnHalfPatchSize))
            continue;
        vCornersAndSTScores.push_back(std::pair<double,cv::Point2i>(c.dSTScore, c.ptLevelPos));
    }
    // Sort according to Shi-Tomasi score, highest score first in sorted list
    std::sort(vCornersAndSTScores.begin(), vCornersAndSTScores.end(), sort_judge);

    int nToAdd = 1000;//GV2.GetInt("MaxInitialTrails", 1000, SILENT);
    for(unsigned int i = 0; i<vCornersAndSTScores.size() && nToAdd > 0; i++)
    {
        if(!ImgProc::IsInImageWithBorder(mCurrentKF.aLevels[0].im,vCornersAndSTScores[i].second,MiniPatch::mnHalfPatchSize))
            continue;
        Trail t;
        t.mPatch.SampleFromImage(mCurrentKF.aLevels[0].im, vCornersAndSTScores[i].second);
        t.ptInitialPos = vCornersAndSTScores[i].second;
        t.ptCurrentPos = t.ptInitialPos;
        mlTrails.push_back(t);
        nToAdd--;
    }

    mPreviousFrameKF = mFirstKF;
}

int Tracker::TrailTracking_Advance()
{
    int nGoodTrails = 0;
    MiniPatch BackwardsPatch;
    Level &lCurrentFrame = mCurrentKF.aLevels[0];
    Level &lPreviousFrame = mPreviousFrameKF.aLevels[0];

    for(std::list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end();)
    {
        std::list<Trail>::iterator next = i;
        next++;
        Trail &trail = *i;
        cv::Point2i ptStart = trail.ptCurrentPos;
        cv::Point2i ptEnd = ptStart;
        bool bFound = trail.mPatch.FindPatch(ptEnd, lCurrentFrame.im, 10, lCurrentFrame.vCorners);
        if(bFound)
        {
            // Also find backwards in a married-matches check
            BackwardsPatch.SampleFromImage(lCurrentFrame.im, ptEnd);
            cv::Point2i ptBackWardsFound = ptEnd;
            bFound = BackwardsPatch.FindPatch(ptBackWardsFound, lPreviousFrame.im, 10, lPreviousFrame.vCorners);
            cv::Point2i diffPts = ptBackWardsFound - ptStart;
            if(diffPts.dot(diffPts) > 2)
                bFound = false;
            trail.ptCurrentPos = ptEnd;
            nGoodTrails++;
        }
        if(mbDraw)
        {
            GS::RGB rgbStart, rgbEnd;
            if(bFound)
            {
                rgbStart = GS::RGB(1,1,0);
                rgbEnd   = GS::RGB(1,0,0);
            }
            else
            {
                rgbStart = rgbEnd = GS::RGB(0,1,1);
            }
            mpPangolinWindow->DrawLines(trail.ptInitialPos, rgbStart, trail.ptCurrentPos, rgbEnd, 2.0f);
        }
        if(!bFound)
        {
            mlTrails.erase(i);
        }
        i = next;
    }

    mPreviousFrameKF = mCurrentKF;
    return nGoodTrails;
}

void Tracker::Reset()
{
    mnFrame = 0;
    mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
    mlTrails.clear();
}
