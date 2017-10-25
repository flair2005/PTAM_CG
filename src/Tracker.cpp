#include "Tracker.h"
#include "GLWindowPangolin.h"

#include <utility>

Tracker::Tracker()
{
    mCurrentKF.bFixed = false;

    Reset();
}

void Tracker::TrackFrame(cv::Mat imgBW, bool bDraw)
{
    mbDraw = bDraw;
    mCurrentKF.MakeKeyFrame_Lite(imgBW);

    mnFrame++;

    if(false)
    {
        GLWindowPangolin pangolinWin;
        pangolinWin.DrawPoints2f(mCurrentKF.aLevels[0].vCorners, GLWindowPangolin::RGB(1,0,1), 1.0f);
    }

    TrackForInitialMap();
}

void Tracker::TrackForInitialMap()
{
//    // MiniPatch tracking threshhold.
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
        int nGoodTrails = TrailTracking_Advance();  // This call actually tracks the trails
        if(nGoodTrails < 10) // if most trails have been wiped out, no point continuing.
        {
            Reset();
            return;
        }

        if(mnFrame==12)
        {
//            vector<pair<ImageRef, ImageRef> > vMatches;   // This is the format the mapmaker wants for the stereo pairs
//            for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end(); i++)
//                vMatches.push_back(pair<ImageRef, ImageRef>(i->irInitialPos,i->irCurrentPos));
//            mMapMaker.InitFromStereo(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld);  // This will take some time!
            exit(1);
            mnInitialStage = TRAIL_TRACKING_COMPLETE;
        }
    }

}

bool sort_judge(const std::pair<double,cv::Point2f> a,const std::pair<double,cv::Point2f> b)
{
    return a.first > b.first;
}

void Tracker::TrailTracking_Start()
{
    mCurrentKF.MakeKeyFrame_Rest();
    mFirstKF = mCurrentKF;

    std::vector<std::pair<double,cv::Point2f> > vCornersAndSTScores;
    for(unsigned int i=0; i<mCurrentKF.aLevels[0].vCandidates.size(); i++)
    {                                                                     // so that we can choose the image corners with max ST score
        Candidate &c = mCurrentKF.aLevels[0].vCandidates[i];
        if(!ImgProc::IsInImageWithBorder(mCurrentKF.aLevels[0].im,c.ptLevelPos,MiniPatch::mnHalfPatchSize))
            continue;
        vCornersAndSTScores.push_back(std::pair<double,cv::Point2f>(c.dSTScore, c.ptLevelPos));
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

// Steady-state trail tracking: Advance from the previous frame, remove duds.
int Tracker::TrailTracking_Advance()
{
    int nGoodTrails = 0;
    if(mbDraw)
    {
        glPointSize(5);
        glLineWidth(2);
        glEnable(GL_POINT_SMOOTH);
        glEnable(GL_LINE_SMOOTH);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
        glBegin(GL_LINES);
    }

    MiniPatch BackwardsPatch;
    Level &lCurrentFrame = mCurrentKF.aLevels[0];
    Level &lPreviousFrame = mPreviousFrameKF.aLevels[0];

    for(std::list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end();)
    {
        std::list<Trail>::iterator next = i;
        next++;
        Trail &trail = *i;
        cv::Point2f ptStart = trail.ptCurrentPos;
        cv::Point2f ptEnd = ptStart;
        bool bFound = trail.mPatch.FindPatch(ptEnd, lCurrentFrame.im, 10, lCurrentFrame.vCorners);
        if(bFound)
        {
            // Also find backwards in a married-matches check
            BackwardsPatch.SampleFromImage(lCurrentFrame.im, ptEnd);
            cv::Point2f ptBackWardsFound = ptEnd;
            bFound = BackwardsPatch.FindPatch(ptBackWardsFound, lPreviousFrame.im, 10, lPreviousFrame.vCorners);
            cv::Point2f diffPts = ptBackWardsFound - ptStart;
            if(diffPts.dot(diffPts) > 2)
                bFound = false;
            trail.ptCurrentPos = ptEnd;
            nGoodTrails++;
        }
        if(mbDraw)
        {
            if(!bFound)
                glColor3f(0,1,1); // Failed trails flash purple before dying.
            else
                glColor3f(1,1,0);
            glVertex2f(trail.ptInitialPos.x, trail.ptInitialPos.y);
            if(bFound)
                glColor3f(1,0,0);
            glVertex2f(trail.ptCurrentPos.x, trail.ptCurrentPos.y);
        }
        if(!bFound) // Erase from list of trails if not found this frame.
        {
            mlTrails.erase(i);
        }
        i = next;
    }
    if(mbDraw)
        glEnd();

    mPreviousFrameKF = mCurrentKF;
    return nGoodTrails;
}

void Tracker::Reset()
{
    mnFrame = 0;
    mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
    mlTrails.clear();
}
