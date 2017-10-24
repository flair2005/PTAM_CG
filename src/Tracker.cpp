#include "Tracker.h"
#include "GLWindowPangolin.h"

void Tracker::TrackFrame(cv::Mat imgBW, bool bDraw)
{
    mCurrentKF.MakeKeyFrame_Lite(imgBW);

    if(bDraw)
    {
        GLWindowPangolin pangolinWin;
        pangolinWin.DrawPoints2f(mCurrentKF.aLevels[0].vCorners, GLWindowPangolin::RGB(1,0,1), 1.0f);
    }
}
