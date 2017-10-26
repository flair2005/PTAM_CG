#include "ATANCamera.h"


// Project from the camera z=1 plane to image pixels,
cv::Point2f ATANCamera::Project(const cv::Point2f& vCam)
{
    mvLastCam = vCam;
    mdLastR = std::sqrt(vCam.ddot(vCam));
    mbInvalid = (mdLastR > mdMaxR);
    mdLastFactor = rtrans_factor(mdLastR);
    mdLastDistR = mdLastFactor * mdLastR;
    mvLastDistCam = mdLastFactor * mvLastCam;

    mvLastIm.x = mvCenter.x + mvFocal[0] * mvLastDistCam.x;
    mvLastIm.y = mvCenter.y + mvFocal[1] * mvLastDistCam.y;

    return mvLastIm;
}

// Un-project from image pixel coords to the camera z=1 plane
cv::Point2f ATANCamera::UnProject(const cv::Point2f& v2Im)
{
    mvLastIm = v2Im;
    mvLastDistCam.x = (mvLastIm.x - mvCenter.x) * mvInvFocal[0];
    mvLastDistCam.y = (mvLastIm.x - mvCenter.x) * mvInvFocal[1];
    mdLastDistR = std::sqrt(mvLastDistCam.ddot(mvLastDistCam));
    mdLastR = invrtrans(mdLastDistR);
    double dFactor;
    if(mdLastDistR > 0.01)
        dFactor =  mdLastR / mdLastDistR;
    else
        dFactor = 1.0;
    mdLastFactor = 1.0 / dFactor;
    mvLastCam = dFactor * mvLastDistCam;
    return mvLastCam;
}

cv::Matx22f ATANCamera::GetProjectionDerivs()
{
    // get the derivative of image frame wrt camera z=1 frame at the last computed projection
    // in the form (d im1/d cam1, d im1/d cam2)
    //             (d im2/d cam1, d im2/d cam2)

    double dFracBydx = 0.0;
    double dFracBydy = 0.0;

    const double &k = md2Tan;
    const double &x = mvLastCam.x;
    const double &y = mvLastCam.y;
    double r = mdLastR * mdDistortionEnabled;

    if(r < 0.01)
    {
        dFracBydx = 0.0;
        dFracBydy = 0.0;
    }
    else
    {
        dFracBydx = mdWinv * (k * x) / (r*r*(1 + k*k*r*r)) - x * mdLastFactor / (r*r);
        dFracBydy = mdWinv * (k * y) / (r*r*(1 + k*k*r*r)) - y * mdLastFactor / (r*r);
    }

    cv::Matx22f m2Derivs = cv::Matx22f::ones();
    m2Derivs(0,0) = mvFocal[0] * (dFracBydx * x + mdLastFactor);
    m2Derivs(1,0) = mvFocal[1] * (dFracBydx * y);
    m2Derivs(0,1) = mvFocal[0] * (dFracBydy * x);
    m2Derivs(1,1) = mvFocal[1] * (dFracBydy * y + mdLastFactor);

    return m2Derivs;
}
