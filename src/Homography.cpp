#include "Homography.h"

#include <Eigen/SVD>

#include "Common.h"

int Homography::HomographyFromMatches(std::vector<HomographyMatch> vMatches, Eigen::Matrix3f &m3Homography)
{
    if(vMatches.empty())
        return GS::RET_FAILED;
    unsigned int nSizeMatches = vMatches.size();
    if(nSizeMatches < 4)
        return GS::RET_FAILED;
    int nRows = 2 * nSizeMatches;
    if(nRows < 9)
        nRows = 9;

    Eigen::MatrixXf m2Nx9(nRows,9);
    m2Nx9 = Eigen::MatrixXf::Zero(nRows,9);
    for(unsigned int n=0; n<nSizeMatches; n++)
    {
        double u = vMatches[n].v2CamPlaneSecond[0];
        double v = vMatches[n].v2CamPlaneSecond[1];

        double x = vMatches[n].v2CamPlaneFirst[0];
        double y = vMatches[n].v2CamPlaneFirst[1];

        // [u v]T = H [x y]T
        m2Nx9(n*2+0,0) = x;
        m2Nx9(n*2+0,1) = y;
        m2Nx9(n*2+0,2) = 1;
        m2Nx9(n*2+0,3) = 0;
        m2Nx9(n*2+0,4) = 0;
        m2Nx9(n*2+0,5) = 0;
        m2Nx9(n*2+0,6) = -x*u;
        m2Nx9(n*2+0,7) = -y*u;
        m2Nx9(n*2+0,8) = -u;

        m2Nx9(n*2+1,0) = 0;
        m2Nx9(n*2+1,1) = 0;
        m2Nx9(n*2+1,2) = 0;
        m2Nx9(n*2+1,3) = x;
        m2Nx9(n*2+1,4) = y;
        m2Nx9(n*2+1,5) = 1;
        m2Nx9(n*2+1,6) = -x*v;
        m2Nx9(n*2+1,7) = -y*v;
        m2Nx9(n*2+1,8) = -v;
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(m2Nx9,Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf m9V = svd.matrixV();
    Eigen::VectorXf v9V = m9V.col(8);//column vector correspond to the smallest singular value

    m3Homography.row(0) = v9V.segment(0,3);
    m3Homography.row(1) = v9V.segment(3,3);
    m3Homography.row(2) = v9V.segment(6,3);

    return GS::RET_SUCESS;
}
