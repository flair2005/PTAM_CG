#ifndef __MathUtility_H
#define __MathUtility_H

#include <Eigen/Core>

namespace cg {

class MathUtility
{
public:
    inline static Eigen::Vector3d Homogeneous(Eigen::Vector2d v2d)
    {
        Eigen::Vector3d v3d;
        v3d.segment(0,2) = v2d;
        v3d[2] = 1.0;
        return v3d;
    }
    inline static Eigen::Vector2d InHomogeneous(Eigen::Vector3d v3d)
    {
        Eigen::Vector2d v2d;
        v2d[0] = v3d[0]/v3d[2];
        v2d[1] = v3d[1]/v3d[2];
        return v2d;
    }
};

}


#endif
