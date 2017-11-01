#include "MathUtility.h"

#include <iostream>
#include <gtest/gtest.h>
#include <Eigen/Geometry>

TEST(MathUtility,EigenBasics)
{
    Eigen::Matrix4f m4;
    m4 << 1, 2, 3, 4,
          5, 6, 7, 8,
          9, 10,11,12,
          13,14,15,16;
    Eigen::VectorXf v4V = m4.row(2);

    std::cout << m4*v4V << std::endl;

    Eigen::Matrix2f m2;
    m2.row(0) = v4V.segment(0,2);
    m2.row(1) = v4V.segment(2,2);

    std::cout << "m2:\n" << m2 << std::endl;

    Eigen::Matrix3f m31;
    m31 <<  3,  2, -2,
            4, -1,  2,
           -2,  0, -1;
    Eigen::Matrix3f m32;
    m32 << 1, -2,  0,
           2,  1, -1,
           1, -3,  2 ;

    Eigen::Matrix3f m3cross;
    for(unsigned char i=0; i<3; ++i)
    {
        m3cross.col(i) = m31.col(i).cross(m32.col(i));
    }
    std::cout << "m3cross:\n" << m3cross << std::endl;
}
