/*
 * Copyright 2012-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// check memory allocation in some method
#define EIGEN_RUNTIME_NO_MALLOC

// includes
// std
#include <iostream>

// SpaceVecAlg
#include <SpaceVecAlg/Conversions.h>
#include "Constants.h"

int main()
{
    {
        using namespace Eigen;
        using namespace sva;

        Matrix4d hom   = Matrix4d::Zero();
        hom(0, 1)      = -1;
        hom(1, 0)      = +1;
        hom(2, 2)      = +1;
        hom(3, 3)      = +1;
        PTransformd pt = conversions::fromHomogeneous(hom, conversions::RightHanded);

        const Vector4d vec(4, 3, 2, 1);
        Matrix4d homVec          = Matrix4d::Identity();
        homVec.block<4, 1>(0, 3) = vec;
        PTransformd ptVec        = conversions::fromHomogeneous(homVec, conversions::RightHanded);

        Matrix4d hom2 = conversions::toHomogeneous(pt, conversions::RightHanded);

        const PTransformd rotatedPT = ptVec * pt;
        const Vector4d rotated      = hom * vec;
        const Vector4d rotated2     = hom2 * vec;
    }

    {
        using namespace Eigen;
        using namespace sva;

        PTransformd pt(sva::RotX(Pi / 2) * sva::RotZ(Pi / 4), Eigen::Vector3d(1., 2., 3.));
        conversions::affine3_t<double> et = conversions::toAffine(pt, conversions::RightHanded);
        PTransformd pt2                   = conversions::fromAffine(et, conversions::RightHanded);

        conversions::affine3_t<double> etL = conversions::toAffine(pt, conversions::LeftHanded);
        PTransformd pt3                    = conversions::fromAffine(etL, conversions::LeftHanded);

        Eigen::Vector3d vec(5, 7, 12);

        sva::PTransformd transformed      = sva::PTransformd(vec) * pt;
        Eigen::Vector3d affineTransformed = et * vec;

        std::cout << transformed.translation().transpose() << std::endl;
        std::cout << affineTransformed.transpose() << std::endl;
    }

    return 0;
}
