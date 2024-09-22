/*
 * Copyright 2012-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// includes
// std
#include <iostream>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Xd;

int main()
{
    {
        using namespace sva;

        const std::size_t size = 10000000;
        std::vector<PTransformd> pt1(size, PTransformd::Identity());
        std::vector<PTransformd> pt2(size, PTransformd::Identity());
        std::vector<PTransformd> ptRes(size);

        std::cout << "PTransform vs PTransform" << std::endl;
        {
            for(std::size_t i = 0; i < size; ++i)
            {
                ptRes[i] = pt1[i] * pt2[i];
            }
        }
        std::cout << std::endl;
    }
    {
        using namespace sva;

        const std::size_t size = 10000000;
        std::vector<PTransformd> pt1(size, PTransformd::Identity());
        std::vector<MotionVecd> mv(size, MotionVecd(Eigen::Vector6d::Random()));
        std::vector<MotionVecd> mvRes(size);

        std::cout << "PTransform vs MotionVec" << std::endl;
        {
            for(std::size_t i = 0; i < size; ++i)
            {
                mvRes[i] = pt1[i] * mv[i];
            }
        }
        std::cout << std::endl;
    }
    {
        using namespace sva;

        const std::size_t size = 10000000;
        const std::size_t cols = 3;
        std::vector<PTransformd> pt1(size, PTransformd::Identity());
        std::vector<Matrix6Xd> mv(size, Matrix6Xd::Random(6, cols));
        std::vector<Matrix6Xd> mvRes(size, Matrix6Xd(6, cols));

        std::cout << "PTransform vs MotionEigen" << std::endl;
        {
            for(std::size_t i = 0; i < size; ++i)
            {
                pt1[i].mul(mv[i], mvRes[i]);
            }
        }
        std::cout << std::endl;
    }
    {
        using namespace sva;

        const std::size_t size = 10000000;
        const std::size_t cols = 3;
        std::vector<PTransformd> pt1(size, PTransformd::Identity());
        std::vector<Matrix6Xd> mv(size, Matrix6Xd::Random(6, cols));
        std::vector<Matrix6Xd> mvRes(size, Matrix6Xd(6, cols));

        std::cout << "PTransform as matrix vs MotionEigen" << std::endl;
        {
            for(std::size_t i = 0; i < size; ++i)
            {
                mvRes[i].noalias() = pt1[i].matrix() * mv[i];
            }
        }
        std::cout << std::endl;
    }
    {
        using namespace sva;

        const std::size_t size = 10000000;
        const std::size_t cols = 3;
        std::vector<PTransformd> pt1(size, PTransformd::Identity());
        std::vector<Matrix6Xd> mv(size, Matrix6Xd::Random(6, cols));
        std::vector<Matrix6Xd> mvRes(size, Matrix6Xd(6, cols));

        std::cout << "PTransform vs MotionEigen as MotionVec" << std::endl;
        {
            for(std::size_t i = 0; i < size; ++i)
            {
                for(std::size_t j = 0; j < cols; ++j)
                {
                    mvRes[i].col(j).noalias() = (pt1[i] * MotionVecd(mv[i].col(j))).vector();
                }
            }
        }
        std::cout << std::endl;
    }
    {
        using namespace sva;

        const std::size_t size = 10000000;
        std::vector<PTransformd> pt1(size, PTransformd::Identity());
        std::vector<MotionVecd> mv(size, MotionVecd(Eigen::Vector6d::Random()));
        std::vector<MotionVecd> mvRes(size);

        std::cout << "PTransform_inv vs MotionVec" << std::endl;
        {
            for(std::size_t i = 0; i < size; ++i)
            {
                mvRes[i] = pt1[i].inv() * mv[i];
            }
        }
        std::cout << std::endl;
    }
    {
        using namespace sva;

        const std::size_t size = 10000000;
        std::vector<PTransformd> pt1(size, PTransformd::Identity());
        std::vector<MotionVecd> mv(size, MotionVecd(Eigen::Vector6d::Random()));
        std::vector<MotionVecd> mvRes(size);

        std::cout << "PTransform_invMul vs MotionVec" << std::endl;
        {
            for(std::size_t i = 0; i < size; ++i)
            {
                mvRes[i] = pt1[i].invMul(mv[i]);
            }
        }
        std::cout << std::endl;
    }
    {
        using namespace sva;

        const std::size_t size = 10000000;
        std::vector<PTransformd> pt1(size, PTransformd::Identity());
        std::vector<ForceVecd> mv(size, ForceVecd(Eigen::Vector6d::Random()));
        std::vector<ForceVecd> mvRes(size);

        std::cout << "PTransform dual vs ForceVec" << std::endl;
        {
            for(std::size_t i = 0; i < size; ++i)
            {
                mvRes[i] = pt1[i].dualMul(mv[i]);
            }
        }
        std::cout << std::endl;
    }
    {
        using namespace sva;

        const std::size_t size = 10000000;
        std::vector<PTransformd> pt1(size, PTransformd::Identity());
        std::vector<ForceVecd> mv(size, ForceVecd(Eigen::Vector6d::Random()));
        std::vector<ForceVecd> mvRes(size);

        std::cout << "PTransform trans vs ForceVec" << std::endl;
        {
            for(std::size_t i = 0; i < size; ++i)
            {
                mvRes[i] = pt1[i].transMul(mv[i]);
            }
        }
        std::cout << std::endl;
    }

    return 0;
}


