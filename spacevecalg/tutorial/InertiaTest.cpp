/*
 * Copyright 2012-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// check memory allocation in some method
#define EIGEN_RUNTIME_NO_MALLOC

// includes
// std
#include <iostream>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Xd;

const double TOL = 0.00001;

bool isUpperNull(const Eigen::Matrix3d & m)
{
  using namespace Eigen;
  return (Matrix3d(m.triangularView<StrictlyUpper>()).array() == 0.).all();
}

int main()
{
    {
        using namespace Eigen;
        using namespace sva;

        double mass = 1.;
        Matrix3d I;
        I << 1., 2., 3., 2., 1., 4., 3., 4., 1.;
        Vector3d h = Vector3d::Random() * 100.;

        // Parametrized constructor double Vector3d Matrix3d
        RBInertiad rb2(mass, h, I);

        // Parametrized constructor double Vector3d Matrix3d
        RBInertiad rb3(mass, h, I.triangularView<Lower>());

        // rbI + rbI
        RBInertiad rb4 = rb2 + rb3;

        // alpha * rbI
        RBInertiad rb5 = 2. * rb2;

        // rbI * alpha
        RBInertiad rb6 = rb2 * 2.;

        // rbI - rbI
        RBInertiad rb7 = rb2 - rb3;

        // -rbI
        RBInertiad rb8 = -rb2;

        // rbI += rbI
        RBInertiad rb9(rb2);
        rb9 += rb3;

        // rbI -= rbI
        RBInertiad rb10(rb2);
        rb10 -= rb3;
    }
    {
        using namespace Eigen;
        using namespace sva;

        Matrix3d M, H, I;
        M << 1., 2., 3., 2., 1., 4., 3., 4., 1.;
        H = Matrix3d::Random() * 100.;
        I << 1., 2., 3., 2., 1., 4., 3., 4., 1.;

        // Parametrized constructor double Vector3d Matrix3d
        ABInertiad ab1(M, H, I);

        // Parametrized constructor double Vector3d Matrix3d
        ABInertiad ab2(M.triangularView<Lower>(), H, I.triangularView<Lower>());

        // abI + abI
        ABInertiad ab3 = ab1 + ab2;

        // alpha * rbI
        ABInertiad ab4 = 2. * ab2;

        // abI * alpha
        ABInertiad ab5 = ab2 * 2.;

        // abI - abI
        ABInertiad ab6 = ab1 - ab2;

        // -abI
        ABInertiad ab7 = -ab1;

        // abI += abI
        ABInertiad ab8(ab1);
        ab8 += ab2;

        // abI -= abI
        ABInertiad ab9(ab1);
        ab9 -= ab2;
    }
    {
        using namespace Eigen;
        using namespace sva;
        double mass = 1.;
        Matrix3d I;
        I << 1., 2., 3., 2., 1., 4., 3., 4., 1.;
        Vector3d h = Vector3d::Random() * 100.;
        RBInertiad rb(mass, h, I);
        Matrix6d rb6d = rb.matrix();

        Vector3d w, v;
        w = Vector3d::Random() * 100.;
        v = Vector3d::Random() * 100.;
        sva::MotionVecd mVec(w, v);
        Vector6d mVec6d = mVec.vector();

        // RBInertiad * MotionVecd
        ForceVecd fVec = rb * mVec;
        Vector6d fVec6d(rb6d * mVec6d);

        // vectorized version
        Matrix6Xd mVec6Xd(6, 2);
        Matrix6Xd fVecRes6Xd(6, 2);
        mVec6Xd << mVec.vector(), mVec.vector();

        Eigen::internal::set_is_malloc_allowed(false);
        rb.mul(mVec6Xd, fVecRes6Xd);
        Eigen::internal::set_is_malloc_allowed(true);
    }
    {
        using namespace Eigen;
        using namespace sva;
        Matrix3d M, H, I;
        M << 1., 2., 3., 2., 1., 4., 3., 4., 1.;
        H = Matrix3d::Random() * 100.;
        I << 1., 2., 3., 2., 1., 4., 3., 4., 1.;

        ABInertiad ab(M, H, I);
        Matrix6d ab6d = ab.matrix();

        double mass = 1.;
        Vector3d h = Vector3d::Random() * 100.;
        RBInertiad rb(mass, h, I);
        Matrix6d rb6d = rb.matrix();

        Vector3d w, v;
        w = Vector3d::Random() * 100.;
        v = Vector3d::Random() * 100.;
        sva::MotionVecd mVec(w, v);
        Vector6d mVec6d = mVec.vector();

        // ABInertiad + RBInertiad
        ABInertiad abRes = ab + rb;
        Matrix6d abRes6d = ab6d + rb6d;

        // ABInertiad * MotionVecd
        ForceVecd fVec = ab * mVec;
        Vector6d fVec6d(ab6d * mVec6d);

        // vectorized version
        Matrix6Xd mVec6Xd(6, 2);
        Matrix6Xd fVecRes6Xd(6, 2);
        mVec6Xd << mVec.vector(), mVec.vector();

        Eigen::internal::set_is_malloc_allowed(false);
        ab.mul(mVec6Xd, fVecRes6Xd);
        Eigen::internal::set_is_malloc_allowed(true);
    }
}

