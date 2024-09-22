// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>
#include "UnitTestUtils.hpp"

using namespace RobotDynamics::Math;

class MotionVectorTests : public testing::Test
{
  public:
    MotionVectorTests(){};

    void SetUp()
    {
    }

    void TearDown()
    {
    }
};

TEST_F(MotionVectorTests, TestSpatialVectorCross)
{
    MotionVector s_vec(1., 2., 3., 4., 5., 6.);

    SpatialMatrix test_cross(0., -3., 2., 0., 0., 0., 3., 0., -1., 0., 0., 0., -2., 1., 0., 0., 0., 0., 0., -6., 5., 0., -3., 2., 6., 0., -4., 3., 0., -1., -5., 4., 0.,
                             -2., 1., 0.);

    SpatialMatrix s_vec_cross(s_vec.crossm());
    EXPECT_EQ(test_cross, s_vec_cross);

    SpatialMatrix s_vec_crossf(s_vec.crossf());
    SpatialMatrix test_crossf = -1. * s_vec.crossm().transpose();

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(test_crossf, s_vec_crossf, unit_test_utils::E_MINUS_14));
}

TEST_F(MotionVectorTests, TestSpatialVectorCrossmCrossf)
{
    MotionVector s_vec(1., 2., 3., 4., 5., 6.);
    MotionVector t_vec(9., 8., 7., 6., 5., 4.);
    ForceVector f_vec = t_vec;

    // by explicitly building the matrices (crossm/f with only one vector)
    SpatialVector crossm_s_x_t = s_vec.crossm() * t_vec;
    SpatialVector crossf_s_x_t = s_vec.crossf() * t_vec;

    // by using direct computation that avoids building of the matrix
    SpatialVector crossm_s_t = s_vec % t_vec;
    SpatialVector crossf_s_t = s_vec % f_vec;

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(crossm_s_x_t, crossm_s_t, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(crossf_s_x_t, crossf_s_t, unit_test_utils::E_MINUS_14));
}

TEST_F(MotionVectorTests, testMotionCross)
{
    MotionVector m1(1.1, 2.2, 3.3, 4.4, 5.5, 6.6);
    ForceVector f1 = m1;
    MotionVector m2(1.3, 1. - 2, 3.6, 4.44, 23.5, 4.6);

    SpatialMatrix crossm, crossf;

    crossm.block<3, 3>(0, 0) = m2.getAngularPart().toTildeForm();
    crossm.block<3, 3>(3, 3) = m2.getAngularPart().toTildeForm();
    crossm.block<3, 3>(3, 0) = m2.getLinearPart().toTildeForm();

    crossf.block<3, 3>(0, 0) = m2.getAngularPart().toTildeForm();
    crossf.block<3, 3>(3, 3) = m2.getAngularPart().toTildeForm();
    crossf.block<3, 3>(0, 3) = m2.getLinearPart().toTildeForm();

    MotionVector m3 = crossm * m1;
    MotionVector m4 = m2 % m1;

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m4, m3, unit_test_utils::E_MINUS_14));

    ForceVector f2 = crossf * f1;
    ForceVector f3 = m2 % f1;

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(f2, f3, unit_test_utils::E_MINUS_14));
}

TEST_F(MotionVectorTests, testConstructorsAndAccessors)
{
    MotionVector m(1.1, 2.2, 3.3, 4.4, 5.5, 6.6);

    EXPECT_EQ(m.wx(), 1.1);
    EXPECT_EQ(m.wy(), 2.2);
    EXPECT_EQ(m.wz(), 3.3);
    EXPECT_EQ(m.vx(), 4.4);
    EXPECT_EQ(m.vy(), 5.5);
    EXPECT_EQ(m.vz(), 6.6);

    m.wx() = -1.1;
    m.wy() = -2.2;
    m.wz() = -3.3;
    m.vx() = -4.4;
    m.vy() = -5.5;
    m.vz() = -6.6;

    EXPECT_EQ(m.wx(), -1.1);
    EXPECT_EQ(m.wy(), -2.2);
    EXPECT_EQ(m.wz(), -3.3);
    EXPECT_EQ(m.vx(), -4.4);
    EXPECT_EQ(m.vy(), -5.5);
    EXPECT_EQ(m.vz(), -6.6);
}

TEST_F(MotionVectorTests, testTransform)
{
    Vector3d rot(1.1, 1.2, 1.3);
    Vector3d trans(1.1, 1.2, 1.3);

    SpatialTransform X_st;
    X_st.r = trans;

    SpatialMatrix X_66_matrix(SpatialMatrix::Zero(6, 6));
    X_66_matrix = Xrotz(rot[2]).toMatrix() * Xroty(rot[1]).toMatrix() * Xrotx(rot[0]).toMatrix() * SpatialTransform(trans).toMatrix();
    X_st.E = X_66_matrix.block<3, 3>(0, 0);

    SpatialVector v(1.1, 2.1, 3.1, 4.1, 5.1, 6.1);
    MotionVector m(v);
    MotionVector m2(v);
    m.transform(X_st);
    m2 = X_st * m2;
    SpatialVector v_66_res = X_66_matrix * v;
    EXPECT_TRUE(v_66_res.isApprox(m, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(v_66_res.isApprox(m2, unit_test_utils::E_MINUS_14));
}

TEST_F(MotionVectorTests, testOperatorOverloads)
{
    MotionVector m1 = MotionVector(0.1, 0.2, -0.3, -0.1, 1.0, -2.);
    MotionVector m2 = MotionVector(-0.1, -0.2, 0.3, 0.1, -1.0, 2.);

    m1 += m2;
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m1, SpatialVectorZero, unit_test_utils::E_MINUS_14));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
