// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>
#include "Fixtures.hpp"
#include "UnitTestUtils.hpp"

class UnitTestUtilsTests : public testing::Test
{
  public:
    void SetUp()
    {
    }

    void TearDown()
    {
    }
};

TEST_F(UnitTestUtilsTests, testCheckSpatialMatrixEpsilonClose)
{
    RobotDynamics::Math::SpatialMatrix m1, m2;

    m1.setZero();
    m2.setZero();

    m1(5, 5) = 0.7;
    m2(5, 5) = 0.701;

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(m1, m2, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkSpatialMatrixEpsilonClose(m1, m2, 0.0009));

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(m2, m1, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkSpatialMatrixEpsilonClose(m2, m1, 0.0009));

    m1.setZero();
    m2.setZero();

    m1(0, 0) = 0.701;
    m2(0, 0) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(m1, m2, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkSpatialMatrixEpsilonClose(m1, m2, 0.0009));

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(m2, m1, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkSpatialMatrixEpsilonClose(m2, m1, 0.0009));
}

TEST_F(UnitTestUtilsTests, testCheckSpatialVectorsEpsilonClose)
{
    RobotDynamics::Math::SpatialVector m1, m2;

    m1.setZero();
    m2.setZero();

    m1(5) = 0.7;
    m2(5) = 0.701;

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m1, m2, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkSpatialVectorsEpsilonClose(m1, m2, 0.0009));

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m2, m1, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkSpatialVectorsEpsilonClose(m2, m1, 0.0009));

    m1.setZero();
    m2.setZero();

    m1(0) = 0.701;
    m2(0) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m1, m2, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkSpatialVectorsEpsilonClose(m1, m2, 0.0009));

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m2, m1, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkSpatialVectorsEpsilonClose(m2, m1, 0.0009));
}

TEST_F(UnitTestUtilsTests, testMatrixNdEpsilonClose)
{
    RobotDynamics::Math::MatrixNd m1(2, 4), m2(2, 4);

    m1.setZero();
    m2.setZero();

    m1(1, 3) = 0.7;
    m2(1, 3) = 0.701;

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(m1, m2, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkMatrixNdEpsilonClose(m1, m2, 0.0009));

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(m2, m1, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkMatrixNdEpsilonClose(m2, m1, 0.0009));

    m1.setZero();
    m2.setZero();

    m1(0, 0) = 0.701;
    m2(0, 0) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(m1, m2, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkMatrixNdEpsilonClose(m1, m2, 0.0009));

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(m2, m1, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkMatrixNdEpsilonClose(m2, m1, 0.0009));

    RobotDynamics::Math::MatrixNd m3(4, 2);
    m3.setZero();

    try
    {
        unit_test_utils::checkMatrixNdEpsilonClose(m1, m3, 0.0011);
        FAIL();
    }
    catch (RobotDynamics::RdlException& e)
    {
        EXPECT_STREQ(e.what(), "Cannot compare MatrixNd's of different sizes!");
    }
}

TEST_F(UnitTestUtilsTests, testCheckMatrix3dEpsilonClose)
{
    RobotDynamics::Math::Matrix3d m1, m2;

    m1.setZero();
    m2.setZero();

    m1(2, 2) = 0.7;
    m2(2, 2) = 0.701;

    EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(m1, m2, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkMatrix3dEpsilonClose(m1, m2, 0.0009));

    EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(m2, m1, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkMatrix3dEpsilonClose(m2, m1, 0.0009));

    m1.setZero();
    m2.setZero();

    m1(0, 0) = 0.701;
    m2(0, 0) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(m1, m2, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkMatrix3dEpsilonClose(m1, m2, 0.0009));

    EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(m2, m1, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkMatrix3dEpsilonClose(m2, m1, 0.0009));
}

TEST_F(UnitTestUtilsTests, testCheckMatrix3dEq)
{
    RobotDynamics::Math::Matrix3d m1, m2;

    m1.setZero();
    m2.setZero();

    m1(2, 2) = 0.7;
    m2(2, 2) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkMatrix3dEq(m1, m2));

    m2(2, 2) = 0.7000001;
    EXPECT_FALSE(unit_test_utils::checkMatrix3dEq(m2, m1));

    m1.setZero();
    m2.setZero();

    m1(0, 0) = 0.7;
    m2(0, 0) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkMatrix3dEq(m1, m2));

    m2(0, 0) = 0.70000001;
    EXPECT_FALSE(unit_test_utils::checkMatrix3dEq(m2, m1));
}

TEST_F(UnitTestUtilsTests, testCheckMatrixNdEq)
{
    RobotDynamics::Math::MatrixNd m1(3, 3), m2(3, 3);

    m1.setZero();
    m2.setZero();

    m1(2, 2) = 0.7;
    m2(2, 2) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEq(m1, m2));

    m2(2, 2) = 0.7000001;
    EXPECT_FALSE(unit_test_utils::checkMatrixNdEq(m2, m1));

    m1.setZero();
    m2.setZero();

    m1(0, 0) = 0.7;
    m2(0, 0) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEq(m1, m2));

    m2(0, 0) = 0.70000001;
    EXPECT_FALSE(unit_test_utils::checkMatrixNdEq(m2, m1));

    RobotDynamics::Math::MatrixNd m3(3, 4);

    try
    {
        EXPECT_FALSE(unit_test_utils::checkMatrixNdEq(m2, m3));
        FAIL();
    }
    catch (RobotDynamics::RdlException& e)
    {
        EXPECT_STREQ(e.what(), "Cannot compare MatrixNd's of different sizes!");
    }
}

TEST_F(UnitTestUtilsTests, testCheckSpatialVectorsEq)
{
    RobotDynamics::Math::SpatialVector m1, m2;

    m1.setZero();
    m2.setZero();

    m1(2) = 0.7;
    m2(2) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEq(m1, m2));

    m2(2) = 0.7000001;
    EXPECT_FALSE(unit_test_utils::checkSpatialVectorsEq(m2, m1));

    m1.setZero();
    m2.setZero();

    m1(0) = 0.7;
    m2(0) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEq(m1, m2));

    m2(0) = 0.70000001;
    EXPECT_FALSE(unit_test_utils::checkSpatialVectorsEq(m2, m1));
}

TEST_F(UnitTestUtilsTests, testCheckVector3dEq)
{
    RobotDynamics::Math::Vector3d m1, m2;

    m1.setZero();
    m2.setZero();

    m1(2) = 0.7;
    m2(2) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkVector3dEq(m1, m2));

    m2(2) = 0.7000001;
    EXPECT_FALSE(unit_test_utils::checkVector3dEq(m2, m1));

    m1.setZero();
    m2.setZero();

    m1(0) = 0.7;
    m2(0) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkVector3dEq(m1, m2));

    m2(0) = 0.70000001;
    EXPECT_FALSE(unit_test_utils::checkVector3dEq(m2, m1));
}

TEST_F(UnitTestUtilsTests, testCheckVector4dEq)
{
    RobotDynamics::Math::Vector4d m1, m2;

    m1.setZero();
    m2.setZero();

    m1(2) = 0.7;
    m2(2) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkVector4dEq(m1, m2));

    m2(2) = 0.7000001;
    EXPECT_FALSE(unit_test_utils::checkVector4dEq(m2, m1));

    m1.setZero();
    m2.setZero();

    m1(0) = 0.7;
    m2(0) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkVector4dEq(m1, m2));

    m2(0) = 0.70000001;
    EXPECT_FALSE(unit_test_utils::checkVector4dEq(m2, m1));
}

TEST_F(UnitTestUtilsTests, testCheckVectorNdEq)
{
    RobotDynamics::Math::VectorNd m1(8), m2(8);

    m1.setZero();
    m2.setZero();

    m1(7) = 0.7;
    m2(7) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkVectorNdEq(m1, m2));

    m2(2) = 0.7000001;
    EXPECT_FALSE(unit_test_utils::checkVectorNdEq(m2, m1));

    m1.setZero();
    m2.setZero();

    m1(0) = 0.7;
    m2(0) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkVectorNdEq(m1, m2));

    m2(0) = 0.70000001;
    EXPECT_FALSE(unit_test_utils::checkVectorNdEq(m2, m1));

    RobotDynamics::Math::VectorNd m3(4);

    try
    {
        unit_test_utils::checkVectorNdEq(m2, m3);
        FAIL();
    }
    catch (RobotDynamics::RdlException& e)
    {
        EXPECT_STREQ(e.what(), "Cannot compare vectors because they are not the same length!");
    }
}

TEST_F(UnitTestUtilsTests, testCheckVector3dEpsilonClose)
{
    RobotDynamics::Math::Vector3d m1, m2;

    m1.setZero();
    m2.setZero();

    m1(2) = 0.7;
    m2(2) = 0.701;

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(m1, m2, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkVector3dEpsilonClose(m1, m2, 0.0009));

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(m2, m1, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkVector3dEpsilonClose(m2, m1, 0.0009));

    m1.setZero();
    m2.setZero();

    m1(0) = 0.701;
    m2(0) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(m1, m2, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkVector3dEpsilonClose(m1, m2, 0.0009));

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(m2, m1, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkVector3dEpsilonClose(m2, m1, 0.0009));
}

TEST_F(UnitTestUtilsTests, testCheckVector4dEpsilonClose)
{
    RobotDynamics::Math::Vector4d m1, m2;

    m1.setZero();
    m2.setZero();

    m1(2) = 0.7;
    m2(2) = 0.701;

    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(m1, m2, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkVector4dEpsilonClose(m1, m2, 0.0009));

    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(m2, m1, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkVector4dEpsilonClose(m2, m1, 0.0009));

    m1.setZero();
    m2.setZero();

    m1(0) = 0.701;
    m2(0) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(m1, m2, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkVector4dEpsilonClose(m1, m2, 0.0009));

    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(m2, m1, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkVector4dEpsilonClose(m2, m1, 0.0009));
}

TEST_F(UnitTestUtilsTests, testCheckVectorNdEpsilonClose)
{
    RobotDynamics::Math::VectorNd m1(8), m2(8);

    m1.setZero();
    m2.setZero();

    m1(7) = 0.7;
    m2(7) = 0.701;

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(m1, m2, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkVectorNdEpsilonClose(m1, m2, 0.0009));

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(m2, m1, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkVectorNdEpsilonClose(m2, m1, 0.0009));

    m1.setZero();
    m2.setZero();

    m1(0) = 0.701;
    m2(0) = 0.7;

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(m1, m2, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkVectorNdEpsilonClose(m1, m2, 0.0009));

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(m2, m1, 0.0011));
    EXPECT_FALSE(unit_test_utils::checkVectorNdEpsilonClose(m2, m1, 0.0009));

    RobotDynamics::Math::VectorNd m3(2);

    try
    {
        unit_test_utils::checkVectorNdEpsilonClose(m1, m3, 0.0011);
        FAIL();
    }
    catch (RobotDynamics::RdlException& e)
    {
        EXPECT_STREQ(e.what(), "Cannot compare vectors because they are not the same length!");
    }
}

TEST_F(FixedBase3DoFPlanar, integration)
{
    randomizeStates();
    double h = 0.001;
    Math::VectorNd x_euler = Math::VectorNd::Zero(model->q_size + model->qdot_size);
    Math::VectorNd x_rk4 = Math::VectorNd::Zero(model->q_size + model->qdot_size);
    unit_test_utils::integrateEuler(*model, Q, QDot, x_euler, Tau, h);
    unit_test_utils::integrateRk4(*model, Q, QDot, x_rk4, Tau, h);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(x_euler, x_rk4, 1.e-3));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
