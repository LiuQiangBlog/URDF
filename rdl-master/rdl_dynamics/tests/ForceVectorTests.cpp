// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include "UnitTestUtils.hpp"
#include <gtest/gtest.h>

using namespace RobotDynamics::Math;

class ForceVectorTests : public testing::Test
{
  public:
    ForceVectorTests()
    {
    }

    void SetUp()
    {
    }

    void TearDown()
    {
    }
};

TEST_F(ForceVectorTests, testConstructorsAndAccessors)
{
    ForceVector f(1.1, 2.2, 3.3, 4.4, 5.5, 6.6);

    EXPECT_EQ(f.mx(), 1.1);
    EXPECT_EQ(f.my(), 2.2);
    EXPECT_EQ(f.mz(), 3.3);
    EXPECT_EQ(f.fx(), 4.4);
    EXPECT_EQ(f.fy(), 5.5);
    EXPECT_EQ(f.fz(), 6.6);

    f.mx() = -1.1;
    f.my() = -2.2;
    f.mz() = -3.3;
    f.fx() = -4.4;
    f.fy() = -5.5;
    f.fz() = -6.6;

    EXPECT_EQ(f.mx(), -1.1);
    EXPECT_EQ(f.my(), -2.2);
    EXPECT_EQ(f.mz(), -3.3);
    EXPECT_EQ(f.fx(), -4.4);
    EXPECT_EQ(f.fy(), -5.5);
    EXPECT_EQ(f.fz(), -6.6);
}

TEST_F(ForceVectorTests, testTransform)
{
    SpatialVector v(1.1, 2.1, 3.1, 4.1, 5.1, 6.1);
    ForceVector f(v);
    ForceVector f2(v), f3(v);

    SpatialTransform X = Xrotx(0.1) * Xtrans(Vector3d(0.1, 0.2, -0.3));
    f2 = X * f2;

    ForceVector f_tr = f.transform_copy(X);
    ForceVector f_exp;

    f_exp = X.toMatrixAdjoint() * ForceVector(v);
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(f_exp, f_tr, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(f_exp, f2, unit_test_utils::E_MINUS_14));
}

TEST_F(ForceVectorTests, testOperatorOverloads)
{
    ForceVector f1(0.1, 0.2, -0.3, -0.1, 1.0, -3.);
    ForceVector f2(-0.1, -0.2, 0.3, 0.1, -1.0, 3.);

    f1 += f2;
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(f1, SpatialVectorZero, unit_test_utils::E_MINUS_14));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
