// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>
#include "rdl_dynamics/Joint.hpp"
#include "UnitTestUtils.hpp"
#include "rdl_dynamics/Model.hpp"

using namespace RobotDynamics;

class JointTestsFixture : public testing::Test
{
};

TEST_F(JointTestsFixture, testJointJcalc_XJ)
{
    Model m;
    Body b(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
    Joint j(JointTypeRevolute, Vector3d(1., 0., 0.));
    unsigned int id = m.addBody(0, RobotDynamics::Math::SpatialTransform(), j, b);
    VectorNd q(m.q_size);
    q[0] = 0.1;
    SpatialTransform X_calc = RobotDynamics::jcalc_XJ(m, id, q);

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(X_calc.toMatrix(), Xrotx(0.1).toMatrix(), unit_test_utils::E_MINUS_14));

    j = Joint(JointTypeEulerXYZ);
    id = m.appendBody(RobotDynamics::Math::SpatialTransform(), j, b);
    try
    {
        X_calc = RobotDynamics::jcalc_XJ(m, id, q);
        FAIL();
    }
    catch (RobotDynamics::RdlException& e)
    {
        EXPECT_STREQ(e.what(), "Error: invalid joint type!");
    }
}

TEST_F(JointTestsFixture, testJointTypeConstructor)
{
    Joint j1(JointTypeRevoluteX);
    EXPECT_EQ(j1.mDoFCount, (unsigned int)1);
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEq(SpatialVector(1., 0., 0., 0., 0., 0.), *j1.mJointAxes));

    Joint j2(JointTypeRevoluteY);
    EXPECT_EQ(j2.mDoFCount, (unsigned int)1);
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEq(SpatialVector(0., 1., 0., 0., 0., 0.), *j2.mJointAxes));

    Joint j3(JointTypeRevoluteZ);
    EXPECT_EQ(j3.mDoFCount, (unsigned int)1);
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEq(SpatialVector(0., 0., 1., 0., 0., 0.), *j3.mJointAxes));

    Joint j4(JointType5DoF);
    EXPECT_EQ(j4.mDoFCount, (unsigned int)5);

    try
    {
        Joint j5(JointTypeCustom);
        FAIL();
    }
    catch (RobotDynamics::RdlException& e)
    {
        EXPECT_STREQ(e.what(), "Error: Invalid use of Joint constructor Joint(JointType type). Only allowed when type != JointTypeCustom");
    }

    try
    {
        Joint j5(JointTypeUndefined);
        FAIL();
    }
    catch (RobotDynamics::RdlException& e)
    {
        EXPECT_STREQ(e.what(), "Error: Invalid use of Joint constructor Joint(JointType type).");
    }
}

TEST_F(JointTestsFixture, testJointNumDegFreedomConstructor)
{
    try
    {
        Joint j1(JointType6DoF, 3);
        FAIL();
    }
    catch (RobotDynamics::RdlException& e)
    {
        EXPECT_STREQ(e.what(), "Error: Invalid use of Joint constructor Joint(JointType type, int degreesOfFreedom). Only allowed when type  == JointTypeCustom.");
    }

    Joint j1(JointTypeCustom, 1);
    EXPECT_EQ(j1.mDoFCount, (unsigned int)1);
}

TEST_F(JointTestsFixture, testJointTypeAxisConstructor)
{
    Joint j(JointTypeRevolute, Vector3d(1., 0., 0.));

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEq(*j.mJointAxes, SpatialVector(1., 0., 0., 0., 0., 0.)));
}

TEST_F(JointTestsFixture, testValidateSpatialAxis)
{
    SpatialVector v(1., 1., 1., 1., 1., 1.);
    Joint j;

    EXPECT_FALSE(j.validate_spatial_axis(v));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
