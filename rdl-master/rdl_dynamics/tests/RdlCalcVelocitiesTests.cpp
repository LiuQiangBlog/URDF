// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <iostream>

#include "rdl_dynamics/Model.hpp"
#include "rdl_dynamics/Kinematics.hpp"

#include <gtest/gtest.h>

#include "UnitTestUtils.hpp"

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

class RdlCalcVelocitiesTests : public testing::Test
{
  public:
    RdlCalcVelocitiesTests()
    {
    }
};

struct RdlModelVelocitiesFixture : public testing::Test
{
    RdlModelVelocitiesFixture()
    {
    }

    void SetUp()
    {
        model = new Model;

        body_a = Body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
        Joint joint_a(SpatialVector(0., 0., 1., 0., 0., 0.));

        body_a_id = model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

        body_b = Body(1., Vector3d(0., 1., 0.), Vector3d(1., 1., 1.));
        Joint joint_b(SpatialVector(0., 1., 0., 0., 0., 0.));

        body_b_id = model->addBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

        body_c = Body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
        Joint joint_c(SpatialVector(1., 0., 0., 0., 0., 0.));

        body_c_id = model->addBody(2, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

        Q = VectorNd::Constant((size_t)model->dof_count, 0.);
        QDot = VectorNd::Constant((size_t)model->dof_count, 0.);

        point_position = Vector3d::Zero(3);
        point_velocity.setToZero();

        ref_body_id = 0;
    }

    void TearDown()
    {
        delete model;
    }

    Model* model;

    unsigned int body_a_id, body_b_id, body_c_id, ref_body_id;
    Body body_a, body_b, body_c;
    Joint joint_a, joint_b, joint_c;

    VectorNd Q;
    VectorNd QDot;

    Vector3d point_position;
    FrameVector point_velocity;
};

TEST_F(RdlModelVelocitiesFixture, TestCalcPointSimple)
{
    ref_body_id = 1;
    QDot[0] = 1.;
    point_position = Vector3d(1., 0., 0.);
    FrameVector point_velocity = calcPointVelocity(*model, Q, QDot, ref_body_id, point_position);

    EXPECT_NEAR(0., point_velocity[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(1., point_velocity[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_velocity[2], unit_test_utils::E_MINUS_14);
}

TEST_F(RdlModelVelocitiesFixture, TestCalcPointRotatedBaseSimple)
{
    // rotated first joint

    ref_body_id = 1;
    Q[0] = M_PI * 0.5;
    QDot[0] = 1.;
    point_position = Vector3d(1., 0., 0.);
    point_velocity = calcPointVelocity(*model, Q, QDot, ref_body_id, point_position);

    EXPECT_NEAR(-1., point_velocity[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_velocity[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_velocity[2], unit_test_utils::E_MINUS_14);
}

TEST_F(RdlModelVelocitiesFixture, TestCalcPointRotatingBodyB)
{
    // rotating second joint, point at third body

    ref_body_id = 3;
    QDot[1] = 1.;
    point_position = Vector3d(1., 0., 0.);
    point_velocity = calcPointVelocity(*model, Q, QDot, ref_body_id, point_position);

    EXPECT_NEAR(0., point_velocity[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_velocity[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(-1., point_velocity[2], unit_test_utils::E_MINUS_14);
}

TEST_F(RdlModelVelocitiesFixture, TestCalcPointRotatingBaseXAxis)
{
    // also rotate the first joint and take a point that is
    // on the X direction

    ref_body_id = 3;
    QDot[0] = 1.;
    QDot[1] = 1.;
    point_position = Vector3d(1., -1., 0.);
    point_velocity = calcPointVelocity(*model, Q, QDot, ref_body_id, point_position);

    EXPECT_NEAR(2., point_velocity[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_velocity[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(-1., point_velocity[2], unit_test_utils::E_MINUS_14);
}

TEST_F(RdlModelVelocitiesFixture, TestCalcPointRotatedBaseXAxis)
{
    // perform the previous test with the first joint rotated by pi/2
    // upwards

    ref_body_id = 3;
    point_position = Vector3d(1., -1., 0.);

    Q[0] = M_PI * 0.5;
    QDot[0] = 1.;
    QDot[1] = 1.;
    point_velocity = calcPointVelocity(*model, Q, QDot, ref_body_id, point_position);

    EXPECT_NEAR(-2., point_velocity[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_velocity[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(-1., point_velocity[2], unit_test_utils::E_MINUS_14);
}

TEST_F(RdlModelVelocitiesFixture, TestCalcPointBodyOrigin)
{
    // Checks whether the computation is also correct for points at the origin
    // of a body

    ref_body_id = body_b_id;
    point_position = Vector3d(0., 0., 0.);

    Q[0] = 0.;
    QDot[0] = 1.;

    point_velocity = calcPointVelocity(*model, Q, QDot, ref_body_id, point_position);

    EXPECT_NEAR(0., point_velocity[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(1., point_velocity[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_velocity[2], unit_test_utils::E_MINUS_14);
}

TEST_F(RdlCalcVelocitiesTests, FixedJointCalcPointVelocity)
{
    // the standard modeling using a null body
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Model model;

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));
    model.addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);

    SpatialTransform transform = Xtrans(Vector3d(1., 0., 0.));
    unsigned int fixed_body_id = model.appendBody(transform, Joint(JointTypeFixed), fixed_body, "fixed_body");

    VectorNd Q = VectorNd::Zero(model.dof_count);
    VectorNd QDot = VectorNd::Zero(model.dof_count);

    QDot[0] = 1.;

    FrameVector point0_velocity = calcPointVelocity(model, Q, QDot, fixed_body_id, Vector3d(0., 0., 0.));

    FrameVector point1_velocity = calcPointVelocity(model, Q, QDot, fixed_body_id, Vector3d(1., 0., 0.));

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 1., 0.), point0_velocity, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 2., 0.), point1_velocity, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlCalcVelocitiesTests, FixedJointCalcPointVelocityRotated)
{
    // the standard modeling using a null body
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Model model;

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));
    model.addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);

    SpatialTransform transform = Xtrans(Vector3d(1., 0., 0.));
    unsigned int fixed_body_id = model.appendBody(transform, Joint(JointTypeFixed), fixed_body, "fixed_body");

    VectorNd Q = VectorNd::Zero(model.dof_count);
    VectorNd QDot = VectorNd::Zero(model.dof_count);

    Q[0] = M_PI * 0.5;
    QDot[0] = 1.;

    FrameVector point0_velocity = calcPointVelocity(model, Q, QDot, fixed_body_id, Vector3d(0., 0., 0.));

    FrameVector point1_velocity = calcPointVelocity(model, Q, QDot, fixed_body_id, Vector3d(1., 0., 0.));

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(-1., 0., 0.), point0_velocity, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(-2., 0., 0.), point1_velocity, unit_test_utils::E_MINUS_14));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
