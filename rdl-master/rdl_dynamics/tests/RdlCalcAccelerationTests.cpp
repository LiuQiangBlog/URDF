// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <iostream>

#include <gtest/gtest.h>

#include "rdl_dynamics/Model.hpp"
#include "rdl_dynamics/Kinematics.hpp"
#include "rdl_dynamics/Dynamics.hpp"

#include "UnitTestUtils.hpp"

#include "Fixtures.hpp"

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

class RdlCalcAccelerationTests : public testing::Test
{
  public:
    RdlCalcAccelerationTests()
    {
    }

    void SetUp()
    {
    }

    void TearDown()
    {
    }
};

TEST_F(FixedBase3DoF, TestCalcPointSimple)
{
    QDDot[0] = 1.;
    ref_body_id = body_a_id;
    point_position = Vector3d(1., 0., 0.);
    point_accel = calcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

    EXPECT_NEAR(0., point_accel[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(1., point_accel[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_accel[2], unit_test_utils::E_MINUS_14);
}

TEST_F(FixedBase3DoF, TestCalcPointSimpleRotated)
{
    Q[0] = 0.5 * M_PI;

    ref_body_id = body_a_id;
    QDDot[0] = 1.;
    point_position = Vector3d(1., 0., 0.);
    point_accel = calcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

    EXPECT_NEAR(-1., point_accel[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_accel[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_accel[2], unit_test_utils::E_MINUS_14);
}

TEST_F(FixedBase3DoF, TestCalcPointRotation)
{
    ref_body_id = 1;
    QDot[0] = 1.;
    point_position = Vector3d(1., 0., 0.);
    point_accel = calcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

    EXPECT_NEAR(-1., point_accel[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_accel[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_accel[2], unit_test_utils::E_MINUS_14);

    // if we are on the other side we should have the opposite value
    point_position = Vector3d(-1., 0., 0.);
    point_accel = calcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

    EXPECT_NEAR(1., point_accel[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_accel[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_accel[2], unit_test_utils::E_MINUS_14);
}

TEST_F(FixedBase3DoF, TestCalcPointRotatedBaseSimple)
{
    ref_body_id = 1;
    Q[0] = M_PI * 0.5;
    QDot[0] = 1.;
    point_position = Vector3d(1., 0., 0.);
    point_accel = calcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

    EXPECT_NEAR(0., point_accel[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(-1., point_accel[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_accel[2], unit_test_utils::E_MINUS_14);

    point_position = Vector3d(-1., 0., 0.);
    point_accel = calcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

    EXPECT_NEAR(0., point_accel[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(1., point_accel[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_accel[2], unit_test_utils::E_MINUS_14);
}

TEST_F(FixedBase3DoF, TestCalcPointRotatingBodyB)
{
    // rotating second joint, point at third body
    ref_body_id = 3;
    QDot[1] = 1.;
    point_position = Vector3d(1., 0., 0.);
    point_accel = calcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

    EXPECT_NEAR(-1., point_accel[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_accel[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_accel[2], unit_test_utils::E_MINUS_14);

    // move it a bit further up (acceleration should stay the same)
    point_position = Vector3d(1., 1., 0.);
    point_accel = calcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

    EXPECT_NEAR(-1., point_accel[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_accel[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_accel[2], unit_test_utils::E_MINUS_14);
}

TEST_F(FixedBase3DoF, TestCalcPointBodyOrigin)
{
    // rotating second joint, point at third body

    QDot[0] = 1.;

    ref_body_id = body_b_id;
    point_position = Vector3d(0., 0., 0.);
    point_accel = calcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

    EXPECT_NEAR(-1., point_accel[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_accel[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_accel[2], unit_test_utils::E_MINUS_14);
}

TEST_F(FixedBase3DoF, TestAccelerationLinearFuncOfQddot)
{
    // rotating second joint, point at third body

    QDot[0] = 1.1;
    QDot[1] = 1.3;
    QDot[2] = 1.5;

    ref_body_id = body_c_id;
    point_position = Vector3d(1., 1., 1.);

    VectorNd qddot_1 = VectorNd::Zero(model->dof_count);
    VectorNd qddot_2 = VectorNd::Zero(model->dof_count);
    VectorNd qddot_0 = VectorNd::Zero(model->dof_count);

    qddot_1[0] = 0.1;
    qddot_1[1] = 0.2;
    qddot_1[2] = 0.3;

    qddot_2[0] = 0.32;
    qddot_2[1] = -0.1;
    qddot_2[2] = 0.53;

    Vector3d acc_1 = calcPointAcceleration(*model, Q, QDot, qddot_1, ref_body_id, point_position);
    Vector3d acc_2 = calcPointAcceleration(*model, Q, QDot, qddot_2, ref_body_id, point_position);

    MatrixNd G = MatrixNd::Zero(3, model->dof_count);
    calcPointJacobian(*model, Q, ref_body_id, point_position, G, true);

    VectorNd net_acc = G * (qddot_1 - qddot_2);

    Vector3d acc_new = acc_1 - acc_2;

    EXPECT_NEAR(net_acc(0), acc_new(0), unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(net_acc(1), acc_new(1), unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(net_acc(2), acc_new(2), unit_test_utils::E_MINUS_14);
}

TEST_F(FloatingBase12DoF, TestAccelerationFloatingBaseWithUpdateKinematics)
{
    forwardDynamics(*model, Q, QDot, Tau, QDDot);

    Vector3d accel = calcPointAcceleration(*model, Q, QDot, QDDot, child_2_rot_x_id, Vector3d(0., 0., 0.), true);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., -9.81, 0.), accel, unit_test_utils::E_MINUS_14));
}

TEST_F(FloatingBase12DoF, TestAccelerationFloatingBaseWithoutUpdateKinematics)
{
    forwardDynamics(*model, Q, QDot, Tau, QDDot);

    Vector3d accel = calcPointAcceleration(*model, Q, QDot, QDDot, child_2_rot_x_id, Vector3d(0., 0., 0.), false);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 0., 0.), accel, unit_test_utils::E_MINUS_14));
}

TEST_F(FixedBase3DoF, TestCalcPointRotationFixedJoint)
{
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    unsigned int fixed_body_id = model->addBody(body_c_id, Xtrans(Vector3d(1., -1., 0.)), Joint(JointTypeFixed), fixed_body, "fixed_body");

    QDot[0] = 1.;
    point_position = Vector3d(0., 0., 0.);
    Vector3d point_accel_reference = calcPointAcceleration(*model, Q, QDot, QDDot, body_c_id, Vector3d(1., -1., 0.));

    point_accel = calcPointAcceleration(*model, Q, QDot, QDDot, fixed_body_id, point_position);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(point_accel_reference, point_accel, unit_test_utils::E_MINUS_14));
}

TEST_F(FixedBase3DoF, TestCalcPointRotationFixedJointRotatedTransform)
{
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    SpatialTransform fixed_transform = Xtrans(Vector3d(1., -1., 0.)) * Xrotz(M_PI * 0.5);
    unsigned int fixed_body_id = model->addBody(body_c_id, fixed_transform, Joint(JointTypeFixed), fixed_body, "fixed_body");

    QDot[0] = 1.;
    point_position = Vector3d(0., 0., 0.);

    Vector3d point_accel_reference = calcPointAcceleration(*model, Q, QDot, QDDot, body_c_id, Vector3d(1., 1., 0.));

    point_accel = calcPointAcceleration(*model, Q, QDot, QDDot, fixed_body_id, point_position);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(point_accel_reference, point_accel, unit_test_utils::E_MINUS_14));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
