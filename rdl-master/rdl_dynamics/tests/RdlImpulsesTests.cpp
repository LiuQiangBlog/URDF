// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>
#include <iostream>

#include "UnitTestUtils.hpp"

#include "rdl_dynamics/Model.hpp"
#include "rdl_dynamics/Contacts.hpp"
#include "rdl_dynamics/Dynamics.hpp"
#include "rdl_dynamics/Kinematics.hpp"

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

struct RdlImpulsesFixture : public testing::Test
{
    RdlImpulsesFixture()
    {
    }

    void SetUp()
    {
        model = new Model;

        model->gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

        // base body
        base = Body(1., Vector3d(0., 1., 0.), Vector3d(1., 1., 1.));
        joint_rotzyx = Joint(SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.));
        base_id = model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rotzyx, base);

        // child body (3 DoF)
        child = Body(1., Vector3d(0., 1., 0.), Vector3d(1., 1., 1.));
        child_id = model->addBody(base_id, Xtrans(Vector3d(1., 0., 0.)), joint_rotzyx, child);

        Q = VectorNd::Zero(model->dof_count);
        QDot = VectorNd::Zero(model->dof_count);
        QDDot = VectorNd::Zero(model->dof_count);
        Tau = VectorNd::Zero(model->dof_count);

        contact_body_id = child_id;
        contact_point = Vector3d(0., 1., 0.);
        contact_normal = Vector3d(0., 1., 0.);
    }

    void TearDown()
    {
        delete model;
    }

    Model* model;

    unsigned int base_id, child_id;
    Body base, child;
    Joint joint_rotzyx;

    VectorNd Q;
    VectorNd QDot;
    VectorNd QDDot;
    VectorNd Tau;

    unsigned int contact_body_id;
    Vector3d contact_point;
    Vector3d contact_normal;
    ConstraintSet constraint_set;
};

TEST_F(RdlImpulsesFixture, TestContactImpulse)
{
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(1., 0., 0.), NULL, 0.);
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 1., 0.), NULL, 0.);
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 0., 1.), NULL, 0.);

    constraint_set.bind(*model);

    constraint_set.v_plus[0] = 0.;
    constraint_set.v_plus[1] = 0.;
    constraint_set.v_plus[2] = 0.;

    QDot[0] = 0.1;
    QDot[1] = -0.2;
    QDot[2] = 0.1;

    Vector3d point_velocity;
    point_velocity = calcPointVelocity(*model, Q, QDot, contact_body_id, contact_point, true);

    VectorNd qdot_post(QDot.size());
    computeContactImpulsesDirect(*model, Q, QDot, constraint_set, qdot_post);

    point_velocity = calcPointVelocity(*model, Q, qdot_post, contact_body_id, contact_point, true);

    // cout << "Point Velocity = " << point_velocity << endl;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 0., 0.), point_velocity, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlImpulsesFixture, TestContactImpulseRotated)
{
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(1., 0., 0.), NULL, 0.);
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 1., 0.), NULL, 0.);
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 0., 1.), NULL, 0.);

    constraint_set.bind(*model);

    constraint_set.v_plus[0] = 0.;
    constraint_set.v_plus[1] = 0.;
    constraint_set.v_plus[2] = 0.;

    Q[0] = 0.2;
    Q[1] = -0.5;
    Q[2] = 0.1;
    Q[3] = -0.4;
    Q[4] = -0.1;
    Q[5] = 0.4;

    QDot[0] = 0.1;
    QDot[1] = -0.2;
    QDot[2] = 0.1;

    Vector3d point_velocity;
    point_velocity = calcPointVelocity(*model, Q, QDot, contact_body_id, contact_point, true);

    VectorNd qdot_post(QDot.size());
    computeContactImpulsesDirect(*model, Q, QDot, constraint_set, qdot_post);

    point_velocity = calcPointVelocity(*model, Q, qdot_post, contact_body_id, contact_point, true);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 0., 0.), point_velocity, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlImpulsesFixture, TestContactImpulseRotatedCollisionVelocity)
{
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(1., 0., 0.), NULL, 1.);
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 1., 0.), NULL, 2.);
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 0., 1.), NULL, 3.);

    constraint_set.bind(*model);

    constraint_set.v_plus[0] = 1.;
    constraint_set.v_plus[1] = 2.;
    constraint_set.v_plus[2] = 3.;

    Q[0] = 0.2;
    Q[1] = -0.5;
    Q[2] = 0.1;
    Q[3] = -0.4;
    Q[4] = -0.1;
    Q[5] = 0.4;

    QDot[0] = 0.1;
    QDot[1] = -0.2;
    QDot[2] = 0.1;

    Vector3d point_velocity;
    point_velocity = calcPointVelocity(*model, Q, QDot, contact_body_id, contact_point, true);

    VectorNd qdot_post(QDot.size());
    computeContactImpulsesDirect(*model, Q, QDot, constraint_set, qdot_post);

    point_velocity = calcPointVelocity(*model, Q, qdot_post, contact_body_id, contact_point, true);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 2., 3.), point_velocity, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlImpulsesFixture, TestContactImpulseRangeSpaceSparse)
{
    Q[0] = 0.2;
    Q[1] = -0.5;
    Q[2] = 0.1;
    Q[3] = -0.4;
    Q[4] = -0.1;
    Q[5] = 0.4;

    QDot[0] = 0.1;
    QDot[1] = -0.2;
    QDot[2] = 0.1;

    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(1., 0., 0.), NULL, 1.);
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 1., 0.), NULL, 2.);
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 0., 1.), NULL, 3.);

    constraint_set.bind(*model);

    constraint_set.v_plus[0] = 1.;
    constraint_set.v_plus[1] = 2.;
    constraint_set.v_plus[2] = 3.;

    ConstraintSet constraint_set_rangespace;
    constraint_set_rangespace = constraint_set.Copy();
    constraint_set_rangespace.bind(*model);

    VectorNd qdot_post_direct(QDot.size());
    computeContactImpulsesDirect(*model, Q, QDot, constraint_set, qdot_post_direct);

    VectorNd qdot_post_rangespace(QDot.size());
    computeContactImpulsesRangeSpaceSparse(*model, Q, QDot, constraint_set_rangespace, qdot_post_rangespace);

    Vector3d point_velocity_rangespace = calcPointVelocity(*model, Q, qdot_post_rangespace, contact_body_id, contact_point, true);
    Vector3d point_velocity_direct = calcPointVelocity(*model, Q, qdot_post_direct, contact_body_id, contact_point, true);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qdot_post_direct, qdot_post_rangespace, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 2., 3.), point_velocity_rangespace, unit_test_utils::E_MINUS_14));
    // check this
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(point_velocity_direct, point_velocity_rangespace, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlImpulsesFixture, TestContactImpulseNullSpace)
{
    Q[0] = 0.2;
    Q[1] = -0.5;
    Q[2] = 0.1;
    Q[3] = -0.4;
    Q[4] = -0.1;
    Q[5] = 0.4;

    QDot[0] = 0.1;
    QDot[1] = -0.2;
    QDot[2] = 0.1;

    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(1., 0., 0.), NULL, 1.);
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 1., 0.), NULL, 2.);
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 0., 1.), NULL, 3.);

    constraint_set.bind(*model);

    constraint_set.v_plus[0] = 1.;
    constraint_set.v_plus[1] = 2.;
    constraint_set.v_plus[2] = 3.;

    ConstraintSet constraint_set_nullspace;
    constraint_set_nullspace = constraint_set.Copy();
    constraint_set_nullspace.bind(*model);

    VectorNd qdot_post_direct(QDot.size());
    computeContactImpulsesDirect(*model, Q, QDot, constraint_set, qdot_post_direct);

    VectorNd qdot_post_nullspace(QDot.size());
    computeContactImpulsesNullSpace(*model, Q, QDot, constraint_set, qdot_post_nullspace);

    Vector3d point_velocity_direct = calcPointVelocity(*model, Q, qdot_post_direct, contact_body_id, contact_point, true);
    Vector3d point_velocity_nullspace = calcPointVelocity(*model, Q, qdot_post_nullspace, contact_body_id, contact_point, true);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qdot_post_direct, qdot_post_nullspace, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 2., 3.), point_velocity_nullspace, unit_test_utils::E_MINUS_14));
    // check this
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(point_velocity_direct, point_velocity_nullspace, unit_test_utils::E_MINUS_14));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
