// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>

#include <iostream>

#include "rdl_dynamics/Model.hpp"
#include "rdl_dynamics/Contacts.hpp"
#include "rdl_dynamics/Dynamics.hpp"
#include "rdl_dynamics/Kinematics.hpp"

#include "UnitTestUtils.hpp"

#include "Fixtures.hpp"
#include "Human36Fixture.hpp"

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

struct FixedBase6DoF9DoFFixture : public testing::Test
{
    FixedBase6DoF9DoFFixture()
    {
    }

    void SetUp()
    {
        model = new Model;

        model->gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

        /* 3 DoF (rot.) joint at base
         * 3 DoF (rot.) joint child origin
         *
         *          X Contact point (ref child)
         *          |
         *    Base  |
         *   / body |
         *  O-------*
         *           \
         *             Child body
         */

        // base body (3 DoF)
        base = Body(1., Vector3d(0.5, 0., 0.), Vector3d(1., 1., 1.));
        joint_rotzyx = Joint(SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.));
        base_id = model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rotzyx, base);

        // child body 1 (3 DoF)
        child = Body(1., Vector3d(0., 0.5, 0.), Vector3d(1., 1., 1.));
        child_id = model->addBody(base_id, Xtrans(Vector3d(0., 0., 0.)), joint_rotzyx, child);

        // child body (3 DoF)
        child_2 = Body(1., Vector3d(0., 0.5, 0.), Vector3d(1., 1., 1.));
        child_2_id = model->addBody(child_id, Xtrans(Vector3d(0., 0., 0.)), joint_rotzyx, child_2);

        Q = VectorNd::Constant(model->mBodies.size() - 1, 0.);
        QDot = VectorNd::Constant(model->mBodies.size() - 1, 0.);
        QDDot = VectorNd::Constant(model->mBodies.size() - 1, 0.);
        Tau = VectorNd::Constant(model->mBodies.size() - 1, 0.);

        contact_body_id = child_id;
        contact_point = Vector3d(0.5, 0.5, 0.);
        contact_normal = Vector3d(0., 1., 0.);
    }

    void TearDown()
    {
        EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(*model));
        delete model;
    }

    Model* model;

    unsigned int base_id, child_id, child_2_id;

    Body base, child, child_2;

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

//
// ForwardDynamicsContactsDirect
//
TEST_F(FixedBase6DoF9DoFFixture, TestForwardDynamicsContactsDirectSimple)
{
    Model model;
    model.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);
    Body base_body(1., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.));
    unsigned int base_body_id = model.addBody(0, SpatialTransform(),
                                              Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                                                    SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                                              base_body);

    VectorNd Q = VectorNd::Constant((size_t)model.dof_count, 0.);
    VectorNd QDot = VectorNd::Constant((size_t)model.dof_count, 0.);
    VectorNd QDDot = VectorNd::Constant((size_t)model.dof_count, 0.);
    VectorNd Tau = VectorNd::Constant((size_t)model.dof_count, 0.);

    Q[1] = 1.;
    QDot[0] = 1.;
    QDot[3] = -1.;

    unsigned int contact_body_id = base_body_id;
    Vector3d contact_point(0., -1., 0.);

    ConstraintSet constraint_set;

    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(1., 0., 0.), "ground_x");
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 1., 0.), "ground_y");
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 0., 1.), "ground_z");

    constraint_set.bind(model);

    forwardDynamicsContactsDirect(model, Q, QDot, Tau, constraint_set, QDDot);

    Vector3d point_acceleration = calcPointAcceleration(model, Q, QDot, QDDot, contact_body_id, contact_point);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 0., 0.), point_acceleration, unit_test_utils::E_MINUS_11));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));

    constraint_set.clear();

    EXPECT_TRUE(unit_test_utils::checkVectorNdEq(constraint_set.acceleration, VectorNd::Zero(constraint_set.acceleration.rows())));
    EXPECT_TRUE(unit_test_utils::checkVectorNdEq(constraint_set.a, VectorNd::Zero(constraint_set.a.rows())));
    EXPECT_TRUE(unit_test_utils::checkVectorNdEq(constraint_set.force, VectorNd::Zero(constraint_set.force.rows())));
    EXPECT_TRUE(unit_test_utils::checkVectorNdEq(constraint_set.impulse, VectorNd::Zero(constraint_set.force.rows())));
    EXPECT_TRUE(unit_test_utils::checkVectorNdEq(constraint_set.gamma, VectorNd::Zero(constraint_set.gamma.rows())));
    EXPECT_TRUE(unit_test_utils::checkVectorNdEq(constraint_set.b, VectorNd::Zero(constraint_set.b.rows())));
    EXPECT_TRUE(unit_test_utils::checkVectorNdEq(constraint_set.x, VectorNd::Zero(constraint_set.x.rows())));
    EXPECT_TRUE(unit_test_utils::checkVectorNdEq(constraint_set.QDDot_0, VectorNd::Zero(constraint_set.QDDot_0.rows())));
    EXPECT_TRUE(unit_test_utils::checkVectorNdEq(constraint_set.QDDot_t, VectorNd::Zero(constraint_set.QDDot_t.rows())));
    EXPECT_TRUE(unit_test_utils::checkVectorNdEq(constraint_set.d_u, VectorNd::Zero(constraint_set.d_u.rows())));

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEq(constraint_set.H, MatrixNd::Zero(constraint_set.H.rows(), constraint_set.H.cols())));
    EXPECT_TRUE(unit_test_utils::checkMatrixNdEq(constraint_set.C, MatrixNd::Zero(constraint_set.C.rows(), constraint_set.C.cols())));
    EXPECT_TRUE(unit_test_utils::checkMatrixNdEq(constraint_set.G, MatrixNd::Zero(constraint_set.G.rows(), constraint_set.G.cols())));
    EXPECT_TRUE(unit_test_utils::checkMatrixNdEq(constraint_set.A, MatrixNd::Zero(constraint_set.A.rows(), constraint_set.A.cols())));
    EXPECT_TRUE(unit_test_utils::checkMatrixNdEq(constraint_set.K, MatrixNd::Zero(constraint_set.K.rows(), constraint_set.K.cols())));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));

    unsigned int i;
    for (i = 0; i < constraint_set.f_t.size(); i++)
    {
        EXPECT_TRUE(unit_test_utils::checkVectorNdEq(constraint_set.f_t[i], VectorNd::Zero(constraint_set.f_t[i].rows())));
    }

    for (i = 0; i < constraint_set.f_ext_constraints.size(); i++)
    {
        EXPECT_TRUE(unit_test_utils::checkVectorNdEq(constraint_set.f_ext_constraints[i], VectorNd::Zero(constraint_set.f_ext_constraints[i].rows())));
    }

    for (i = 0; i < constraint_set.point_accel_0.size(); i++)
    {
        EXPECT_TRUE(unit_test_utils::checkVectorNdEq(constraint_set.point_accel_0[i], VectorNd::Zero(constraint_set.point_accel_0[i].rows())));
    }

    for (i = 0; i < constraint_set.d_pA.size(); i++)
    {
        EXPECT_TRUE(unit_test_utils::checkVectorNdEq(constraint_set.d_pA[i], VectorNd::Zero(constraint_set.d_pA[i].rows())));
    }

    for (i = 0; i < constraint_set.d_a.size(); i++)
    {
        EXPECT_TRUE(unit_test_utils::checkVectorNdEq(constraint_set.d_a[i], VectorNd::Zero(constraint_set.d_a[i].rows())));
    }
}

TEST_F(FixedBase6DoF9DoFFixture, TestForwardDynamicsContactsDirectMoving)
{
    Model model;
    model.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);
    Body base_body(1., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.));
    unsigned int base_body_id = model.addBody(0, SpatialTransform(),
                                              Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                                                    SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                                              base_body);

    VectorNd Q = VectorNd::Constant((size_t)model.dof_count, 0.);
    VectorNd QDot = VectorNd::Constant((size_t)model.dof_count, 0.);
    VectorNd QDDot = VectorNd::Constant((size_t)model.dof_count, 0.);
    VectorNd Tau = VectorNd::Constant((size_t)model.dof_count, 0.);

    Q[0] = 0.1;
    Q[1] = 0.2;
    Q[2] = 0.3;
    Q[3] = 0.4;
    Q[4] = 0.5;
    Q[5] = 0.6;
    QDot[0] = 1.1;
    QDot[1] = 1.2;
    QDot[2] = 1.3;
    QDot[3] = -1.4;
    QDot[4] = -1.5;
    QDot[5] = -1.6;

    unsigned int contact_body_id = base_body_id;
    Vector3d contact_point(0., -1., 0.);

    ConstraintSet constraint_set;

    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(1., 0., 0.), "ground_x");
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 1., 0.), "ground_y");
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 0., 1.), "ground_z");

    constraint_set.bind(model);

    forwardDynamicsContactsDirect(model, Q, QDot, Tau, constraint_set, QDDot);

    Vector3d point_acceleration = calcPointAcceleration(model, Q, QDot, QDDot, contact_body_id, contact_point);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 0., 0.), point_acceleration, unit_test_utils::E_MINUS_11));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));
}

//
// ForwardDynamicsContacts
//
TEST_F(FixedBase6DoF, ForwardDynamicsContactsSingleContact)
{
    contact_normal.set(0., 1., 0.);
    constraint_set.addConstraint(contact_body_id, contact_point, contact_normal);
    ConstraintSet constraint_set_lagrangian = constraint_set.Copy();

    constraint_set_lagrangian.bind(*model);
    constraint_set.bind(*model);

    Vector3d point_accel_lagrangian, point_accel_contacts;

    VectorNd QDDot_lagrangian = VectorNd::Constant(model->mBodies.size() - 1, 0.);
    VectorNd QDDot_contacts = VectorNd::Constant(model->mBodies.size() - 1, 0.);

    forwardDynamicsContactsDirect(*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);

    forwardDynamicsContactsKokkevis(*model, Q, QDot, Tau, constraint_set, QDDot_contacts);

    point_accel_lagrangian = calcPointAcceleration(*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point, true);
    point_accel_contacts = calcPointAcceleration(*model, Q, QDot, QDDot_contacts, contact_body_id, contact_point, true);

    EXPECT_NEAR(constraint_set_lagrangian.force[0], constraint_set.force[0], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(contact_normal.dot(point_accel_lagrangian), contact_normal.dot(point_accel_contacts), unit_test_utils::E_MINUS_11);
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(point_accel_lagrangian, point_accel_contacts, unit_test_utils::E_MINUS_11));
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_lagrangian, QDDot_contacts, unit_test_utils::E_MINUS_11));
}

TEST_F(FixedBase6DoF, ForwardDynamicsContactsSingleContactRotated)
{
    Q[0] = 0.6;
    Q[3] = M_PI * 0.6;
    Q[4] = 0.1;

    contact_normal.set(0., 1., 0.);

    constraint_set.addConstraint(contact_body_id, contact_point, contact_normal);
    ConstraintSet constraint_set_lagrangian = constraint_set.Copy();

    constraint_set_lagrangian.bind(*model);
    constraint_set.bind(*model);

    Vector3d point_accel_lagrangian, point_accel_contacts, point_accel_contacts_opt;

    VectorNd QDDot_lagrangian = VectorNd::Constant(model->mBodies.size() - 1, 0.);
    VectorNd QDDot_contacts = VectorNd::Constant(model->mBodies.size() - 1, 0.);
    VectorNd QDDot_contacts_opt = VectorNd::Constant(model->mBodies.size() - 1, 0.);

    forwardDynamicsContactsDirect(*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);
    forwardDynamicsContactsKokkevis(*model, Q, QDot, Tau, constraint_set, QDDot_contacts_opt);

    point_accel_lagrangian = calcPointAcceleration(*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point, true);
    point_accel_contacts_opt = calcPointAcceleration(*model, Q, QDot, QDDot_contacts_opt, contact_body_id, contact_point, true);

    EXPECT_NEAR(constraint_set_lagrangian.force[0], constraint_set.force[0], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(contact_normal.dot(point_accel_lagrangian), contact_normal.dot(point_accel_contacts_opt), unit_test_utils::E_MINUS_11);
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(point_accel_lagrangian, point_accel_contacts_opt, unit_test_utils::E_MINUS_11));
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_lagrangian, QDDot_contacts_opt, unit_test_utils::E_MINUS_11));
}

// Similiar to the previous test, this test compares the results of
//   - ForwardDynamicsContactsDirect
//   - ForwardDynamcsContactsOpt
// for the example model in FixedBase6DoF and a moving state (i.e. a
// nonzero QDot)
//
TEST_F(FixedBase6DoF, ForwardDynamicsContactsSingleContactRotatedMoving)
{
    Q[0] = 0.6;
    Q[3] = M_PI * 0.6;
    Q[4] = 0.1;

    QDot[0] = -0.3;
    QDot[1] = 0.1;
    QDot[2] = -0.5;
    QDot[3] = 0.8;

    contact_normal.set(0., 1., 0.);
    constraint_set.addConstraint(contact_body_id, contact_point, contact_normal);
    ConstraintSet constraint_set_lagrangian = constraint_set.Copy();

    constraint_set_lagrangian.bind(*model);
    constraint_set.bind(*model);

    Vector3d point_accel_lagrangian, point_accel_contacts;

    VectorNd QDDot_lagrangian = VectorNd::Constant(model->mBodies.size() - 1, 0.);
    VectorNd QDDot_contacts = VectorNd::Constant(model->mBodies.size() - 1, 0.);

    forwardDynamicsContactsDirect(*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);

    forwardDynamicsContactsKokkevis(*model, Q, QDot, Tau, constraint_set, QDDot_contacts);

    point_accel_lagrangian = calcPointAcceleration(*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point, true);
    point_accel_contacts = calcPointAcceleration(*model, Q, QDot, QDDot_contacts, contact_body_id, contact_point, true);

    // check whether FDContactsLagrangian and FDContactsOld match
    EXPECT_NEAR(constraint_set_lagrangian.force[0], constraint_set.force[0], unit_test_utils::E_MINUS_11);

    EXPECT_NEAR(contact_normal.dot(point_accel_lagrangian), contact_normal.dot(point_accel_contacts), unit_test_utils::E_MINUS_11);
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(point_accel_lagrangian, point_accel_contacts, unit_test_utils::E_MINUS_11));
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_lagrangian, QDDot_contacts, unit_test_utils::E_MINUS_11));
}

TEST_F(FixedBase6DoF, ForwardDynamicsContactsOptDoubleContact)
{
    ConstraintSet constraint_set_lagrangian;

    constraint_set.addConstraint(contact_body_id, Vector3d(1., 0., 0.), contact_normal);
    constraint_set.addConstraint(contact_body_id, Vector3d(0., 1., 0.), contact_normal);

    constraint_set_lagrangian = constraint_set.Copy();
    constraint_set_lagrangian.bind(*model);
    constraint_set.bind(*model);

    Vector3d point_accel_lagrangian, point_accel_contacts;

    VectorNd QDDot_lagrangian = VectorNd::Constant(model->mBodies.size() - 1, 0.);
    VectorNd QDDot_contacts = VectorNd::Constant(model->mBodies.size() - 1, 0.);

    forwardDynamicsContactsDirect(*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);
    forwardDynamicsContactsKokkevis(*model, Q, QDot, Tau, constraint_set, QDDot_contacts);

    point_accel_lagrangian = calcPointAcceleration(*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point, true);
    point_accel_contacts = calcPointAcceleration(*model, Q, QDot, QDDot_contacts, contact_body_id, contact_point, true);

    // check whether FDContactsLagrangian and FDContacts match
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(constraint_set_lagrangian.force, constraint_set.force, unit_test_utils::E_MINUS_11));
    // check whether the point accelerations match
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(point_accel_lagrangian, point_accel_contacts, unit_test_utils::E_MINUS_11));
    // check whether the generalized accelerations match
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_lagrangian, QDDot_contacts, unit_test_utils::E_MINUS_11));
}

TEST_F(FixedBase6DoF, ForwardDynamicsContactsOptDoubleContactRepeated)
{
    // makes sure that all variables in the constraint set gets reset
    // properly when making repeated calls to ForwardDynamicsContacts.
    ConstraintSet constraint_set_lagrangian;

    constraint_set.addConstraint(contact_body_id, Vector3d(1., 0., 0.), contact_normal);
    constraint_set.addConstraint(contact_body_id, Vector3d(0., 1., 0.), contact_normal);

    constraint_set_lagrangian = constraint_set.Copy();
    constraint_set_lagrangian.bind(*model);
    constraint_set.bind(*model);

    Vector3d point_accel_lagrangian, point_accel_contacts;

    VectorNd QDDot_lagrangian = VectorNd::Constant(model->mBodies.size() - 1, 0.);
    VectorNd QDDot_contacts = VectorNd::Constant(model->mBodies.size() - 1, 0.);

    forwardDynamicsContactsDirect(*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);
    // Call ForwardDynamicsContacts multiple times such that old values might
    // be re-used and thus cause erroneus values.
    forwardDynamicsContactsKokkevis(*model, Q, QDot, Tau, constraint_set, QDDot_contacts);
    forwardDynamicsContactsKokkevis(*model, Q, QDot, Tau, constraint_set, QDDot_contacts);
    forwardDynamicsContactsKokkevis(*model, Q, QDot, Tau, constraint_set, QDDot_contacts);

    point_accel_lagrangian = calcPointAcceleration(*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point, true);
    point_accel_contacts = calcPointAcceleration(*model, Q, QDot, QDDot_contacts, contact_body_id, contact_point, true);

    // check whether FDContactsLagrangian and FDContacts match
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(constraint_set_lagrangian.force, constraint_set.force, unit_test_utils::E_MINUS_11));
    // check whether the point accelerations match
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(point_accel_lagrangian, point_accel_contacts, unit_test_utils::E_MINUS_11));
    // check whether the generalized accelerations match
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_lagrangian, QDDot_contacts, unit_test_utils::E_MINUS_11));
}

TEST_F(FixedBase6DoF, ForwardDynamicsContactsOptMultipleContact)
{
    ConstraintSet constraint_set_lagrangian;

    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(1., 0., 0.));
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 1., 0.));

    constraint_set_lagrangian = constraint_set.Copy();
    constraint_set_lagrangian.bind(*model);
    constraint_set.bind(*model);

    // we rotate the joints so that we have full mobility at the contact
    // point:
    //
    //  O       X (contact point)
    //   \     /
    //    \   /
    //     \ /
    //      *
    //

    Q[0] = M_PI * 0.25;
    Q[1] = 0.2;
    Q[3] = M_PI * 0.5;

    VectorNd QDDot_lagrangian = VectorNd::Constant(model->mBodies.size() - 1, 0.);
    VectorNd QDDot_contacts = VectorNd::Constant(model->mBodies.size() - 1, 0.);

    forwardDynamicsContactsDirect(*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);
    forwardDynamicsContactsKokkevis(*model, Q, QDot, Tau, constraint_set, QDDot_contacts);

    Vector3d point_accel_c = calcPointAcceleration(*model, Q, QDot, QDDot, contact_body_id, contact_point);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_lagrangian, QDDot_contacts, unit_test_utils::E_MINUS_11));
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(constraint_set_lagrangian.force, constraint_set.force, unit_test_utils::E_MINUS_11));

    EXPECT_NEAR(0., point_accel_c[0], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_c[1], unit_test_utils::E_MINUS_11);
}

TEST_F(FixedBase6DoF9DoFFixture, ForwardDynamicsContactsOptMultipleContactsMultipleBodiesMoving)
{
    ConstraintSet constraint_set_lagrangian;

    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(1., 0., 0.));
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 1., 0.));
    constraint_set.addConstraint(child_2_id, contact_point, Vector3d(0., 1., 0.));

    constraint_set_lagrangian = constraint_set.Copy();
    constraint_set_lagrangian.bind(*model);
    constraint_set.bind(*model);

    Q[0] = 0.1;
    Q[1] = -0.1;
    Q[2] = 0.1;
    Q[3] = -0.1;
    Q[4] = -0.1;
    Q[5] = 0.1;

    QDot[0] = 1.;
    QDot[1] = -1.;
    QDot[2] = 1;
    QDot[3] = -1.5;
    QDot[4] = 1.5;
    QDot[5] = -1.5;

    VectorNd QDDot_lagrangian = VectorNd::Constant(model->mBodies.size() - 1, 0.);

    forwardDynamicsContactsKokkevis(*model, Q, QDot, Tau, constraint_set, QDDot);

    Vector3d point_accel_c, point_accel_2_c;

    point_accel_c = calcPointAcceleration(*model, Q, QDot, QDDot, contact_body_id, contact_point);
    point_accel_2_c = calcPointAcceleration(*model, Q, QDot, QDDot, child_2_id, contact_point);

    forwardDynamicsContactsDirect(*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(constraint_set_lagrangian.force, constraint_set.force, unit_test_utils::E_MINUS_11));

    EXPECT_NEAR(0., point_accel_c[0], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_c[1], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_2_c[1], unit_test_utils::E_MINUS_11);

    point_accel_c = calcPointAcceleration(*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point);
    point_accel_2_c = calcPointAcceleration(*model, Q, QDot, QDDot_lagrangian, child_2_id, contact_point);

    EXPECT_NEAR(0., point_accel_c[0], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_c[1], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_2_c[1], unit_test_utils::E_MINUS_11);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_lagrangian, QDDot, unit_test_utils::E_MINUS_11));
}

TEST_F(FixedBase6DoF9DoFFixture, ForwardDynamicsContactsOptMultipleContactsMultipleBodiesMovingLinearSolverPartialPivLU)
{
    ConstraintSet constraint_set_lagrangian;

    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(1., 0., 0.));
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 1., 0.));
    constraint_set.addConstraint(child_2_id, contact_point, Vector3d(0., 1., 0.));

    constraint_set_lagrangian = constraint_set.Copy();
    constraint_set_lagrangian.linear_solver = RobotDynamics::Math::LinearSolver::LinearSolverPartialPivLU;
    constraint_set_lagrangian.bind(*model);
    constraint_set.linear_solver = RobotDynamics::Math::LinearSolver::LinearSolverPartialPivLU;
    constraint_set.bind(*model);

    Q[0] = 0.1;
    Q[1] = -0.1;
    Q[2] = 0.1;
    Q[3] = -0.1;
    Q[4] = -0.1;
    Q[5] = 0.1;

    QDot[0] = 1.;
    QDot[1] = -1.;
    QDot[2] = 1;
    QDot[3] = -1.5;
    QDot[4] = 1.5;
    QDot[5] = -1.5;

    VectorNd QDDot_lagrangian = VectorNd::Constant(model->mBodies.size() - 1, 0.);

    forwardDynamicsContactsKokkevis(*model, Q, QDot, Tau, constraint_set, QDDot);

    Vector3d point_accel_c, point_accel_2_c;

    point_accel_c = calcPointAcceleration(*model, Q, QDot, QDDot, contact_body_id, contact_point);
    point_accel_2_c = calcPointAcceleration(*model, Q, QDot, QDDot, child_2_id, contact_point);

    forwardDynamicsContactsDirect(*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(constraint_set_lagrangian.force, constraint_set.force, unit_test_utils::E_MINUS_11));

    EXPECT_NEAR(0., point_accel_c[0], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_c[1], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_2_c[1], unit_test_utils::E_MINUS_11);

    point_accel_c = calcPointAcceleration(*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point);
    point_accel_2_c = calcPointAcceleration(*model, Q, QDot, QDDot_lagrangian, child_2_id, contact_point);

    EXPECT_NEAR(0., point_accel_c[0], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_c[1], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_2_c[1], unit_test_utils::E_MINUS_11);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_lagrangian, QDDot, unit_test_utils::E_MINUS_11));
}

TEST_F(FixedBase6DoF9DoFFixture, ForwardDynamicsContactsOptMultipleContactsMultipleBodiesMovingLinearSolverHouseholderQR)
{
    ConstraintSet constraint_set_lagrangian;

    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(1., 0., 0.));
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 1., 0.));
    constraint_set.addConstraint(child_2_id, contact_point, Vector3d(0., 1., 0.));

    constraint_set_lagrangian = constraint_set.Copy();
    constraint_set_lagrangian.linear_solver = RobotDynamics::Math::LinearSolver::LinearSolverHouseholderQR;
    constraint_set_lagrangian.bind(*model);
    constraint_set.linear_solver = RobotDynamics::Math::LinearSolver::LinearSolverHouseholderQR;
    constraint_set.bind(*model);

    Q[0] = 0.1;
    Q[1] = -0.1;
    Q[2] = 0.1;
    Q[3] = -0.1;
    Q[4] = -0.1;
    Q[5] = 0.1;

    QDot[0] = 1.;
    QDot[1] = -1.;
    QDot[2] = 1;
    QDot[3] = -1.5;
    QDot[4] = 1.5;
    QDot[5] = -1.5;

    VectorNd QDDot_lagrangian = VectorNd::Constant(model->mBodies.size() - 1, 0.);

    forwardDynamicsContactsKokkevis(*model, Q, QDot, Tau, constraint_set, QDDot);

    Vector3d point_accel_c, point_accel_2_c;

    point_accel_c = calcPointAcceleration(*model, Q, QDot, QDDot, contact_body_id, contact_point);
    point_accel_2_c = calcPointAcceleration(*model, Q, QDot, QDDot, child_2_id, contact_point);

    forwardDynamicsContactsDirect(*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(constraint_set_lagrangian.force, constraint_set.force, unit_test_utils::E_MINUS_11));

    EXPECT_NEAR(0., point_accel_c[0], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_c[1], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_2_c[1], unit_test_utils::E_MINUS_11);

    point_accel_c = calcPointAcceleration(*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point);
    point_accel_2_c = calcPointAcceleration(*model, Q, QDot, QDDot_lagrangian, child_2_id, contact_point);

    EXPECT_NEAR(0., point_accel_c[0], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_c[1], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_2_c[1], unit_test_utils::E_MINUS_11);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_lagrangian, QDDot, unit_test_utils::E_MINUS_11));
}

TEST_F(FixedBase6DoF9DoFFixture, ForwardDynamicsContactsOptMultipleContactsMultipleBodiesMovingAlternate)
{
    ConstraintSet constraint_set_lagrangian;

    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(1., 0., 0.));
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 1., 0.));
    constraint_set.addConstraint(child_2_id, contact_point, Vector3d(0., 1., 0.));

    constraint_set_lagrangian = constraint_set.Copy();
    constraint_set_lagrangian.bind(*model);
    constraint_set.bind(*model);

    Q[0] = 0.1;
    Q[1] = -0.3;
    Q[2] = 0.15;
    Q[3] = -0.21;
    Q[4] = -0.81;
    Q[5] = 0.11;
    Q[6] = 0.31;
    Q[7] = -0.91;
    Q[8] = 0.61;

    QDot[0] = 1.3;
    QDot[1] = -1.7;
    QDot[2] = 3;
    QDot[3] = -2.5;
    QDot[4] = 1.5;
    QDot[5] = -5.5;
    QDot[6] = 2.5;
    QDot[7] = -1.5;
    QDot[8] = -3.5;

    VectorNd QDDot_lagrangian = VectorNd::Constant(model->mBodies.size() - 1, 0.);

    forwardDynamicsContactsKokkevis(*model, Q, QDot, Tau, constraint_set, QDDot);

    Vector3d point_accel_c, point_accel_2_c;

    point_accel_c = calcPointAcceleration(*model, Q, QDot, QDDot, contact_body_id, contact_point);
    point_accel_2_c = calcPointAcceleration(*model, Q, QDot, QDDot, child_2_id, contact_point);

    forwardDynamicsContactsDirect(*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(constraint_set_lagrangian.force, constraint_set.force, unit_test_utils::E_MINUS_11));

    EXPECT_NEAR(0., point_accel_c[0], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_c[1], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_2_c[1], unit_test_utils::E_MINUS_11);

    point_accel_c = calcPointAcceleration(*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point);
    point_accel_2_c = calcPointAcceleration(*model, Q, QDot, QDDot_lagrangian, child_2_id, contact_point);

    EXPECT_NEAR(0., point_accel_c[0], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_c[1], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_2_c[1], unit_test_utils::E_MINUS_11);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_lagrangian, QDDot, unit_test_utils::E_MINUS_11));
}

TEST_F(FixedBase6DoF12DoFFloatingBase, ForwardDynamicsContactsMultipleContactsFloatingBase)
{
    ConstraintSet constraint_set_lagrangian;

    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(1., 0., 0.));
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 1., 0.));
    constraint_set.addConstraint(child_2_id, contact_point, Vector3d(0., 1., 0.));

    constraint_set_lagrangian = constraint_set.Copy();
    constraint_set_lagrangian.bind(*model);
    constraint_set.bind(*model);

    VectorNd QDDot_lagrangian = VectorNd::Constant(model->dof_count, 0.);

    Q[0] = 0.1;
    Q[1] = -0.3;
    Q[2] = 0.15;
    Q[3] = -0.21;
    Q[4] = -0.81;
    Q[5] = 0.11;
    Q[6] = 0.31;
    Q[7] = -0.91;
    Q[8] = 0.61;

    QDot[0] = 1.3;
    QDot[1] = -1.7;
    QDot[2] = 3;
    QDot[3] = -2.5;
    QDot[4] = 1.5;
    QDot[5] = -5.5;
    QDot[6] = 2.5;
    QDot[7] = -1.5;
    QDot[8] = -3.5;

    forwardDynamicsContactsKokkevis(*model, Q, QDot, Tau, constraint_set, QDDot);

    Vector3d point_accel_c, point_accel_2_c;

    point_accel_c = calcPointAcceleration(*model, Q, QDot, QDDot, contact_body_id, contact_point);
    point_accel_2_c = calcPointAcceleration(*model, Q, QDot, QDDot, child_2_id, contact_point);

    forwardDynamicsContactsDirect(*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(constraint_set_lagrangian.force, constraint_set.force, unit_test_utils::E_MINUS_11));

    EXPECT_NEAR(0., point_accel_c[0], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_c[1], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_2_c[1], unit_test_utils::E_MINUS_11);

    point_accel_c = calcPointAcceleration(*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point);
    point_accel_2_c = calcPointAcceleration(*model, Q, QDot, QDDot_lagrangian, child_2_id, contact_point);

    EXPECT_NEAR(0., point_accel_c[0], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_c[1], unit_test_utils::E_MINUS_11);
    EXPECT_NEAR(0., point_accel_2_c[1], unit_test_utils::E_MINUS_11);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_lagrangian, QDDot, unit_test_utils::E_MINUS_11));
}

TEST_F(Human36, ForwardDynamicsContactsFixedBody)
{
    VectorNd qddot_lagrangian(VectorNd::Zero(qddot.size()));
    VectorNd qddot_sparse(VectorNd::Zero(qddot.size()));

    randomizeStates();

    ConstraintSet constraint_upper_trunk;
    constraint_upper_trunk.addConstraint(body_id_3dof[BodyUpperTrunk], Vector3d(1.1, 2.2, 3.3), Vector3d(1., 0., 0.));
    constraint_upper_trunk.bind(*model_3dof);

    forwardDynamicsContactsDirect(*model_3dof, q, qdot, tau, constraint_upper_trunk, qddot_lagrangian);
    forwardDynamicsContactsRangeSpaceSparse(*model_3dof, q, qdot, tau, constraint_upper_trunk, qddot_sparse);
    forwardDynamicsContactsKokkevis(*model_3dof, q, qdot, tau, constraint_upper_trunk, qddot);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot, unit_test_utils::E_MINUS_10 * qddot_lagrangian.norm()));
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_sparse, unit_test_utils::E_MINUS_10 * qddot_lagrangian.norm()));
}

TEST_F(Human36, ForwardDynamicsContactsImpulses)
{
    VectorNd qddot_lagrangian(VectorNd::Zero(qddot.size()));

    randomizeStates();

    Vector3d heel_point(-0.03, 0., -0.03);

    ConstraintSet constraint_feet;
    constraint_feet.addConstraint(body_id_3dof[BodyFootLeft], heel_point, Vector3d(1., 0., 0.));
    constraint_feet.addConstraint(body_id_3dof[BodyFootLeft], heel_point, Vector3d(0., 1., 0.));
    constraint_feet.addConstraint(body_id_3dof[BodyFootLeft], heel_point, Vector3d(0., 0., 1.));
    constraint_feet.addConstraint(body_id_3dof[BodyFootRight], heel_point, Vector3d(1., 0., 0.));
    constraint_feet.addConstraint(body_id_3dof[BodyFootRight], heel_point, Vector3d(0., 1., 0.));
    constraint_feet.addConstraint(body_id_3dof[BodyFootRight], heel_point, Vector3d(0., 0., 1.));
    constraint_feet.bind(*model_3dof);

    VectorNd qdotplus(VectorNd::Zero(qdot.size()));

    computeContactImpulsesDirect(*model_3dof, q, qdot, constraint_feet, qdotplus);

    Vector3d heel_left_velocity = calcPointVelocity(*model_3dof, q, qdotplus, body_id_3dof[BodyFootLeft], heel_point);
    Vector3d heel_right_velocity = calcPointVelocity(*model_3dof, q, qdotplus, body_id_3dof[BodyFootRight], heel_point);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 0., 0.), heel_left_velocity, unit_test_utils::E_MINUS_11));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 0., 0.), heel_right_velocity, unit_test_utils::E_MINUS_11));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
