// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>

#include <iostream>

#include "UnitTestUtils.hpp"
#include "Fixtures.hpp"
#include "rdl_dynamics/Kinematics.hpp"
#include "rdl_dynamics/Dynamics.hpp"
#include "rdl_dynamics/rdl_utils.hpp"

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

const double TEST_PREC = 1.0e-12;

struct RdlCustomEulerZYXJoint : public CustomJoint
{
    RdlCustomEulerZYXJoint()
    {
        mDoFCount = 3;
        S = MatrixNd::Zero(6, 3);
        S_o = MatrixNd::Zero(6, 3);
    };

    virtual void jcalc(Model& model, unsigned int joint_id, const Math::VectorNd& q, const Math::VectorNd& qdot)
    {
        double q0 = q[model.mJoints[joint_id].q_index];
        double q1 = q[model.mJoints[joint_id].q_index + 1];
        double q2 = q[model.mJoints[joint_id].q_index + 2];

        double s0 = sin(q0);
        double c0 = cos(q0);
        double s1 = sin(q1);
        double c1 = cos(q1);
        double s2 = sin(q2);
        double c2 = cos(q2);

        model.X_J[joint_id].E =
            Matrix3d(c0 * c1, s0 * c1, -s1, c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2, c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2);
        model.bodyFrames[joint_id]->setTransformFromParent(SpatialTransform(Matrix3d(c0 * c1, s0 * c1, -s1, c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2,
                                                                                     c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2),
                                                                            Vector3d(0., 0., 0.)) *
                                                           model.X_T[joint_id]);

        S(0, 0) = -s1;
        S(0, 2) = 1.;

        S(1, 0) = c1 * s2;
        S(1, 1) = c2;

        S(2, 0) = c1 * c2;
        S(2, 1) = -s2;

        double qdot0 = qdot[model.mJoints[joint_id].q_index];
        double qdot1 = qdot[model.mJoints[joint_id].q_index + 1];
        double qdot2 = qdot[model.mJoints[joint_id].q_index + 2];

        Vector3d qdotv(qdot0, qdot1, qdot2);
        model.v_J[joint_id].set(S * qdotv);

        S_o(0, 0) = -c1 * qdot1;
        S_o(1, 0) = -s1 * s2 * qdot1 + c1 * c2 * qdot2;
        S_o(1, 1) = -s2 * qdot2;
        S_o(2, 0) = -s1 * c2 * qdot1 - c1 * s2 * qdot2;
        S_o(2, 1) = -c2 * qdot2;

        model.c_J[joint_id] = S_o * qdotv;  //.set(-c1 * qdot0 * qdot1, -s1 * s2 * qdot0 * qdot1 + c1 * c2 * qdot0 * qdot2 - s2 * qdot1 * qdot2, -s1 * c2 * qdot0 * qdot1
                                            //- c1 * s2 * qdot0 * qdot2 - c2 * qdot1 * qdot2, 0., 0., 0.);

        model.bodyFrames[joint_id]->update();
    }

    virtual void jcalc_X_lambda_S(Model& model, unsigned int joint_id, const Math::VectorNd& q)
    {
        double q0 = q[model.mJoints[joint_id].q_index];
        double q1 = q[model.mJoints[joint_id].q_index + 1];
        double q2 = q[model.mJoints[joint_id].q_index + 2];

        double s0 = sin(q0);
        double c0 = cos(q0);
        double s1 = sin(q1);
        double c1 = cos(q1);
        double s2 = sin(q2);
        double c2 = cos(q2);

        model.bodyFrames[joint_id]->setTransformFromParent(SpatialTransform(Matrix3d(c0 * c1, s0 * c1, -s1, c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2,
                                                                                     c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2),
                                                                            Vector3d(0., 0., 0.)) *
                                                           model.X_T[joint_id]);
        model.bodyFrames[joint_id]->update();

        S.setZero();
        S(0, 0) = -s1;
        S(0, 2) = 1.;
        S(1, 0) = c1 * s2;
        S(1, 1) = c2;
        S(2, 0) = c1 * c2;
        S(2, 1) = -s2;
    }
};

struct RdlCustomJointFixture : public testing::Test
{
    RdlCustomJointFixture()
    {
    }

    void SetUp()
    {
        custom_joint = new RdlCustomEulerZYXJoint();

        Matrix3d inertia = Matrix3d::Identity(3, 3);
        body = Body(1., Vector3d(1.1, 1.2, 1.3), inertia);
        reference_body_id = reference_model.addBody(0, SpatialTransform(), Joint(JointTypeEulerZYX), body);
        custom_body_id = custom_model.addBodyCustomJoint(0, SpatialTransform(), custom_joint, body);

        q = VectorNd::Zero(reference_model.q_size);
        qdot = VectorNd::Zero(reference_model.qdot_size);
        qddot = VectorNd::Zero(reference_model.qdot_size);
        tau = VectorNd::Zero(reference_model.qdot_size);
    }

    void TearDown()
    {
        EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(reference_model));
        EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(custom_model));
        delete custom_joint;
    }

    Model reference_model;
    Model custom_model;

    Body body;
    CustomJoint* custom_joint;

    unsigned int reference_body_id;
    unsigned int custom_body_id;

    VectorNd q;
    VectorNd qdot;
    VectorNd qddot;
    VectorNd tau;
};

TEST_F(RdlCustomJointFixture, updateKinematics)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
    }

    updateKinematics(reference_model, q, qdot, qddot);
    updateKinematics(custom_model, q, qdot, qddot);

    EXPECT_TRUE(unit_test_utils::checkMatrix3dEq(reference_model.bodyFrames[reference_body_id]->getTransformToRoot().E,
                                                 custom_model.bodyFrames[custom_body_id]->getTransformToRoot().E));

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEq(reference_model.v[reference_body_id], custom_model.v[custom_body_id]));

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEq(reference_model.a[reference_body_id], custom_model.a[custom_body_id]));
}

TEST_F(RdlCustomJointFixture, updateKinematicsCustom)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
    }

    updateKinematicsCustom(reference_model, &q, nullptr, nullptr);
    updateKinematicsCustom(custom_model, &q, nullptr, nullptr);

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(reference_model.bodyFrames[reference_body_id]->getTransformToRoot().toMatrix(),
                                                                custom_model.bodyFrames[custom_body_id]->getTransformToRoot().toMatrix(), unit_test_utils::E_MINUS_14));

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEq(Math::SpatialVectorZero, custom_model.v[custom_body_id]));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEq(Math::SpatialVectorZero, custom_model.a[custom_body_id]));

    updateKinematicsCustom(reference_model, &q, &qdot, nullptr);
    updateKinematicsCustom(custom_model, &q, &qdot, nullptr);

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(reference_model.bodyFrames[reference_body_id]->getTransformToRoot().toMatrix(),
                                                                custom_model.bodyFrames[custom_body_id]->getTransformToRoot().toMatrix(), unit_test_utils::E_MINUS_14));

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEq(reference_model.v[reference_body_id], custom_model.v[custom_body_id]));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEq(Math::SpatialVectorZero, custom_model.a[custom_body_id]));

    updateKinematicsCustom(reference_model, &q, &qdot, &qddot);
    updateKinematicsCustom(custom_model, &q, &qdot, &qddot);

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(reference_model.bodyFrames[reference_body_id]->getTransformToRoot().toMatrix(),
                                                                custom_model.bodyFrames[custom_body_id]->getTransformToRoot().toMatrix(), unit_test_utils::E_MINUS_14));

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEq(reference_model.v[reference_body_id], custom_model.v[custom_body_id]));

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEq(reference_model.a[reference_body_id], custom_model.a[custom_body_id]));
}

TEST_F(RdlCustomJointFixture, inverseDynamics)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
    }

    VectorNd tau_ref = VectorNd::Zero(tau.size());
    VectorNd tau_cust = VectorNd::Zero(tau.size());

    inverseDynamics(reference_model, q, qdot, qddot, tau_ref);
    inverseDynamics(custom_model, q, qdot, qddot, tau_cust);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(tau_ref, tau_cust, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlCustomJointFixture, compositeRigidBodyAlgorithm)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
    }

    MatrixNd H_ref(reference_model.qdot_size, reference_model.qdot_size);
    MatrixNd H_cust(custom_model.qdot_size, custom_model.qdot_size);

    compositeRigidBodyAlgorithm(reference_model, q, H_ref);
    compositeRigidBodyAlgorithm(custom_model, q, H_cust);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(H_ref, H_cust, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlCustomJointFixture, forwardDynamics)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
    }

    VectorNd qddot_ref = VectorNd::Zero(qdot.size());
    VectorNd qddot_cust = VectorNd::Zero(qdot.size());

    forwardDynamics(reference_model, q, qdot, tau, qddot_ref);
    forwardDynamics(custom_model, q, qdot, tau, qddot_cust);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_ref, qddot_cust, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlCustomJointFixture, calcMInvTimesTau)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
        tau[i] = i * 0.12;
    }

    VectorNd qddot_ref = VectorNd::Zero(qdot.size());
    VectorNd qddot_cust = VectorNd::Zero(qdot.size());

    calcMInvTimesTau(custom_model, q, tau, qddot_cust);
    calcMInvTimesTau(reference_model, q, tau, qddot_ref);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_cust, qddot_ref, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlCustomJointFixture, nonlinearEffects)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
        tau[i] = i * 0.12;
    }

    VectorNd tau_ref = VectorNd::Zero(qdot.size());
    VectorNd tau_cust = VectorNd::Zero(qdot.size());

    nonlinearEffects(custom_model, q, qdot, tau_cust);
    nonlinearEffects(reference_model, q, qdot, tau_ref);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(tau_cust, tau_ref, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlCustomJointFixture, calcPointJacobian)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
        tau[i] = i * 0.12;
    }

    MatrixNd G_cust(3, custom_model.qdot_size);
    MatrixNd G_ref(3, reference_model.qdot_size);

    Math::Vector3d point_position(0.1, -0.2, 0.1);

    calcPointJacobian(custom_model, q, custom_body_id, point_position, G_cust);
    calcPointJacobian(reference_model, q, reference_body_id, point_position, G_ref);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(G_cust, G_ref, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlCustomJointFixture, calcPointJacobian6D)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
        tau[i] = i * 0.12;
    }

    MatrixNd G_cust(6, custom_model.qdot_size);
    MatrixNd G_ref(6, reference_model.qdot_size);

    Math::Vector3d point_position(0.1, -0.2, 0.1);

    calcPointJacobian6D(custom_model, q, custom_body_id, point_position, G_cust);
    calcPointJacobian6D(reference_model, q, reference_body_id, point_position, G_ref);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(G_cust, G_ref, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlCustomJointFixture, calcBodySpatialJacobian)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
        tau[i] = i * 0.12;
    }

    MatrixNd G_cust(6, custom_model.qdot_size);
    MatrixNd G_ref(6, reference_model.qdot_size);

    Math::Vector3d point_position(0.1, -0.2, 0.1);

    calcBodySpatialJacobian(custom_model, q, custom_body_id, G_cust);
    calcBodySpatialJacobian(reference_model, q, reference_body_id, G_ref);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(G_cust, G_ref, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlCustomJointFixture, calcBodySpatialJacobianDot)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
        tau[i] = i * 0.12;
    }

    MatrixNd GDot_cust(6, custom_model.qdot_size), GDot_ref(6, custom_model.qdot_size);
    GDot_cust.setZero();
    GDot_ref.setZero();

    calcBodySpatialJacobianDot(custom_model, q, qdot, custom_body_id, GDot_cust);
    calcBodySpatialJacobianDot(reference_model, q, qdot, custom_body_id, GDot_ref);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDot_cust, GDot_ref, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlCustomJointFixture, calcPointJacobianDot)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
        tau[i] = i * 0.12;
    }

    MatrixNd GDot_cust(3, custom_model.qdot_size), GDot_ref(3, custom_model.qdot_size);
    GDot_cust.setZero();
    GDot_ref.setZero();

    Vector3d p(-0.1, -0.2, 1.1);
    calcPointJacobianDot(custom_model, q, qdot, custom_body_id, p, GDot_cust);
    calcPointJacobianDot(reference_model, q, qdot, custom_body_id, p, GDot_ref);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDot_cust, GDot_ref, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlCustomJointFixture, calcCentroidalMomentumMatrix)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
    }

    MatrixNd A(6, custom_model.qdot_size);
    A.setZero();

    Vector3d com, com_velocity, ang_momentum;

    Vector3d p(-0.1, -0.2, 1.1);
    Utils::calcCentroidalMomentumMatrix(custom_model, q, A);
    Utils::calcCenterOfMass(custom_model, q, qdot, com, &com_velocity, &ang_momentum);

    SpatialVector m_exp;
    m_exp.setLinearPart(com_velocity * custom_model.mass);
    m_exp.setAngularPart(ang_momentum);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(SpatialVector(A * qdot), m_exp, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlCustomJointFixture, calcCentroidalMomentumMatrixDot)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
        tau[i] = i * 0.12;
    }

    MatrixNd ADot_cust(6, custom_model.qdot_size), ADot_ref(6, custom_model.qdot_size);
    ADot_cust.setZero();
    ADot_ref.setZero();

    Utils::calcCentroidalMomentumMatrixDot(reference_model, q, qdot, ADot_ref);
    Utils::calcCentroidalMomentumMatrixDot(custom_model, q, qdot, ADot_cust);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(ADot_ref, ADot_cust, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlCustomJointFixture, calcPointJacobianDot6D)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
        tau[i] = i * 0.12;
    }

    MatrixNd GDot_cust(6, custom_model.qdot_size), GDot_ref(6, custom_model.qdot_size);
    GDot_cust.setZero();
    GDot_ref.setZero();

    Vector3d p(-0.1, -0.2, 1.1);
    calcPointJacobianDot6D(custom_model, q, qdot, custom_body_id, p, GDot_cust);
    calcPointJacobianDot6D(reference_model, q, qdot, custom_body_id, p, GDot_ref);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDot_cust, GDot_ref, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlCustomJointFixture, calcPointVelocity)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
        tau[i] = i * 0.12;
    }

    MatrixNd G_cust(6, custom_model.qdot_size);
    MatrixNd G_ref(6, reference_model.qdot_size);

    Math::Vector3d point_position(0.1, -0.2, 0.1);

    FrameVector v_cust = calcPointVelocity(custom_model, q, qdot, custom_body_id, point_position);
    FrameVector v_ref = calcPointVelocity(reference_model, q, qdot, reference_body_id, point_position);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v_cust, v_ref, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlCustomJointFixture, calcPointVelocity6D)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
        tau[i] = i * 0.12;
    }

    MatrixNd G_cust(6, custom_model.qdot_size);
    MatrixNd G_ref(6, reference_model.qdot_size);

    Math::Vector3d point_position(0.1, -0.2, 0.1);

    FrameVectorPair v_cust = calcPointVelocity6D(custom_model, q, qdot, custom_body_id, point_position);
    FrameVectorPair v_ref = calcPointVelocity6D(reference_model, q, qdot, reference_body_id, point_position);

    EXPECT_TRUE(v_cust.linear().isApprox(v_ref.linear(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(v_cust.angular().isApprox(v_ref.angular(), unit_test_utils::E_MINUS_12));
}

TEST_F(RdlCustomJointFixture, forwardDynamicsContacts)
{
    ConstraintSet c_ref, c_cust;

    for (unsigned int i = 0; i < 3; i++)
    {
        q[i] = i * 0.1;
        qdot[i] = i * 0.15;
        qddot[i] = i * 0.17;
        tau[i] = i * 0.12;
    }

    Math::VectorNd qddot_ref(qddot.size());
    Math::VectorNd qddot_cust(qddot.size());

    c_ref.addConstraint(1, Math::Vector3d(0., 0.05, -0.12), Math::Vector3d(1., 0., 0.));
    c_cust.addConstraint(1, Math::Vector3d(0., 0.05, -0.12), Math::Vector3d(1., 0., 0.));
    c_cust.bind(custom_model);
    c_ref.bind(reference_model);

    forwardDynamicsContactsDirect(custom_model, q, qdot, tau, c_cust, qddot_cust);
    forwardDynamicsContactsDirect(reference_model, q, qdot, tau, c_ref, qddot_ref);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_cust, qddot_ref, unit_test_utils::E_MINUS_14));

    forwardDynamicsContactsRangeSpaceSparse(custom_model, q, qdot, tau, c_cust, qddot_cust);
    forwardDynamicsContactsRangeSpaceSparse(reference_model, q, qdot, tau, c_ref, qddot_ref);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_cust, qddot_ref, unit_test_utils::E_MINUS_14));

    forwardDynamicsContactsNullSpace(custom_model, q, qdot, tau, c_cust, qddot_cust);
    forwardDynamicsContactsNullSpace(reference_model, q, qdot, tau, c_ref, qddot_ref);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_cust, qddot_ref, unit_test_utils::E_MINUS_14));

    forwardDynamicsContactsKokkevis(custom_model, q, qdot, tau, c_cust, qddot_cust);
    forwardDynamicsContactsKokkevis(reference_model, q, qdot, tau, c_ref, qddot_ref);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_cust, qddot_ref, unit_test_utils::E_MINUS_14));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
