// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>
#include <iostream>

#include "Fixtures.hpp"
#include "Human36Fixture.hpp"
#include "rdl_dynamics/rdl_mathutils.hpp"
#include "rdl_dynamics/Model.hpp"
#include "rdl_dynamics/Kinematics.hpp"
#include "rdl_dynamics/Dynamics.hpp"
#include "rdl_dynamics/Contacts.hpp"
#include "rdl_dynamics/rdl_utils.hpp"
#include <vector>

#include "UnitTestUtils.hpp"

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

const unsigned int NUMBER_OF_MODELS = 3;
const unsigned int NUMBER_OF_BODIES = 3;

//==============================================================================
/*

  The purpose of this test is to test that all of the code in RDL
  related to a multibody mechanism that includes a custom joint functions.
  Specifically this test is for the multi-pass algorithms in rdl ... namely
  the CompositeRigidBodyAlgorithm. However, because these tests have already
  been written for CustomJointSingleBodyTests.cc, we'll run them all on the
  multibody models that we will be testing.

  We will be testing 3 models to get good coverage of the
  CompositeRigidBodyAlgorithm:

  1. Rx   - multidof - custom
  2. Rx   - custom   - multidof
  3. custom - multidof - Rx

  As before, to test that the model works, we will create a model using
  standard RDL versions (the reference model), and then we will create
  a model using a custom joint in the correct location. The following
  algorithms will be tested in this manner:

  UpdateKinematicsCustom
  Jacobians
  InverseDynamics
  CompositeRigidBodyAlgorithm
  ForwardDynamics
  CalcMInvTimestau
  ForwardDynamicsContactsKokkevis

*/
//==============================================================================

struct CustomJointTypeRevoluteX : public CustomJoint
{
    CustomJointTypeRevoluteX()
    {
        mDoFCount = 1;
        S = MatrixNd::Zero(6, 1);
        S(0, 0) = 1.0;
        S_o = MatrixNd::Zero(6, 1);
        d_u = MatrixNd::Zero(mDoFCount, 1);
    }

    virtual void jcalc(Model& model, unsigned int joint_id, const Math::VectorNd& q, const Math::VectorNd& qdot)
    {
        model.X_J[joint_id] = Xrotx(q[model.mJoints[joint_id].q_index]);
        model.v_J[joint_id][0] = qdot[model.mJoints[joint_id].q_index];
    }

    virtual void jcalc_X_lambda_S(Model& model, unsigned int joint_id, const Math::VectorNd& q)
    {
        model.X_J[joint_id] = Xrotx(q[model.mJoints[joint_id].q_index]);

        const Joint& joint = model.mJoints[joint_id];
        model.mCustomJoints[joint.custom_joint_index]->S = S;
    }
};

struct CustomEulerZYXJoint : public CustomJoint
{
    CustomEulerZYXJoint()
    {
        mDoFCount = 3;
        S = MatrixNd::Zero(6, 3);
        S_o = MatrixNd::Zero(6, 3);
        d_u = MatrixNd::Zero(mDoFCount, 1);
    }

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

        S.setZero();
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

        model.c_J[joint_id] = S_o * qdotv;
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

        model.X_J[joint_id] = SpatialTransform(
            Matrix3d(c0 * c1, s0 * c1, -s1, c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2, c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2),
            Vector3d(0., 0., 0.));

        S.setZero();
        S(0, 0) = -s1;
        S(0, 2) = 1.;

        S(1, 0) = c1 * s2;
        S(1, 1) = c2;

        S(2, 0) = c1 * c2;
        S(2, 1) = -s2;

        const Joint& joint = model.mJoints[joint_id];
        model.mCustomJoints[joint.custom_joint_index]->S = S;
    }
};

//==============================================================================
// Test Fixture
//==============================================================================

struct RdlCustomJointMultiBodyFixture : public testing::Test
{
    RdlCustomJointMultiBodyFixture()
    {
    }

    void SetUp()
    {
        reference_model.resize(NUMBER_OF_MODELS);
        custom_model.resize(NUMBER_OF_MODELS);

        body.resize(NUMBER_OF_MODELS);
        custom_joint.resize(NUMBER_OF_MODELS);

        reference_body_id.resize(NUMBER_OF_MODELS);
        custom_body_id.resize(NUMBER_OF_MODELS);

        for (unsigned int i = 0; i < NUMBER_OF_MODELS; ++i)
        {
            body.at(i).resize(3);
            custom_joint.at(i).resize(1);

            reference_body_id.at(i).resize(3);
            custom_body_id.at(i).resize(3);
        }

        q.resize(NUMBER_OF_MODELS);
        qdot.resize(NUMBER_OF_MODELS);
        qddot.resize(NUMBER_OF_MODELS);
        tau.resize(NUMBER_OF_MODELS);

        //========================================================
        // Test Model 1: Rx - multidof3 - custom
        //========================================================

        custom_rx_joint1 = CustomJointTypeRevoluteX();

        Matrix3d inertia1 = Matrix3d::Identity(3, 3);

        Body body11 = Body(1., Vector3d(1.1, 1.2, 1.3), inertia1);
        Body body12 = Body(2., Vector3d(2.1, 2.2, 2.3), inertia1);
        Body body13 = Body(3., Vector3d(3.1, 3.2, 3.3), inertia1);

        ModelPtr reference1(new Model()), custom1(new Model());

        Vector3d r1 = Vector3d(0.78, -0.125, 0.37);

        double th1 = M_PI / 6.0;
        double sinTh1 = sin(th1);
        double cosTh1 = cos(th1);

        Matrix3d rm1 = Matrix3d(1.0, 0., 0., 0., cosTh1, -sinTh1, 0., sinTh1, cosTh1);

        Vector3d r2 = Vector3d(-0.178, 0.2125, -0.937);

        double th2 = M_PI / 2.15;
        double sinTh2 = sin(th2);
        double cosTh2 = cos(th2);

        Matrix3d rm2 = Matrix3d(cosTh2, 0., sinTh2, 0., 1., 0., -sinTh2, 0., cosTh2);

        unsigned int reference_body_id10 = reference1->addBody(0, SpatialTransform(), Joint(JointTypeRevoluteX), body11);

        unsigned int reference_body_id11 = reference1->addBody(reference_body_id10, SpatialTransform(rm1, r1), Joint(JointTypeEulerZYX), body12);

        unsigned int reference_body_id12 = reference1->addBody(reference_body_id11, SpatialTransform(rm2, r2), Joint(JointTypeRevoluteX), body13);

        unsigned int custom_body_id10 = custom1->addBody(0, SpatialTransform(), Joint(JointTypeRevoluteX), body11);

        unsigned int custom_body_id11 = custom1->addBody(custom_body_id10, SpatialTransform(rm1, r1), Joint(JointTypeEulerZYX), body12);

        unsigned int custom_body_id12 = custom1->addBodyCustomJoint(custom_body_id11, SpatialTransform(rm2, r2), &custom_rx_joint1, body13);

        VectorNd q1 = VectorNd::Zero(reference1->q_size);
        VectorNd qdot1 = VectorNd::Zero(reference1->qdot_size);
        VectorNd qddot1 = VectorNd::Zero(reference1->qdot_size);
        VectorNd tau1 = VectorNd::Zero(reference1->qdot_size);

        int idx = 0;
        reference_model.at(idx) = (reference1);
        custom_model.at(idx) = (custom1);

        reference_body_id.at(idx).at(0) = (reference_body_id10);
        reference_body_id.at(idx).at(1) = (reference_body_id11);
        reference_body_id.at(idx).at(2) = (reference_body_id12);

        custom_body_id.at(idx).at(0) = (custom_body_id10);
        custom_body_id.at(idx).at(1) = (custom_body_id11);
        custom_body_id.at(idx).at(2) = (custom_body_id12);

        body.at(idx).at(0) = (body11);
        body.at(idx).at(1) = (body12);
        body.at(idx).at(2) = (body13);
        custom_joint.at(idx).at(0) = (&custom_rx_joint1);

        q.at(idx) = (q1);
        qdot.at(idx) = (qdot1);
        qddot.at(idx) = (qddot1);
        tau.at(idx) = (tau1);

        //========================================================
        // Test Model 2: Rx - custom - multidof3
        //========================================================

        ModelPtr reference2(new Model()), custom2(new Model());

        unsigned int reference_body_id20 = reference2->addBody(0, SpatialTransform(), Joint(JointTypeRevoluteX), body11);

        unsigned int reference_body_id21 = reference2->addBody(reference_body_id20, SpatialTransform(rm2, r2), Joint(JointTypeRevoluteX), body12);

        unsigned int reference_body_id22 = reference2->addBody(reference_body_id21, SpatialTransform(rm1, r1), Joint(JointTypeEulerZYX), body13);

        unsigned int custom_body_id20 = custom2->addBody(0, SpatialTransform(), Joint(JointTypeRevoluteX), body11);

        unsigned int custom_body_id21 = custom2->addBodyCustomJoint(custom_body_id20, SpatialTransform(rm2, r2), &custom_rx_joint1, body12);

        unsigned int custom_body_id22 = custom2->addBody(custom_body_id21, SpatialTransform(rm1, r1), Joint(JointTypeEulerZYX), body13);

        VectorNd q2 = VectorNd::Zero(reference2->q_size);
        VectorNd qdot2 = VectorNd::Zero(reference2->qdot_size);
        VectorNd qddot2 = VectorNd::Zero(reference2->qdot_size);
        VectorNd tau2 = VectorNd::Zero(reference2->qdot_size);

        idx = 1;
        reference_model.at(idx) = (reference2);
        custom_model.at(idx) = (custom2);

        reference_body_id.at(idx).at(0) = (reference_body_id20);
        reference_body_id.at(idx).at(1) = (reference_body_id21);
        reference_body_id.at(idx).at(2) = (reference_body_id22);

        custom_body_id.at(idx).at(0) = (custom_body_id20);
        custom_body_id.at(idx).at(1) = (custom_body_id21);
        custom_body_id.at(idx).at(2) = (custom_body_id22);

        body.at(idx).at(0) = (body11);
        body.at(idx).at(1) = (body12);
        body.at(idx).at(2) = (body13);
        custom_joint.at(idx).at(0) = (&custom_rx_joint1);

        q.at(idx) = (q2);
        qdot.at(idx) = (qdot2);
        qddot.at(idx) = (qddot2);
        tau.at(idx) = (tau2);

        //========================================================
        // Test Model 3: custom - multidof3 - Rx
        //========================================================

        ModelPtr reference3(new Model()), custom3(new Model());

        unsigned int reference_body_id30 = reference3->addBody(0, SpatialTransform(), Joint(JointTypeRevoluteX), body11);

        unsigned int reference_body_id31 = reference3->addBody(reference_body_id30, SpatialTransform(rm1, r1), Joint(JointTypeEulerZYX), body12);

        unsigned int reference_body_id32 = reference3->addBody(reference_body_id31, SpatialTransform(rm2, r2), Joint(JointTypeRevoluteX), body13);

        unsigned int custom_body_id30 = custom3->addBodyCustomJoint(0, SpatialTransform(), &custom_rx_joint1, body11);

        unsigned int custom_body_id31 = custom3->addBody(custom_body_id30, SpatialTransform(rm1, r1), Joint(JointTypeEulerZYX), body12);

        unsigned int custom_body_id32 = custom3->addBody(custom_body_id31, SpatialTransform(rm2, r2), Joint(JointTypeRevoluteX), body13);

        VectorNd q3 = VectorNd::Zero(reference3->q_size);
        VectorNd qdot3 = VectorNd::Zero(reference3->qdot_size);
        VectorNd qddot3 = VectorNd::Zero(reference3->qdot_size);
        VectorNd tau3 = VectorNd::Zero(reference3->qdot_size);

        idx = 2;
        reference_model.at(idx) = (reference3);
        custom_model.at(idx) = (custom3);

        reference_body_id.at(idx).at(0) = (reference_body_id30);
        reference_body_id.at(idx).at(1) = (reference_body_id31);
        reference_body_id.at(idx).at(2) = (reference_body_id32);

        custom_body_id.at(idx).at(0) = (custom_body_id30);
        custom_body_id.at(idx).at(1) = (custom_body_id31);
        custom_body_id.at(idx).at(2) = (custom_body_id32);

        body.at(idx).at(0) = (body11);
        body.at(idx).at(1) = (body12);
        body.at(idx).at(2) = (body13);
        custom_joint.at(idx).at(0) = (&custom_rx_joint1);

        q.at(idx) = (q3);
        qdot.at(idx) = (qdot3);
        qddot.at(idx) = (qddot3);
        tau.at(idx) = (tau3);
    }

    void TearDown()
    {
        for (unsigned int i = 0; i < reference_model.size(); i++)
        {
            EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(*reference_model[i]));
        }

        for (unsigned int i = 0; i < custom_model.size(); i++)
        {
            EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(*custom_model[i]));
        }
    }

    vector<ModelPtr> reference_model;
    vector<ModelPtr> custom_model;

    vector<vector<Body>> body;
    vector<vector<CustomJoint*>> custom_joint;

    vector<vector<unsigned int>> reference_body_id;
    vector<vector<unsigned int>> custom_body_id;

    vector<VectorNd> q;
    vector<VectorNd> qdot;
    vector<VectorNd> qddot;
    vector<VectorNd> tau;

    CustomJointTypeRevoluteX custom_rx_joint1;
    CustomJointTypeRevoluteX custom_rx_joint2;
    CustomJointTypeRevoluteX custom_rx_joint3;
};

//==============================================================================
//
// Tests
//  UpdateKinematicsCustom
//  Jacobians
//  InverseDynamics
//  CompositeRigidBodyAlgorithm
//  ForwardDynamics
//  CalcMInvTimestau
//  ForwardDynamicsContactsKokkevis
//
//==============================================================================

TEST_F(RdlCustomJointMultiBodyFixture, UpdateKinematics)
{
    VectorNd test;

    for (unsigned int idx = 0; idx < NUMBER_OF_MODELS; ++idx)
    {
        unsigned int dof = reference_model.at(idx)->dof_count;
        for (unsigned int i = 0; i < dof; ++i)
        {
            q.at(idx)[i] = i * 0.1;
            qdot.at(idx)[i] = i * 0.15;
            qddot.at(idx)[i] = i * 0.17;
        }

        updateKinematics(*reference_model.at(idx), q.at(idx), qdot.at(idx), qddot.at(idx));
        updateKinematics(*custom_model.at(idx), q.at(idx), qdot.at(idx), qddot.at(idx));

        EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(
            reference_model.at(idx)->bodyFrames[reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1)]->getTransformToRoot().E,
            custom_model.at(idx)->bodyFrames[custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1)]->getTransformToRoot().E, unit_test_utils::E_MINUS_11));
        EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(
            reference_model.at(idx)->bodyFrames[reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1)]->getTransformToRoot().r,
            custom_model.at(idx)->bodyFrames[custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1)]->getTransformToRoot().r, unit_test_utils::E_MINUS_11));
        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(reference_model.at(idx)->v[reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1)],
                                                                     custom_model.at(idx)->v[custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1)],
                                                                     unit_test_utils::E_MINUS_11));
        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(reference_model.at(idx)->a[reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1)],
                                                                     custom_model.at(idx)->a[custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1)],
                                                                     unit_test_utils::E_MINUS_11));
    }
}

TEST_F(RdlCustomJointMultiBodyFixture, updateKinematicsCustom)
{
    for (unsigned int idx = 0; idx < NUMBER_OF_MODELS; ++idx)
    {
        unsigned int dof = reference_model.at(idx)->dof_count;
        for (unsigned int i = 0; i < dof; ++i)
        {
            q.at(idx)[i] = i * 9.133758561390194e-01;
            qdot.at(idx)[i] = i * 6.323592462254095e-01;
            qddot.at(idx)[i] = i * 9.754040499940952e-02;
        }

        updateKinematicsCustom(*reference_model.at(idx), &q.at(idx), NULL, NULL);
        updateKinematicsCustom(*custom_model.at(idx), &q.at(idx), NULL, NULL);

        EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(
            reference_model.at(idx)->bodyFrames[reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1)]->getTransformToRoot().E,
            custom_model.at(idx)->bodyFrames[custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1)]->getTransformToRoot().E, unit_test_utils::E_MINUS_11));
        EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(
            reference_model.at(idx)->bodyFrames[reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1)]->getTransformToRoot().r,
            custom_model.at(idx)->bodyFrames[custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1)]->getTransformToRoot().r, unit_test_utils::E_MINUS_11));

        // velocity
        updateKinematicsCustom(*reference_model.at(idx), &q.at(idx), &qdot.at(idx), NULL);
        updateKinematicsCustom(*custom_model.at(idx), &q.at(idx), &qdot.at(idx), NULL);

        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(reference_model.at(idx)->v[reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1)],
                                                                     custom_model.at(idx)->v[custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1)],
                                                                     unit_test_utils::E_MINUS_11));

        // All
        updateKinematicsCustom(*reference_model.at(idx), &q.at(idx), &qdot.at(idx), &qddot.at(idx));
        updateKinematicsCustom(*custom_model.at(idx), &q.at(idx), &qdot.at(idx), &qddot.at(idx));

        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(reference_model.at(idx)->a[reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1)],
                                                                     custom_model.at(idx)->a[custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1)],
                                                                     unit_test_utils::E_MINUS_11));
    }
}

TEST_F(RdlCustomJointMultiBodyFixture, Jacobians)
{
    for (unsigned int idx = 0; idx < NUMBER_OF_MODELS; ++idx)
    {
        unsigned int dof = reference_model.at(idx)->dof_count;

        for (unsigned int i = 0; i < dof; i++)
        {
            q.at(idx)[i] = i * 9.133758561390194e-01;
            qdot.at(idx)[i] = i * 6.323592462254095e-01;
            qddot.at(idx)[i] = i * 9.754040499940952e-02;
        }

        // position
        updateKinematics(*reference_model.at(idx), q.at(idx), qdot.at(idx), qddot.at(idx));
        updateKinematics(*custom_model.at(idx), q.at(idx), qdot.at(idx), qddot.at(idx));

        // Check the Spatial Jacobian
        MatrixNd Gref = MatrixNd(MatrixNd::Zero(6, reference_model.at(idx)->dof_count));
        MatrixNd Gcus = MatrixNd(MatrixNd::Zero(6, reference_model.at(idx)->dof_count));
        MatrixNd Gcus2 = MatrixNd(MatrixNd::Zero(6, reference_model.at(idx)->dof_count));
        MatrixNd GDotref = MatrixNd(MatrixNd::Zero(6, reference_model.at(idx)->dof_count));
        MatrixNd GDotcus = MatrixNd(MatrixNd::Zero(6, reference_model.at(idx)->dof_count));
        MatrixNd GDotcus2 = MatrixNd(MatrixNd::Zero(6, reference_model.at(idx)->dof_count));

        calcBodySpatialJacobian(*reference_model.at(idx), q.at(idx), reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1), Gref);
        calcBodySpatialJacobian(*custom_model.at(idx), q.at(idx), custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1), Gcus);

        EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(Gref, Gcus, unit_test_utils::E_MINUS_11));

        // Check the Spatial Jacobian
        Gref.setZero();
        Gcus.setZero();

        ReferenceFramePtr b1_ref_frame = reference_model.at(idx)->bodyFrames[reference_body_id.at(idx)[0]];
        ReferenceFramePtr b2_ref_frame = reference_model.at(idx)->bodyFrames[reference_body_id.at(idx)[1]];
        ReferenceFramePtr b3_ref_frame = reference_model.at(idx)->bodyFrames[reference_body_id.at(idx)[2]];

        ReferenceFramePtr b1_cus_frame = custom_model.at(idx)->bodyFrames[custom_body_id.at(idx)[0]];
        ReferenceFramePtr b2_cus_frame = custom_model.at(idx)->bodyFrames[custom_body_id.at(idx)[1]];
        ReferenceFramePtr b3_cus_frame = custom_model.at(idx)->bodyFrames[custom_body_id.at(idx)[2]];

        calcRelativeBodySpatialJacobian(*reference_model.at(idx), q.at(idx), Gref, b1_ref_frame, b2_ref_frame, b3_ref_frame);
        calcRelativeBodySpatialJacobian(*custom_model.at(idx), q.at(idx), Gcus, b1_cus_frame, b2_cus_frame, b3_cus_frame);

        EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(Gref, Gcus, unit_test_utils::E_MINUS_14));

        GDotref.setZero();
        GDotcus.setZero();
        GDotcus2.setZero();
        Gcus2.setZero();
        Gcus.setZero();

        calcRelativeBodySpatialJacobianDot(*reference_model.at(idx), q.at(idx), qdot.at(idx), GDotref, b1_ref_frame, b2_ref_frame, b3_ref_frame);
        calcRelativeBodySpatialJacobianDot(*custom_model.at(idx), q.at(idx), qdot.at(idx), GDotcus, b1_cus_frame, b2_cus_frame, b3_cus_frame);

        calcRelativeBodySpatialJacobian(*custom_model.at(idx), q.at(idx), Gcus, b1_cus_frame, b2_cus_frame, b3_cus_frame);
        calcRelativeBodySpatialJacobianAndJacobianDot(*custom_model.at(idx), q.at(idx), qdot.at(idx), Gcus2, GDotcus2, b1_cus_frame, b2_cus_frame, b3_cus_frame);

        EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDotref, GDotcus, unit_test_utils::E_MINUS_11));
        EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDotcus, GDotcus2, unit_test_utils::E_MINUS_11));
        EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(Gcus, Gcus2, unit_test_utils::E_MINUS_11));

        GDotref.setZero();
        GDotcus.setZero();

        calcBodySpatialJacobianDot(*reference_model.at(idx), q.at(idx), qdot.at(idx), reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1), GDotref);
        calcBodySpatialJacobianDot(*custom_model.at(idx), q.at(idx), qdot.at(idx), custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1), GDotcus);

        for (unsigned int i = 0; i < 6; ++i)
        {
            for (unsigned int j = 0; j < dof; ++j)
            {
                EXPECT_NEAR(GDotref(i, j), GDotcus(i, j), unit_test_utils::E_MINUS_11);
            }
        }

        // Check the 6d point Jacobian
        Vector3d point_position(1.1, 1.2, 2.1);

        calcPointJacobian6D(*reference_model.at(idx), q.at(idx), reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1), point_position, Gref);
        calcPointJacobian6D(*custom_model.at(idx), q.at(idx), custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1), point_position, Gcus);

        for (unsigned int i = 0; i < 6; ++i)
        {
            for (unsigned int j = 0; j < dof; ++j)
            {
                EXPECT_NEAR(Gref(i, j), Gcus(i, j), unit_test_utils::E_MINUS_11);
            }
        }

        Gref.setZero();
        Gcus.setZero();
        calcPointJacobianDot6D(*reference_model.at(idx), q.at(idx), qdot.at(idx), reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1), point_position, Gref);
        calcPointJacobianDot6D(*custom_model.at(idx), q.at(idx), qdot.at(idx), custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1), point_position, Gcus);

        for (unsigned int i = 0; i < 6; ++i)
        {
            for (unsigned int j = 0; j < dof; ++j)
            {
                EXPECT_NEAR(Gref(i, j), Gcus(i, j), unit_test_utils::E_MINUS_11);
            }
        }

        Gref = MatrixNd::Constant(3, reference_model.at(idx)->dof_count, 0.);
        Gcus = MatrixNd::Constant(3, reference_model.at(idx)->dof_count, 0.);

        calcPointJacobianDot(*reference_model.at(idx), q.at(idx), qdot.at(idx), reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1), point_position, Gref);
        calcPointJacobianDot(*custom_model.at(idx), q.at(idx), qdot.at(idx), custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1), point_position, Gcus);

        for (unsigned int i = 0; i < 3; ++i)
        {
            for (unsigned int j = 0; j < dof; ++j)
            {
                EXPECT_NEAR(Gref(i, j), Gcus(i, j), unit_test_utils::E_MINUS_11);
            }
        }

        // Check the 3d point Jacobian
        MatrixNd GrefPt = MatrixNd::Constant(3, reference_model.at(idx)->dof_count, 0.);
        MatrixNd GcusPt = MatrixNd::Constant(3, reference_model.at(idx)->dof_count, 0.);

        MatrixNd GDotrefPt = MatrixNd::Constant(3, reference_model.at(idx)->dof_count, 0.);
        MatrixNd GDotcusPt = MatrixNd::Constant(3, reference_model.at(idx)->dof_count, 0.);

        calcPointJacobian(*reference_model.at(idx), q.at(idx), reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1), point_position, GrefPt);
        calcPointJacobian(*custom_model.at(idx), q.at(idx), custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1), point_position, GcusPt);

        for (unsigned int i = 0; i < 3; ++i)
        {
            for (unsigned int j = 0; j < dof; ++j)
            {
                EXPECT_NEAR(GrefPt(i, j), GcusPt(i, j), unit_test_utils::E_MINUS_11);
            }
        }

        GrefPt = MatrixNd::Constant(6, reference_model.at(idx)->qdot_size, 0.);
        GcusPt = MatrixNd::Constant(6, reference_model.at(idx)->qdot_size, 0.);

        RobotDynamics::ReferenceFramePtr frame1 = reference_model.at(idx)->bodyFrames[reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1)];
        RobotDynamics::ReferenceFramePtr frame2 = custom_model.at(idx)->bodyFrames[custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1)];

        calcPointJacobian6D(*reference_model.at(idx), q.at(idx), GrefPt, frame1);
        calcPointJacobian6D(*custom_model.at(idx), q.at(idx), GcusPt, frame2);

        for (unsigned int i = 0; i < 6; ++i)
        {
            for (unsigned int j = 0; j < dof; ++j)
            {
                EXPECT_NEAR(GrefPt(i, j), GcusPt(i, j), unit_test_utils::E_MINUS_11);
            }
        }

        GrefPt = MatrixNd::Constant(6, reference_model.at(idx)->qdot_size, 0.);
        GcusPt = MatrixNd::Constant(6, reference_model.at(idx)->qdot_size, 0.);

        frame1 = reference_model.at(idx)->bodyFrames[reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1)];
        frame2 = custom_model.at(idx)->bodyFrames[custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1)];

        calcPointJacobianDot6D(*reference_model.at(idx), q.at(idx), qdot.at(idx), frame1, GrefPt);
        calcPointJacobianDot6D(*custom_model.at(idx), q.at(idx), qdot.at(idx), frame2, GcusPt);

        for (unsigned int i = 0; i < 6; ++i)
        {
            for (unsigned int j = 0; j < dof; ++j)
            {
                EXPECT_NEAR(GrefPt(i, j), GcusPt(i, j), unit_test_utils::E_MINUS_11);
            }
        }

        GrefPt = MatrixNd::Constant(3, reference_model.at(idx)->qdot_size, 0.);
        GcusPt = MatrixNd::Constant(3, reference_model.at(idx)->qdot_size, 0.);

        frame1 = reference_model.at(idx)->bodyFrames[reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1)];
        frame2 = custom_model.at(idx)->bodyFrames[custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1)];

        calcPointJacobianDot(*reference_model.at(idx), q.at(idx), qdot.at(idx), frame1, GrefPt);
        calcPointJacobianDot(*custom_model.at(idx), q.at(idx), qdot.at(idx), frame2, GcusPt);

        EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GrefPt, GcusPt, unit_test_utils::E_MINUS_11));

        GrefPt = MatrixNd::Constant(3, reference_model.at(idx)->qdot_size, 0.);
        GcusPt = MatrixNd::Constant(3, reference_model.at(idx)->qdot_size, 0.);

        frame1 = reference_model.at(idx)->bodyFrames[reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1)];
        frame2 = custom_model.at(idx)->bodyFrames[custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1)];

        calcPointJacobian(*reference_model.at(idx), q.at(idx), GrefPt, frame1);
        calcPointJacobian(*custom_model.at(idx), q.at(idx), GcusPt, frame2);

        EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GrefPt, GcusPt, unit_test_utils::E_MINUS_11));

        GrefPt = MatrixNd::Constant(6, reference_model.at(idx)->qdot_size, 0.);
        GcusPt = MatrixNd::Constant(6, reference_model.at(idx)->qdot_size, 0.);

        GDotrefPt = MatrixNd::Constant(6, reference_model.at(idx)->qdot_size, 0.);
        GDotcusPt = MatrixNd::Constant(6, reference_model.at(idx)->qdot_size, 0.);

        frame1 = reference_model.at(idx)->bodyFrames[reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1)];
        RobotDynamics::ReferenceFramePtr frame1Rel = reference_model.at(idx)->bodyFrames[reference_body_id.at(idx).at(NUMBER_OF_BODIES - 2)];
        frame2 = custom_model.at(idx)->bodyFrames[custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1)];
        RobotDynamics::ReferenceFramePtr frame2Rel = custom_model.at(idx)->bodyFrames[custom_body_id.at(idx).at(NUMBER_OF_BODIES - 2)];

        calcRelativePointJacobian6D(*reference_model.at(idx), q.at(idx), GrefPt, frame1, frame1Rel, frame1);
        calcRelativePointJacobian6D(*custom_model.at(idx), q.at(idx), GcusPt, frame2, frame2Rel, frame2);

        EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GrefPt, GcusPt, unit_test_utils::E_MINUS_11));

        calcRelativePointJacobianDot6D(*reference_model.at(idx), q.at(idx), qdot.at(idx), GDotrefPt, frame1, frame1Rel, frame1);
        calcRelativePointJacobianDot6D(*custom_model.at(idx), q.at(idx), qdot.at(idx), GDotcusPt, frame2, frame2Rel, frame2);

        EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDotrefPt, GDotcusPt, unit_test_utils::E_MINUS_11));

        MatrixNd GcusPt2 = MatrixNd::Constant(6, custom_model.at(idx)->qdot_size, 0.);
        MatrixNd GDotcusPt2 = MatrixNd::Constant(6, custom_model.at(idx)->qdot_size, 0.);
        GcusPt = MatrixNd::Constant(6, custom_model.at(idx)->qdot_size, 0.);
        GDotcusPt = MatrixNd::Constant(6, custom_model.at(idx)->qdot_size, 0.);

        calcRelativePointJacobian6D(*custom_model.at(idx), q.at(idx), GcusPt, frame2, frame2Rel, frame2);
        calcRelativePointJacobianDot6D(*custom_model.at(idx), q.at(idx), qdot.at(idx), GDotcusPt, frame2, frame2Rel, frame2);
        calcRelativePointJacobianAndJacobianDot6D(*custom_model.at(idx), q.at(idx), qdot.at(idx), GcusPt2, GDotcusPt2, frame2, frame2Rel, frame2);

        EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GcusPt, GcusPt2, unit_test_utils::E_MINUS_11));
        EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDotcusPt, GDotcusPt2, unit_test_utils::E_MINUS_11));

        MatrixNd G_com_ref = MatrixNd::Constant(3, reference_model.at(idx)->dof_count, 0.);
        MatrixNd G_com_cus = MatrixNd::Constant(3, reference_model.at(idx)->dof_count, 0.);

        Utils::calcCenterOfMassJacobian(*reference_model.at(idx), q.at(idx), G_com_ref);
        Utils::calcCenterOfMassJacobian(*custom_model.at(idx), q.at(idx), G_com_cus);

        unit_test_utils::checkMatrixNdEpsilonClose(G_com_ref, G_com_cus, unit_test_utils::E_MINUS_11);

        MatrixNd Aref = MatrixNd::Constant(6, reference_model.at(idx)->dof_count, 0.);
        MatrixNd Acus = MatrixNd::Constant(6, reference_model.at(idx)->dof_count, 0.);

        Utils::calcCentroidalMomentumMatrix(*reference_model.at(idx), q.at(idx), Aref);
        Utils::calcCentroidalMomentumMatrix(*custom_model.at(idx), q.at(idx), Acus);

        EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(Aref, Acus, unit_test_utils::E_MINUS_11));
    }
}

TEST_F(RdlCustomJointMultiBodyFixture, calcCentroidalMomentumMatrixDot)
{
    for (unsigned int idx = 0; idx < NUMBER_OF_MODELS; ++idx)
    {
        unsigned int dof = reference_model.at(idx)->dof_count;

        for (unsigned int i = 0; i < dof; ++i)
        {
            q.at(idx)[i] = i * 9.133758561390194e-01;
            qdot.at(idx)[i] = i * 6.323592462254095e-01;
            qddot.at(idx)[i] = i * 9.754040499940952e-02;
        }

        // Check the Spatial Jacobian
        MatrixNd ADotref = MatrixNd(MatrixNd::Zero(6, reference_model.at(idx)->qdot_size));
        MatrixNd ADotcus = MatrixNd(MatrixNd::Zero(6, reference_model.at(idx)->qdot_size));

        Utils::calcCentroidalMomentumMatrixDot(*reference_model.at(idx), q.at(idx), qdot.at(idx), ADotref);
        Utils::calcCentroidalMomentumMatrixDot(*custom_model.at(idx), q.at(idx), qdot.at(idx), ADotcus);

        EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(ADotref, ADotcus, unit_test_utils::E_MINUS_13));
    }
}

TEST_F(RdlCustomJointMultiBodyFixture, GravityEffects)
{
    for (unsigned int idx = 0; idx < NUMBER_OF_MODELS; ++idx)
    {
        unsigned int dof = reference_model.at(idx)->dof_count;

        for (unsigned int i = 0; i < dof; ++i)
        {
            q.at(idx)[i] = i * 9.133758561390194e-01;
            qdot.at(idx)[i] = i * 6.323592462254095e-01;
            qddot.at(idx)[i] = i * 9.754040499940952e-02;
        }

        // position
        VectorNd tauRef = VectorNd::Zero(reference_model.at(idx)->qdot_size);
        VectorNd tauCus = VectorNd::Zero(reference_model.at(idx)->qdot_size);

        updateKinematics(*reference_model.at(idx), q.at(idx), qdot.at(idx), qddot.at(idx));
        updateKinematics(*custom_model.at(idx), q.at(idx), qdot.at(idx), qddot.at(idx));

        qdot.at(idx).setZero();
        nonlinearEffects(*reference_model.at(idx), q.at(idx), qdot.at(idx), tauRef);
        gravityEffects(*custom_model.at(idx), tauCus);

        EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(tauRef, tauCus, unit_test_utils::E_MINUS_11));
    }
}

TEST_F(RdlCustomJointMultiBodyFixture, InverseDynamics)
{
    for (unsigned int idx = 0; idx < NUMBER_OF_MODELS; ++idx)
    {
        unsigned int dof = reference_model.at(idx)->dof_count;

        for (unsigned int i = 0; i < dof; ++i)
        {
            q.at(idx)[i] = i * 9.133758561390194e-01;
            qdot.at(idx)[i] = i * 6.323592462254095e-01;
            qddot.at(idx)[i] = i * 9.754040499940952e-02;
        }

        // position
        VectorNd tauRef = VectorNd::Zero(reference_model.at(idx)->qdot_size);
        VectorNd tauCus = VectorNd::Zero(reference_model.at(idx)->qdot_size);

        inverseDynamics(*reference_model.at(idx), q.at(idx), qdot.at(idx), qddot.at(idx), tauRef);
        inverseDynamics(*custom_model.at(idx), q.at(idx), qdot.at(idx), qddot.at(idx), tauCus);

        VectorNd tauErr = tauRef - tauCus;

        EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(tauRef, tauCus, unit_test_utils::E_MINUS_11));
    }
}

TEST_F(RdlCustomJointMultiBodyFixture, CompositeRigidBodyAlgorithm)
{
    for (unsigned int idx = 0; idx < 2; ++idx)
    {
        unsigned int dof = reference_model.at(idx)->dof_count;

        for (unsigned int i = 0; i < dof; i++)
        {
            q.at(idx)[i] = (i + 0.1) * 9.133758561390194e-01;
            qdot.at(idx)[i] = (i + 0.1) * 6.323592462254095e-01;
            tau.at(idx)[i] = (i + 0.1) * 9.754040499940952e-02;
        }

        MatrixNd h_ref = MatrixNd::Constant(dof, dof, 0.);
        VectorNd c_ref = VectorNd::Constant(dof, 0.);
        VectorNd qddot_zero_ref = VectorNd::Constant(dof, 0.);
        VectorNd qddot_crba_ref = VectorNd::Constant(dof, 0.);

        MatrixNd h_cus = MatrixNd::Constant(dof, dof, 0.);
        VectorNd c_cus = VectorNd::Constant(dof, 0.);
        VectorNd qddot_zero_cus = VectorNd::Constant(dof, 0.);
        VectorNd qddot_crba_cus = VectorNd::Constant(dof, 0.);

        VectorNd qddotRef = VectorNd::Zero(dof);
        VectorNd qddotCus = VectorNd::Zero(dof);

        // Ref
        forwardDynamics(*reference_model.at(idx), q.at(idx), qdot.at(idx), tau.at(idx), qddotRef);
        compositeRigidBodyAlgorithm(*reference_model.at(idx), q.at(idx), h_ref);
        inverseDynamics(*reference_model.at(idx), q.at(idx), qdot.at(idx), qddot_zero_ref, c_ref);
        linSolveGaussElimPivot(h_ref, c_ref * -1. + tau.at(idx), qddot_crba_ref);

        // Custom
        forwardDynamics(*custom_model.at(idx), q.at(idx), qdot.at(idx), tau.at(idx), qddotCus);
        compositeRigidBodyAlgorithm(*custom_model.at(idx), q.at(idx), h_cus);
        inverseDynamics(*custom_model.at(idx), q.at(idx), qdot.at(idx), qddot_zero_cus, c_cus);
        linSolveGaussElimPivot(h_cus, c_cus * -1. + tau.at(idx), qddot_crba_cus);

        EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_crba_ref, qddot_crba_cus, unit_test_utils::E_MINUS_11));
    }
}

TEST_F(RdlCustomJointMultiBodyFixture, ForwardDynamics)
{
    for (unsigned int idx = 0; idx < NUMBER_OF_MODELS; ++idx)
    {
        unsigned int dof = reference_model.at(idx)->dof_count;

        for (unsigned int i = 0; i < dof; i++)
        {
            q.at(idx)[i] = (i + 0.1) * 9.133758561390194e-01;
            qdot.at(idx)[i] = (i + 0.1) * 6.323592462254095e-01;
            qddot.at(idx)[i] = (i + 0.1) * 2.323592499940952e-01;
            tau.at(idx)[i] = (i + 0.1) * 9.754040499940952e-02;
        }

        VectorNd qddotRef = VectorNd::Zero(reference_model.at(idx)->qdot_size);
        VectorNd qddotCus = VectorNd::Zero(reference_model.at(idx)->qdot_size);

        forwardDynamics(*reference_model.at(idx), q.at(idx), qdot.at(idx), tau.at(idx), qddotRef);
        forwardDynamics(*custom_model.at(idx), q.at(idx), qdot.at(idx), tau.at(idx), qddotCus);

        EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddotRef, qddotCus, unit_test_utils::E_MINUS_11));
    }
}

TEST_F(RdlCustomJointMultiBodyFixture, CalcMInvTimestau)
{
    for (unsigned int idx = 0; idx < NUMBER_OF_MODELS; ++idx)
    {
        unsigned int dof = reference_model.at(idx)->dof_count;

        for (unsigned int i = 0; i < dof; ++i)
        {
            q.at(idx)[i] = (i + 0.1) * 9.133758561390194e-01;
            tau.at(idx)[i] = (i + 0.1) * 9.754040499940952e-02;
        }

        // reference
        VectorNd qddot_minv_ref = VectorNd::Zero(dof);

        calcMInvTimesTau(*reference_model.at(idx), q.at(idx), tau.at(idx), qddot_minv_ref, true);

        // custom
        VectorNd qddot_minv_cus = VectorNd::Zero(dof);
        calcMInvTimesTau(*custom_model.at(idx), q.at(idx), tau.at(idx), qddot_minv_cus, true);

        // check.
        EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_minv_ref, qddot_minv_cus, unit_test_utils::E_MINUS_11));
    }
}

TEST_F(RdlCustomJointMultiBodyFixture, ForwardDynamicsContactsKokkevis)
{
    for (unsigned int idx = 0; idx < NUMBER_OF_MODELS; ++idx)
    {
        unsigned int dof = reference_model.at(idx)->dof_count;

        // Adding a 1 constraint to a system with 1 dof is
        // a no-no
        if (dof > 1)
        {
            for (unsigned int i = 0; i < dof; ++i)
            {
                q.at(idx)[i] = (i + 0.1) * 9.133758561390194e-01;
                qdot.at(idx)[i] = (i + 0.1) * 6.323592462254095e-01;
                tau.at(idx)[i] = (i + 0.1) * 9.754040499940952e-02;
            }

            VectorNd qddot_ref = VectorNd::Zero(dof);
            VectorNd qddot_cus = VectorNd::Zero(dof);

            VectorNd qdot_plus_ref = VectorNd::Zero(dof);
            VectorNd qdot_plus_cus = VectorNd::Zero(dof);

            Vector3d contact_point(0., 1., 0.);

            ConstraintSet constraint_set_ref;
            ConstraintSet constraint_set_cus;

            // Reference
            constraint_set_ref.addConstraint(reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1), contact_point, Vector3d(1., 0., 0.), "ground_x");

            constraint_set_ref.addConstraint(reference_body_id.at(idx).at(NUMBER_OF_BODIES - 1), contact_point, Vector3d(0., 1., 0.), "ground_y");

            constraint_set_ref.bind(*reference_model.at(idx));

            // Custom
            constraint_set_cus.addConstraint(custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1), contact_point, Vector3d(1., 0., 0.), "ground_x");

            constraint_set_cus.addConstraint(custom_body_id.at(idx).at(NUMBER_OF_BODIES - 1), contact_point, Vector3d(0., 1., 0.), "ground_y");

            constraint_set_cus.bind(*custom_model.at(idx));

            computeContactImpulsesDirect(*reference_model.at(idx), q.at(idx), qdot.at(idx), constraint_set_ref, qdot_plus_ref);
            forwardDynamicsContactsKokkevis(*reference_model.at(idx), q.at(idx), qdot_plus_ref, tau.at(idx), constraint_set_ref, qddot_ref);

            computeContactImpulsesDirect(*custom_model.at(idx), q.at(idx), qdot.at(idx), constraint_set_cus, qdot_plus_cus);
            forwardDynamicsContactsKokkevis(*custom_model.at(idx), q.at(idx), qdot_plus_cus, tau.at(idx), constraint_set_cus, qddot_cus);

            VectorNd qdot_plus_error = qdot_plus_ref - qdot_plus_cus;
            VectorNd qddot_error = qddot_ref - qddot_cus;

            EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qdot_plus_ref, qdot_plus_cus, unit_test_utils::E_MINUS_11));

            EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_ref, qddot_cus, unit_test_utils::E_MINUS_11));
        }
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
