// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>

#include "UnitTestUtils.hpp"

#include <iostream>

#include "Fixtures.hpp"
#include "rdl_dynamics/Kinematics.hpp"
#include "rdl_dynamics/Dynamics.hpp"
#include "rdl_dynamics/Contacts.hpp"
#include "rdl_dynamics/rdl_utils.hpp"
#include <vector>

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

const unsigned int NUMBER_OF_MODELS = 2;

//==============================================================================
/*

  The purpose of this test is to test that all of the code in RDL
  related to a single CustomJoint functions. To do this we will implement
  joints that already exist in RBDL but using the CustomJoint interface. The
  test will then numerically compare the results produced by the CustomJoint
  and the equivalent built-in joint in RBDL. The following algorithms will
  be tested:

    UpdateKinematicsCustom
    Jacobians
    InverseDynamics
    CompositeRigidBodyAlgorithm
    ForwardDynamics
    CalcMInvTimestau
    ForwardDynamicsContactsKokkevis

*/
//==============================================================================
//==============================================================================
/*
  As a note, below are the basic fields and functions that every CustomJoint
  class member must provide. Refer to Featherstone's dynamics text if the field
  names below don't make sense to you.

   1. Extend from CustomJoint:

      struct CustomJointClass: public CustomJoint

   2. Make a default constructor, and initialize member variables
          mDoFCount
          S
          d_u

      e.g.

      CustomJointClass()

  3. Implement the method jcalc. This method must populate X_J, v_J, c_J, and S.

        virtual void jcalc
          model.X_J[joint_id]
          model.v_J
          model.c_J
          model.mCustomJoints[joint.custom_joint_index]->S = S

  4. Implement the method jcalc_X_lambda_S. This method must populate X_lambda
      and S.

        virtual void jcalc_X_lambda_S
          model.X_lambda
          model.mCustomJoints[joint.custom_joint_index]->S = S;

 */
//==============================================================================
// Custom Joint Code
//==============================================================================
struct CustomJointTypeRevoluteX : public CustomJoint
{
    CustomJointTypeRevoluteX()
    {
        mDoFCount = 1;
        S = MatrixNd::Zero(6, 1);
        S(0, 0) = 1.0;
        d_u = MatrixNd::Zero(mDoFCount, 1);
        S_o = MatrixNd::Zero(6, 1);
    }

    virtual void jcalc(Model& model, unsigned int joint_id, const Math::VectorNd& q, const Math::VectorNd& qdot)
    {
        model.X_J[joint_id] = Xrotx(q[model.mJoints[joint_id].q_index]);
        model.v_J[joint_id][0] = qdot[model.mJoints[joint_id].q_index];

        model.bodyFrames[joint_id]->setTransformFromParent(model.X_J[joint_id] * model.X_T[joint_id]);
        model.bodyFrames[joint_id]->update();
    }

    virtual void jcalc_X_lambda_S(Model& model, unsigned int joint_id, const Math::VectorNd& q)
    {
        model.bodyFrames[joint_id]->setTransformFromParent(Xrotx(q[model.mJoints[joint_id].q_index]) * model.X_T[joint_id]);
        model.bodyFrames[joint_id]->update();

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

        model.bodyFrames[joint_id]->setTransformFromParent(model.X_J[joint_id] * model.X_T[joint_id]);
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

        const Joint& joint = model.mJoints[joint_id];
        model.mCustomJoints[joint.custom_joint_index]->S = S;
    }
};

//==============================================================================
// Test Fixture
//==============================================================================

struct CustomJointSingleBodyFixture : public testing::Test
{
    CustomJointSingleBodyFixture()
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

        q.resize(NUMBER_OF_MODELS);
        qdot.resize(NUMBER_OF_MODELS);
        qddot.resize(NUMBER_OF_MODELS);
        tau.resize(NUMBER_OF_MODELS);

        //========================================================
        // Test Model 0: 3dof rotational custom joint vs. standard.
        //========================================================

        custom_joint0 = CustomEulerZYXJoint();

        Matrix3d inertia0 = Matrix3d::Identity(3, 3);
        Body body0 = Body(1., Vector3d(1.1, 1.2, 1.3), inertia0);

        ModelPtr reference0(new Model()), custom0(new Model());

        unsigned int reference_body_id0 = reference0->addBody(0, SpatialTransform(), Joint(JointTypeEulerZYX), body0);

        unsigned int custom_body_id0 = custom0->addBodyCustomJoint(0, SpatialTransform(), &custom_joint0, body0);

        VectorNd q0 = VectorNd::Zero(reference0->q_size);
        VectorNd qdot0 = VectorNd::Zero(reference0->qdot_size);
        VectorNd qddot0 = VectorNd::Zero(reference0->qdot_size);
        VectorNd tau0 = VectorNd::Zero(reference0->qdot_size);

        reference_model.at(0) = reference0;
        custom_model.at(0) = custom0;

        reference_body_id.at(0) = (reference_body_id0);
        custom_body_id.at(0) = (custom_body_id0);

        body.at(0) = (body0);
        custom_joint.at(0) = (&custom_joint0);

        q.at(0) = (q0);
        qdot.at(0) = (qdot0);
        qddot.at(0) = (qddot0);
        tau.at(0) = (tau0);

        //========================================================
        // Test Model 1: 1 dof rotational custom joint vs. standard.
        //========================================================

        custom_joint1 = CustomJointTypeRevoluteX();

        ModelPtr reference1(new Model()), custom1(new Model());

        unsigned int reference_body_id1 = reference1->addBody(0, SpatialTransform(), Joint(JointTypeRevoluteX), body0);

        unsigned int custom_body_id1 = custom1->addBodyCustomJoint(0, SpatialTransform(), &custom_joint1, body0);

        VectorNd q1 = VectorNd::Zero(reference1->q_size);
        VectorNd qdot1 = VectorNd::Zero(reference1->qdot_size);
        VectorNd qddot1 = VectorNd::Zero(reference1->qdot_size);
        VectorNd tau1 = VectorNd::Zero(reference1->qdot_size);

        reference_model.at(1) = (reference1);
        custom_model.at(1) = (custom1);

        reference_body_id.at(1) = (reference_body_id1);
        custom_body_id.at(1) = (custom_body_id1);

        body.at(1) = (body0);
        custom_joint.at(1) = (&custom_joint1);

        q.at(1) = (q1);
        qdot.at(1) = (qdot1);
        qddot.at(1) = (qddot1);
        tau.at(1) = (tau1);
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

    vector<Body> body;
    vector<CustomJoint*> custom_joint;

    vector<unsigned int> reference_body_id;
    vector<unsigned int> custom_body_id;

    vector<VectorNd> q;
    vector<VectorNd> qdot;
    vector<VectorNd> qddot;
    vector<VectorNd> tau;
    CustomJointTypeRevoluteX custom_joint1;
    CustomEulerZYXJoint custom_joint0;
};

//==============================================================================
//
// Tests
//    UpdateKinematicsCustom
//    Jacobians
//    InverseDynamics
//    CompositeRigidBodyAlgorithm
//    ForwardDynamics
//    CalcMInvTimestau
//    ForwardDynamicsContactsKokkevis
//
//==============================================================================

TEST_F(CustomJointSingleBodyFixture, UpdateKinematics)
{
    VectorNd test;

    for (unsigned int idx = 0; idx < NUMBER_OF_MODELS; ++idx)
    {
        unsigned int dof = reference_model.at(idx)->dof_count;
        for (unsigned int i = 0; i < dof; i++)
        {
            q.at(idx)[i] = i * 0.1;
            qdot.at(idx)[i] = i * 0.15;
            qddot.at(idx)[i] = i * 0.17;
        }

        updateKinematics(*reference_model.at(idx), q.at(idx), qdot.at(idx), qddot.at(idx));
        updateKinematics(*custom_model.at(idx), q.at(idx), qdot.at(idx), qddot.at(idx));

        EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(reference_model.at(idx)->bodyFrames[reference_body_id.at(idx)]->getInverseTransformToRoot().E,
                                                               custom_model.at(idx)->bodyFrames[custom_body_id.at(idx)]->getInverseTransformToRoot().E,
                                                               unit_test_utils::E_MINUS_14));

        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(reference_model.at(idx)->v[reference_body_id.at(idx)],
                                                                     custom_model.at(idx)->v[custom_body_id.at(idx)], unit_test_utils::E_MINUS_14));

        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(reference_model.at(idx)->a[reference_body_id.at(idx)],
                                                                     custom_model.at(idx)->a[custom_body_id.at(idx)], unit_test_utils::E_MINUS_14));
    }
}

TEST_F(CustomJointSingleBodyFixture, UpdateKinematicsCustom)
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

        updateKinematicsCustom(*reference_model.at(idx), &q.at(idx), NULL, NULL);
        updateKinematicsCustom(*custom_model.at(idx), &q.at(idx), NULL, NULL);

        EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(reference_model.at(idx)->bodyFrames[reference_body_id.at(idx)]->getInverseTransformToRoot().E,
                                                               custom_model.at(idx)->bodyFrames[custom_body_id.at(idx)]->getInverseTransformToRoot().E,
                                                               unit_test_utils::E_MINUS_14));

        // velocity
        updateKinematicsCustom(*reference_model.at(idx), &q.at(idx), &qdot.at(idx), NULL);
        updateKinematicsCustom(*custom_model.at(idx), &q.at(idx), &qdot.at(idx), NULL);

        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(reference_model.at(idx)->v[reference_body_id.at(idx)],
                                                                     custom_model.at(idx)->v[custom_body_id.at(idx)], unit_test_utils::E_MINUS_14));

        // All
        updateKinematicsCustom(*reference_model.at(idx), &q.at(idx), &qdot.at(idx), &qddot.at(idx));
        updateKinematicsCustom(*custom_model.at(idx), &q.at(idx), &qdot.at(idx), &qddot.at(idx));

        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(reference_model.at(idx)->a[reference_body_id.at(idx)],
                                                                     custom_model.at(idx)->a[custom_body_id.at(idx)], unit_test_utils::E_MINUS_14));
    }
}

TEST_F(CustomJointSingleBodyFixture, Jacobians)
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
        updateKinematics(*reference_model.at(idx), q.at(idx), qdot.at(idx), qddot.at(idx));
        updateKinematics(*custom_model.at(idx), q.at(idx), qdot.at(idx), qddot.at(idx));

        // Check the Spatial Jacobian
        MatrixNd Gref = MatrixNd(MatrixNd::Zero(6, reference_model.at(idx)->dof_count));

        MatrixNd Gcus = MatrixNd(MatrixNd::Zero(6, reference_model.at(idx)->dof_count));

        calcBodySpatialJacobian(*reference_model.at(idx), q.at(idx), reference_body_id.at(idx), Gref);
        calcBodySpatialJacobian(*custom_model.at(idx), q.at(idx), custom_body_id.at(idx), Gcus);

        for (unsigned int i = 0; i < 6; ++i)
        {
            for (unsigned int j = 0; j < dof; ++j)
            {
                EXPECT_NEAR(Gref(i, j), Gcus(i, j), unit_test_utils::E_MINUS_12);
            }
        }

        // Check the 6d point Jacobian
        Vector3d point_position(1.1, 1.2, 2.1);

        calcPointJacobian6D(*reference_model.at(idx), q.at(idx), reference_body_id.at(idx), point_position, Gref);
        calcPointJacobian6D(*custom_model.at(idx), q.at(idx), custom_body_id.at(idx), point_position, Gcus);

        for (unsigned int i = 0; i < 6; ++i)
        {
            for (unsigned int j = 0; j < dof; ++j)
            {
                EXPECT_NEAR(Gref(i, j), Gcus(i, j), unit_test_utils::E_MINUS_12);
            }
        }

        // Check the 3d point Jacobian
        MatrixNd GrefPt = MatrixNd::Constant(3, reference_model.at(idx)->dof_count, 0.);
        MatrixNd GcusPt = MatrixNd::Constant(3, reference_model.at(idx)->dof_count, 0.);

        calcPointJacobian(*reference_model.at(idx), q.at(idx), reference_body_id.at(idx), point_position, GrefPt);
        calcPointJacobian(*custom_model.at(idx), q.at(idx), custom_body_id.at(idx), point_position, GcusPt);

        for (unsigned int i = 0; i < 3; ++i)
        {
            for (unsigned int j = 0; j < dof; ++j)
            {
                EXPECT_NEAR(GrefPt(i, j), GcusPt(i, j), unit_test_utils::E_MINUS_12);
            }
        }

        // Check the body spatial jacobian dot
        MatrixNd GDotrefPt = MatrixNd::Constant(6, reference_model.at(idx)->dof_count, 0.);
        MatrixNd GDotcusPt = MatrixNd::Constant(6, reference_model.at(idx)->dof_count, 0.);

        calcBodySpatialJacobianDot(*reference_model.at(idx), q.at(idx), qdot.at(idx), reference_body_id.at(idx), GDotrefPt);
        calcBodySpatialJacobianDot(*custom_model.at(idx), q.at(idx), qdot.at(idx), custom_body_id.at(idx), GDotcusPt);

        for (int i = 0; i < GDotcusPt.rows(); ++i)
        {
            for (int j = 0; j < GDotcusPt.cols(); ++j)
            {
                EXPECT_NEAR(GDotrefPt(i, j), GDotcusPt(i, j), unit_test_utils::E_MINUS_12);
            }
        }

        GDotrefPt = MatrixNd::Constant(6, reference_model.at(idx)->dof_count, 0.);
        GDotcusPt = MatrixNd::Constant(6, reference_model.at(idx)->dof_count, 0.);

        calcPointJacobianDot6D(*reference_model.at(idx), q.at(idx), qdot.at(idx), reference_body_id.at(idx), point_position, GDotrefPt);
        calcPointJacobianDot6D(*custom_model.at(idx), q.at(idx), qdot.at(idx), custom_body_id.at(idx), point_position, GDotcusPt);

        for (int i = 0; i < GDotcusPt.rows(); ++i)
        {
            for (int j = 0; j < GDotcusPt.cols(); ++j)
            {
                EXPECT_NEAR(GDotrefPt(i, j), GDotcusPt(i, j), unit_test_utils::E_MINUS_12);
            }
        }

        GDotrefPt = MatrixNd::Constant(3, reference_model.at(idx)->dof_count, 0.);
        GDotcusPt = MatrixNd::Constant(3, reference_model.at(idx)->dof_count, 0.);

        calcPointJacobianDot(*reference_model.at(idx), q.at(idx), qdot.at(idx), reference_body_id.at(idx), point_position, GDotrefPt);
        calcPointJacobianDot(*custom_model.at(idx), q.at(idx), qdot.at(idx), custom_body_id.at(idx), point_position, GDotcusPt);

        for (int i = 0; i < GDotcusPt.rows(); ++i)
        {
            for (int j = 0; j < GDotcusPt.cols(); ++j)
            {
                EXPECT_NEAR(GDotrefPt(i, j), GDotcusPt(i, j), unit_test_utils::E_MINUS_12);
            }
        }

        MatrixNd Aref = MatrixNd::Constant(6, reference_model.at(idx)->dof_count, 0.);
        MatrixNd Acus = MatrixNd::Constant(6, reference_model.at(idx)->dof_count, 0.);

        Utils::calcCentroidalMomentumMatrix(*reference_model.at(idx), q.at(idx), Aref);
        Utils::calcCentroidalMomentumMatrix(*custom_model.at(idx), q.at(idx), Acus);

        EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(Aref, Acus, unit_test_utils::E_MINUS_12));
    }
}

TEST_F(CustomJointSingleBodyFixture, CentroidalMomentum)
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

        // Check the Spatial Jacobian
        MatrixNd ADotref = MatrixNd(MatrixNd::Zero(6, reference_model.at(idx)->qdot_size));
        MatrixNd ADotcus = MatrixNd(MatrixNd::Zero(6, reference_model.at(idx)->qdot_size));

        Utils::calcCentroidalMomentumMatrixDot(*reference_model.at(idx), q.at(idx), qdot.at(idx), ADotref);
        Utils::calcCentroidalMomentumMatrixDot(*custom_model.at(idx), q.at(idx), qdot.at(idx), ADotcus);

        EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(ADotref, ADotcus, unit_test_utils::E_MINUS_14));
    }
}

TEST_F(CustomJointSingleBodyFixture, InverseDynamics)
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

        VectorNd tauRef = VectorNd::Zero(reference_model.at(idx)->qdot_size);
        VectorNd tauCus = VectorNd::Zero(reference_model.at(idx)->qdot_size);

        inverseDynamics(*reference_model.at(idx), q.at(idx), qdot.at(idx), qddot.at(idx), tauRef);
        inverseDynamics(*custom_model.at(idx), q.at(idx), qdot.at(idx), qddot.at(idx), tauCus);

        EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(tauRef, tauCus, unit_test_utils::E_MINUS_14));
    }
}

TEST_F(CustomJointSingleBodyFixture, CompositeRigidBodyAlgorithm)
{
    for (unsigned int idx = 0; idx < NUMBER_OF_MODELS; ++idx)
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

        EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_crba_ref, qddot_crba_cus, unit_test_utils::E_MINUS_14));
    }
}

TEST_F(CustomJointSingleBodyFixture, ForwardDynamics)
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

        EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddotRef, qddotCus, unit_test_utils::E_MINUS_14));
    }
}

TEST_F(CustomJointSingleBodyFixture, CalcMInvTimestau)
{
    for (unsigned int idx = 0; idx < NUMBER_OF_MODELS; ++idx)
    {
        unsigned int dof = reference_model.at(idx)->dof_count;

        for (unsigned int i = 0; i < dof; i++)
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
        EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_minv_ref, qddot_minv_cus, unit_test_utils::E_MINUS_14));
    }
}

TEST_F(CustomJointSingleBodyFixture, ForwardDynamicsContactsKokkevis)
{
    for (unsigned int idx = 0; idx < NUMBER_OF_MODELS; ++idx)
    {
        unsigned int dof = reference_model.at(idx)->dof_count;

        // Adding a 1 constraint to a system with 1 dof is
        // a no-no
        if (dof > 1)
        {
            for (unsigned int i = 0; i < dof; i++)
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
            constraint_set_ref.addConstraint(reference_body_id.at(idx), contact_point, Vector3d(1., 0., 0.), "ground_x");
            constraint_set_ref.addConstraint(reference_body_id.at(idx), contact_point, Vector3d(0., 1., 0.), "ground_y");
            constraint_set_ref.bind(*reference_model.at(idx));

            // Custom
            constraint_set_cus.addConstraint(custom_body_id.at(idx), contact_point, Vector3d(1., 0., 0.), "ground_x");
            constraint_set_cus.addConstraint(custom_body_id.at(idx), contact_point, Vector3d(0., 1., 0.), "ground_y");
            constraint_set_cus.bind(*custom_model.at(idx));

            computeContactImpulsesDirect(*reference_model.at(idx), q.at(idx), qdot.at(idx), constraint_set_ref, qdot_plus_ref);
            forwardDynamicsContactsKokkevis(*reference_model.at(idx), q.at(idx), qdot_plus_ref, tau.at(idx), constraint_set_ref, qddot_ref);

            computeContactImpulsesDirect(*custom_model.at(idx), q.at(idx), qdot.at(idx), constraint_set_cus, qdot_plus_cus);
            forwardDynamicsContactsKokkevis(*custom_model.at(idx), q.at(idx), qdot_plus_cus, tau.at(idx), constraint_set_cus, qddot_cus);

            VectorNd qdot_plus_error = qdot_plus_ref - qdot_plus_cus;
            VectorNd qddot_error = qddot_ref - qddot_cus;

            EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qdot_plus_ref, qdot_plus_cus, unit_test_utils::E_MINUS_14));

            EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_ref, qddot_cus, unit_test_utils::E_MINUS_14));
        }
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
