// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>

#include "UnitTestUtils.hpp"

#include "Fixtures.hpp"
#include "rdl_dynamics/Dynamics.hpp"

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

TEST_F(FloatingBase12DoF, TestSparseFactorizationLTL)
{
    for (unsigned int i = 0; i < model->q_size; i++)
    {
        Q[i] = static_cast<double>(i + 1) * 0.1;
    }

    MatrixNd H(MatrixNd::Zero(model->qdot_size, model->qdot_size));

    compositeRigidBodyAlgorithm(*model, Q, H);

    MatrixNd L(H);
    SparseFactorizeLTL(*model, L);
    MatrixNd LTL = L.transpose() * L;

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(H, LTL, unit_test_utils::E_MINUS_12));
}

TEST_F(FloatingBase12DoF, TestSparseSolveLx)
{
    for (unsigned int i = 0; i < model->q_size; i++)
    {
        Q[i] = static_cast<double>(i + 1) * 0.1;
    }

    MatrixNd H(MatrixNd::Zero(model->qdot_size, model->qdot_size));

    compositeRigidBodyAlgorithm(*model, Q, H);

    MatrixNd L(H);
    SparseFactorizeLTL(*model, L);
    VectorNd x = L * Q;

    SparseSolveLx(*model, L, x);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(Q, x, unit_test_utils::E_MINUS_12));
}

TEST_F(FloatingBase12DoF, TestSparseSolveLTx)
{
    for (unsigned int i = 0; i < model->q_size; i++)
    {
        Q[i] = static_cast<double>(i + 1) * 0.1;
    }

    MatrixNd H(MatrixNd::Zero(model->qdot_size, model->qdot_size));

    compositeRigidBodyAlgorithm(*model, Q, H);

    MatrixNd L(H);
    SparseFactorizeLTL(*model, L);
    VectorNd x = L.transpose() * Q;

    SparseSolveLTx(*model, L, x);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(Q, x, unit_test_utils::E_MINUS_12));
}

TEST_F(FixedBase6DoF12DoFFloatingBase, ForwardDynamicsContactsSparse)
{
    ConstraintSet constraint_set_var1;

    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(1., 0., 0.));
    constraint_set.addConstraint(contact_body_id, contact_point, Vector3d(0., 1., 0.));
    constraint_set.addConstraint(child_2_id, contact_point, Vector3d(0., 1., 0.));

    constraint_set_var1 = constraint_set.Copy();
    constraint_set_var1.bind(*model);
    constraint_set.bind(*model);

    VectorNd QDDot_var1 = VectorNd::Constant(model->dof_count, 0.);

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

    forwardDynamicsContactsRangeSpaceSparse(*model, Q, QDot, Tau, constraint_set_var1, QDDot_var1);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot, QDDot_var1, unit_test_utils::E_MINUS_12));
}

TEST_F(FloatingBase12DoF, TestSparseFactorizationMultiDof)
{
    Model model_emulated;
    Model model_3dof;

    Body body(1., Vector3d(1., 2., 1.), Matrix3d(1., 0., 0, 0., 1., 0., 0., 0., 1.));
    Joint joint_emulated(SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.), SpatialVector(0., 0., 1., 0., 0., 0.));
    Joint joint_3dof(JointTypeEulerYXZ);

    Joint joint_rot_y(SpatialVector(0., 1., 0., 0., 0., 0.));

    model_emulated.appendBody(SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()), joint_rot_y, body);
    unsigned int multdof_body_id_emulated = model_emulated.appendBody(SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()), joint_emulated, body);
    model_emulated.appendBody(SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()), joint_emulated, body);
    model_emulated.addBody(multdof_body_id_emulated, SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()), joint_rot_y, body);
    model_emulated.appendBody(SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()), joint_emulated, body);

    model_3dof.appendBody(SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()), joint_rot_y, body);
    unsigned int multdof_body_id_3dof = model_3dof.appendBody(SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()), joint_3dof, body);
    model_3dof.appendBody(SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()), joint_3dof, body);
    model_3dof.addBody(multdof_body_id_3dof, SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()), joint_rot_y, body);
    model_3dof.appendBody(SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()), joint_3dof, body);

    VectorNd q(VectorNd::Zero(model_emulated.q_size));
    VectorNd qdot(VectorNd::Zero(model_emulated.qdot_size));
    VectorNd qddot_emulated(VectorNd::Zero(model_emulated.qdot_size));
    VectorNd qddot_3dof(VectorNd::Zero(model_emulated.qdot_size));
    VectorNd tau(VectorNd::Zero(model_emulated.qdot_size));

    for (int i = 0; i < q.size(); i++)
    {
        q[i] = 1.1 * (static_cast<double>(i + 1));
        qdot[i] = 0.55 * (static_cast<double>(i + 1));
        qddot_emulated[i] = 0.23 * (static_cast<double>(i + 1));
        qddot_3dof[i] = 0.22 * (static_cast<double>(i + 1));
        tau[i] = 2.1 * (static_cast<double>(i + 1));
    }

    MatrixNd H_emulated(MatrixNd::Zero(q.size(), q.size()));
    MatrixNd H_3dof(MatrixNd::Zero(q.size(), q.size()));

    compositeRigidBodyAlgorithm(model_emulated, q, H_emulated);
    compositeRigidBodyAlgorithm(model_3dof, q, H_3dof);

    VectorNd b(VectorNd::Zero(q.size()));
    VectorNd x_emulated(VectorNd::Zero(q.size()));
    VectorNd x_3dof(VectorNd::Zero(q.size()));

    for (unsigned int i = 0; i < b.size(); i++)
    {
        b[i] = static_cast<double>(i + 1) * 2.152;
    }
    b = H_emulated * b;

    SparseFactorizeLTL(model_emulated, H_emulated);
    SparseFactorizeLTL(model_3dof, H_3dof);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(H_emulated, H_3dof, unit_test_utils::E_MINUS_12));

    x_emulated = b;
    SparseSolveLx(model_emulated, H_emulated, x_emulated);
    x_3dof = b;
    SparseSolveLx(model_3dof, H_3dof, x_3dof);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(x_emulated, x_3dof, 1.0e-9));

    x_emulated = b;
    SparseSolveLTx(model_emulated, H_emulated, x_emulated);
    x_3dof = b;
    SparseSolveLTx(model_3dof, H_3dof, x_3dof);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(x_emulated, x_3dof, 1.0e-9));
}

TEST_F(FloatingBase12DoF, TestSparseFactorizationMultiDofAndFixed)
{
    Model model_emulated;
    Model model_3dof;

    Body body(1., Vector3d(1., 2., 1.), Matrix3d(1., 0., 0, 0., 1., 0., 0., 0., 1.));
    Joint joint_emulated(SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.), SpatialVector(0., 0., 1., 0., 0., 0.));
    Joint joint_3dof(JointTypeEulerYXZ);

    Joint joint_rot_y(SpatialVector(0., 1., 0., 0., 0., 0.));

    SpatialTransform translate_x(Matrix3d::Identity(), Vector3d(1., 0., 0.));

    model_emulated.appendBody(SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()), joint_rot_y, body);
    unsigned int multdof_body_id_emulated = model_emulated.appendBody(translate_x, joint_emulated, body);
    model_emulated.appendBody(translate_x, joint_emulated, body);
    model_emulated.addBody(multdof_body_id_emulated, translate_x, Joint(JointTypeFixed), body);
    model_emulated.appendBody(translate_x, joint_emulated, body);

    model_3dof.appendBody(SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()), joint_rot_y, body);
    unsigned int multdof_body_id_3dof = model_3dof.appendBody(translate_x, joint_3dof, body);
    model_3dof.appendBody(translate_x, joint_3dof, body);
    model_3dof.addBody(multdof_body_id_3dof, translate_x, Joint(JointTypeFixed), body);
    model_3dof.appendBody(translate_x, joint_3dof, body);

    VectorNd q(VectorNd::Zero(model_emulated.q_size));
    VectorNd qdot(VectorNd::Zero(model_emulated.qdot_size));
    VectorNd qddot_emulated(VectorNd::Zero(model_emulated.qdot_size));
    VectorNd qddot_3dof(VectorNd::Zero(model_emulated.qdot_size));
    VectorNd tau(VectorNd::Zero(model_emulated.qdot_size));

    for (int i = 0; i < q.size(); i++)
    {
        q[i] = 1.1 * (static_cast<double>(i + 1));
        qdot[i] = 0.55 * (static_cast<double>(i + 1));
        qddot_emulated[i] = 0.23 * (static_cast<double>(i + 1));
        qddot_3dof[i] = 0.22 * (static_cast<double>(i + 1));
        tau[i] = 2.1 * (static_cast<double>(i + 1));
    }

    MatrixNd H_emulated(MatrixNd::Zero(q.size(), q.size()));
    MatrixNd H_3dof(MatrixNd::Zero(q.size(), q.size()));

    compositeRigidBodyAlgorithm(model_emulated, q, H_emulated);
    compositeRigidBodyAlgorithm(model_3dof, q, H_3dof);

    VectorNd b(VectorNd::Zero(q.size()));
    VectorNd x_emulated(VectorNd::Zero(q.size()));
    VectorNd x_3dof(VectorNd::Zero(q.size()));

    for (unsigned int i = 0; i < b.size(); i++)
    {
        b[i] = static_cast<double>(i + 1) * 2.152;
    }
    b = H_emulated * b;

    SparseFactorizeLTL(model_emulated, H_emulated);
    SparseFactorizeLTL(model_3dof, H_3dof);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(H_emulated, H_3dof, unit_test_utils::E_MINUS_12));

    x_emulated = b;
    SparseSolveLx(model_emulated, H_emulated, x_emulated);
    x_3dof = b;
    SparseSolveLx(model_3dof, H_3dof, x_3dof);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(x_emulated, x_3dof, 1.0e-9));

    x_emulated = b;
    SparseSolveLTx(model_emulated, H_emulated, x_emulated);
    x_3dof = b;
    SparseSolveLTx(model_3dof, H_3dof, x_3dof);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(x_emulated, x_3dof, 1.0e-9));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
