// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>

#include "UnitTestUtils.hpp"
#include "Human36Fixture.hpp"

#include "rdl_dynamics/Contacts.hpp"
#include "rdl_dynamics/Dynamics.hpp"
#include "rdl_dynamics/Kinematics.hpp"

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

struct RbdlimInverseDynamicsFixture : public testing::Test
{
    RbdlimInverseDynamicsFixture()
    {
    }

    void SetUp()
    {
        model = new Model;
        model->gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);
    }

    void TearDown()
    {
        EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(*model));
        delete model;
    }

    Model* model;
};

TEST_F(RbdlimInverseDynamicsFixture, TestInverseForwardDynamicsFloatingBase)
{
    Body base_body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));

    model->addBody(0, SpatialTransform(),
                   Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                         SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                   base_body);

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd Tau = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd TauInv = VectorNd::Constant((size_t)model->dof_count, 0.);

    Q[0] = 1.1;
    Q[1] = 1.2;
    Q[2] = 1.3;
    Q[3] = 0.1;
    Q[4] = 0.2;
    Q[5] = 0.3;

    QDot[0] = 1.1;
    QDot[1] = -1.2;
    QDot[2] = 1.3;
    QDot[3] = -0.1;
    QDot[4] = 0.2;
    QDot[5] = -0.3;

    Tau[0] = 2.1;
    Tau[1] = 2.2;
    Tau[2] = 2.3;
    Tau[3] = 1.1;
    Tau[4] = 1.2;
    Tau[5] = 1.3;

    forwardDynamics(*model, Q, QDot, Tau, QDDot);
    inverseDynamics(*model, Q, QDot, QDDot, TauInv);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(Tau, TauInv, unit_test_utils::E_MINUS_14));
}

TEST_F(RbdlimInverseDynamicsFixture, TestInverseForwardDynamicsFloatingBaseNoKinematics)
{
    Body base_body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));

    model->addBody(0, SpatialTransform(),
                   Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                         SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                   base_body);

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd Tau = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd TauInv = VectorNd::Constant((size_t)model->dof_count, 0.);

    Q[0] = 1.1;
    Q[1] = 1.2;
    Q[2] = 1.3;
    Q[3] = 0.1;
    Q[4] = 0.2;
    Q[5] = 0.3;

    QDot[0] = 1.1;
    QDot[1] = -1.2;
    QDot[2] = 1.3;
    QDot[3] = -0.1;
    QDot[4] = 0.2;
    QDot[5] = -0.3;

    Tau[0] = 2.1;
    Tau[1] = 2.2;
    Tau[2] = 2.3;
    Tau[3] = 1.1;
    Tau[4] = 1.2;
    Tau[5] = 1.3;

    updateKinematicsCustom(*model, &Q, &QDot, &QDDot);
    forwardDynamics(*model, Q, QDot, Tau, QDDot, nullptr, false);
    inverseDynamics(*model, Q, QDot, QDDot, TauInv);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(Tau, TauInv, unit_test_utils::E_MINUS_14));
}

TEST_F(Human36, TestInverseForwardDynamicsFloatingBaseExternalForces)
{
    randomizeStates();

    MatrixNd M(model_emulated->qdot_size, model_emulated->qdot_size);
    VectorNd N(model_emulated->qdot_size);
    M.setZero();
    N.setZero();

    unsigned int foot_r_id = model_emulated->GetBodyId("foot_r");
    SpatialForce F(model_emulated->bodyFrames[foot_r_id], -1., 2., 3.1, 82., -23., 500.1);

    MatrixNd J_r_foot(6, model_emulated->qdot_size);
    J_r_foot.setZero();

    updateKinematics(*model_emulated, q, qdot, qddot);
    calcBodySpatialJacobian(*model_emulated, q, foot_r_id, J_r_foot, false);
    compositeRigidBodyAlgorithm(*model_emulated, q, M);
    nonlinearEffects(*model_emulated, q, qdot, N);

    VectorNd tau_cf(model_emulated->qdot_size), tau_id(model_emulated->qdot_size);
    tau_cf.setZero();

    tau_cf = M * qddot + N - J_r_foot.transpose() * F;

    std::shared_ptr<Math::SpatialForceV> f_ext(new Math::SpatialForceV(model_emulated->mBodies.size()));

    for (unsigned int i = 0; i < model_emulated->mBodies.size(); ++i)
    {
        (*f_ext)[i].setIncludingFrame(model_emulated->bodyFrames[i], 0., 0., 0., 0., 0., 0.);
        if (i == foot_r_id)
        {
            (*f_ext)[i].fx() = F.fx();
            (*f_ext)[i].fy() = F.fy();
            (*f_ext)[i].fz() = F.fz();
            (*f_ext)[i].mx() = F.mx();
            (*f_ext)[i].my() = F.my();
            (*f_ext)[i].mz() = F.mz();
        }
    }

    inverseDynamics(*model_emulated, q, qdot, qddot, tau_id, f_ext.get());

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(tau_id, tau_cf, 1e-11));
}

TEST_F(Human36, TestInverseForwardDynamicsFloatingBaseExternalForcesNoKinematics)
{
    randomizeStates();

    MatrixNd M(model_emulated->qdot_size, model_emulated->qdot_size);
    VectorNd N(model_emulated->qdot_size);
    M.setZero();
    N.setZero();

    unsigned int foot_r_id = model_emulated->GetBodyId("foot_r");
    SpatialForce F(model_emulated->bodyFrames[foot_r_id], -1., 2., 3.1, 82., -23., 500.1);

    MatrixNd J_r_foot(6, model_emulated->qdot_size);
    J_r_foot.setZero();

    updateKinematics(*model_emulated, q, qdot, qddot);
    calcBodySpatialJacobian(*model_emulated, q, foot_r_id, J_r_foot, false);
    compositeRigidBodyAlgorithm(*model_emulated, q, M);
    nonlinearEffects(*model_emulated, q, qdot, N);

    VectorNd tau_cf(model_emulated->qdot_size), tau_id(model_emulated->qdot_size);
    tau_cf.setZero();

    tau_cf = M * qddot + N - J_r_foot.transpose() * F;

    std::shared_ptr<Math::SpatialForceV> f_ext(new Math::SpatialForceV(model_emulated->mBodies.size()));

    for (unsigned int i = 0; i < model_emulated->mBodies.size(); i++)
    {
        (*f_ext)[i].setIncludingFrame(model_emulated->bodyFrames[i], 0., 0., 0., 0., 0., 0.);
        if (i == foot_r_id)
        {
            (*f_ext)[i].fx() = F.fx();
            (*f_ext)[i].fy() = F.fy();
            (*f_ext)[i].fz() = F.fz();
            (*f_ext)[i].mx() = F.mx();
            (*f_ext)[i].my() = F.my();
            (*f_ext)[i].mz() = F.mz();
        }
    }

    inverseDynamics(*model_emulated, q, qdot, qddot, tau_id, f_ext.get(), false);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(tau_id, tau_cf, 1e-11));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
