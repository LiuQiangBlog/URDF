// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>

#include "UnitTestUtils.hpp"

#include "rdl_dynamics/Model.hpp"
#include "rdl_dynamics/Kinematics.hpp"
#include "rdl_dynamics/Dynamics.hpp"
#include "rdl_dynamics/Contacts.hpp"

#include "Fixtures.hpp"
#include "Human36Fixture.hpp"

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

struct RdlDynamicsFixture : public testing::Test
{
    RdlDynamicsFixture()
    {
        srand(time(nullptr));
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

// Kinda a silly test, but it was a bug, so a test for it anyway.
TEST_F(Human36, TestNonlinearEffects)
{
    randomizeStates();
    q.setZero();
    qddot.setZero();

    VectorNd tau1 = VectorNd::Zero(model->qdot_size);
    VectorNd tau2 = VectorNd::Zero(model->qdot_size);

    nonlinearEffects(*model, q, qdot, tau2);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model->a[0], SpatialVector(0., 0., 0., 0., 0., 9.81), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model->a[1], SpatialVector(0., 0., 0., 0., 0., 9.81), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlDynamicsFixture, TestCalcDynamicSingleChain)
{
    Body body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
    Joint joint(SpatialVector(0., 0., 1., 0., 0., 0.));

    model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body);

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd Tau = VectorNd::Constant((size_t)model->dof_count, 0.);

    forwardDynamics(*model, Q, QDot, Tau, QDDot);

    EXPECT_EQ(-4.905, QDDot[0]);
}

TEST_F(RdlDynamicsFixture, TestCalcDynamicSpatialInertiaSingleChain)
{
    // This function checks the value for a non-trivial spatial inertia
    Body body(1., Vector3d(1.5, 1., 1.), Vector3d(2., 2., 3.));

    Joint joint(SpatialVector(0., 0., 1., 0., 0., 0.));

    model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body);

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd Tau = VectorNd::Constant((size_t)model->dof_count, 0.);

    forwardDynamics(*model, Q, QDot, Tau, QDDot);

    EXPECT_EQ(-2.3544, QDDot[0]);
}

TEST_F(RdlDynamicsFixture, TestCalcDynamicDoubleChain)
{
    Body body_a(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
    Joint joint_a(SpatialVector(0., 0., 1., 0., 0., 0.));

    model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

    Body body_b(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
    Joint joint_b(SpatialVector(0., 0., 1., 0., 0., 0.));

    model->addBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd Tau = VectorNd::Constant((size_t)model->dof_count, 0.);

    //	cout << "--- Double Chain ---" << endl;

    forwardDynamics(*model, Q, QDot, Tau, QDDot);

    EXPECT_NEAR(-5.88600000000000E+00, QDDot[0], unit_test_utils::E_MINUS_13);
    EXPECT_NEAR(3.92400000000000E+00, QDDot[1], unit_test_utils::E_MINUS_13);
}

TEST_F(RdlDynamicsFixture, TestCalcDynamicTripleChain)
{
    Body body_a(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
    Joint joint_a(SpatialVector(0., 0., 1., 0., 0., 0.));

    model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

    Body body_b(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
    Joint joint_b(SpatialVector(0., 0., 1., 0., 0., 0.));

    model->addBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

    Body body_c(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
    Joint joint_c(SpatialVector(0., 0., 1., 0., 0., 0.));

    model->addBody(2, Xtrans(Vector3d(1., 0., 0.)), joint_c, body_c);

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd Tau = VectorNd::Constant((size_t)model->dof_count, 0.);

    // cout << "--- Triple Chain ---" << endl;

    forwardDynamics(*model, Q, QDot, Tau, QDDot);

    EXPECT_NEAR(-6.03692307692308E+00, QDDot[0], unit_test_utils::E_MINUS_13);
    EXPECT_NEAR(3.77307692307692E+00, QDDot[1], unit_test_utils::E_MINUS_13);
    EXPECT_NEAR(1.50923076923077E+00, QDDot[2], unit_test_utils::E_MINUS_13);
}

TEST_F(RdlDynamicsFixture, TestCalcDynamicDoubleChain3D)
{
    Body body_a(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
    Joint joint_a(SpatialVector(0., 0., 1., 0., 0., 0.));

    model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

    Body body_b(1., Vector3d(0., 1., 0.), Vector3d(1., 1., 1.));
    Joint joint_b(SpatialVector(0., 1., 0., 0., 0., 0.));

    model->addBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd Tau = VectorNd::Constant((size_t)model->dof_count, 0.);

    // cout << "--- Double Chain 3D ---" << endl;

    forwardDynamics(*model, Q, QDot, Tau, QDDot);

    // cout << LogOutput.str() << endl;

    EXPECT_NEAR(-3.92400000000000E+00, QDDot[0], unit_test_utils::E_MINUS_13);
    EXPECT_NEAR(0.00000000000000E+00, QDDot[1], unit_test_utils::E_MINUS_13);
}

TEST_F(RdlDynamicsFixture, TestCalcDynamicSimpleTree3D)
{
    Body body_a(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
    Joint joint_a(SpatialVector(0., 0., 1., 0., 0., 0.));

    model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

    Body body_b1(1., Vector3d(0., 1., 0.), Vector3d(1., 1., 1.));
    Joint joint_b1(SpatialVector(0., 1., 0., 0., 0., 0.));

    model->addBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b1, body_b1);

    Body body_c1(1., Vector3d(0., 0., 1.), Vector3d(1., 1., 1.));
    Joint joint_c1(SpatialVector(1., 0., 0., 0., 0., 0.));

    model->addBody(2, Xtrans(Vector3d(0., 1., 0.)), joint_c1, body_c1);

    Body body_b2(1., Vector3d(0., 1., 0.), Vector3d(1., 1., 1.));
    Joint joint_b2(SpatialVector(0., 1., 0., 0., 0., 0.));

    model->addBody(1, Xtrans(Vector3d(-0.5, 0., 0.)), joint_b2, body_b2);

    Body body_c2(1., Vector3d(0., 0., 1.), Vector3d(1., 1., 1.));
    Joint joint_c2(SpatialVector(1., 0., 0., 0., 0., 0.));

    model->addBody(4, Xtrans(Vector3d(0., -0.5, 0.)), joint_c2, body_c2);

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd QDDot = VectorNd::Constant((size_t)model->dof_count, 0.);
    VectorNd Tau = VectorNd::Constant((size_t)model->dof_count, 0.);

    // cout << "--- SimpleTree ---" << endl;

    forwardDynamics(*model, Q, QDot, Tau, QDDot);

    // cout << LogOutput.str() << endl;

    EXPECT_NEAR(-1.60319066147860E+00, QDDot[0], unit_test_utils::E_MINUS_13);
    EXPECT_NEAR(-5.34396887159533E-01, QDDot[1], unit_test_utils::E_MINUS_13);
    EXPECT_NEAR(4.10340466926070E+00, QDDot[2], unit_test_utils::E_MINUS_13);
    EXPECT_NEAR(2.67198443579767E-01, QDDot[3], unit_test_utils::E_MINUS_13);
    EXPECT_NEAR(5.30579766536965E+00, QDDot[4], unit_test_utils::E_MINUS_13);
}

TEST_F(RdlDynamicsFixture, TestForwardDynamicsLagrangian)
{
    Model model;
    Body base_body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));

    model.addBody(0, SpatialTransform(),
                  Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                        SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                  base_body);

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Zero(model.dof_count);
    VectorNd QDot = VectorNd::Zero(model.dof_count);
    VectorNd Tau = VectorNd::Zero(model.dof_count);

    VectorNd QDDot_aba = VectorNd::Zero(model.dof_count);
    VectorNd QDDot_lagrangian = VectorNd::Zero(model.dof_count);

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

    RobotDynamics::Math::MatrixNd H(model.dof_count, model.dof_count);
    RobotDynamics::Math::VectorNd C(model.dof_count);

    forwardDynamics(model, Q, QDot, Tau, QDDot_aba);
    forwardDynamicsLagrangian(model, Q, QDot, Tau, QDDot_lagrangian, H, C);

    EXPECT_EQ(QDDot_aba.size(), QDDot_lagrangian.size());
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_aba, QDDot_lagrangian, unit_test_utils::E_MINUS_13));

    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));
}

TEST_F(RdlDynamicsFixture, TestForwardDynamicsLagrangianNoKinematics)
{
    Model model;
    Body base_body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));

    model.addBody(0, SpatialTransform(),
                  Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                        SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                  base_body);

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Zero(model.dof_count);
    VectorNd QDot = VectorNd::Zero(model.dof_count);
    VectorNd Tau = VectorNd::Zero(model.dof_count);

    VectorNd QDDot_aba = VectorNd::Zero(model.dof_count);
    VectorNd QDDot_lagrangian = VectorNd::Zero(model.dof_count);

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

    RobotDynamics::Math::MatrixNd H(model.dof_count, model.dof_count);
    RobotDynamics::Math::VectorNd C(model.dof_count);

    updateKinematicsCustom(model, &Q, &QDot, nullptr);
    forwardDynamicsLagrangian(model, Q, QDot, Tau, QDDot_lagrangian, H, C, Math::LinearSolverColPivHouseholderQR, nullptr, false);
    forwardDynamics(model, Q, QDot, Tau, QDDot_aba);

    EXPECT_EQ(QDDot_aba.size(), QDDot_lagrangian.size());
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_aba, QDDot_lagrangian, unit_test_utils::E_MINUS_13));

    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));
}

/*
 * A simple test for a model with 3 rotational dof. The reference value was
 * computed with Featherstones spatial_v1 code. This test was written
 * because my benchmark tool showed up inconsistencies, however this was
 * due to the missing gravity term. But as the test now works, I just leave
 * it here.
 */
TEST_F(RdlDynamicsFixture, TestForwardDynamics3DoFModel)
{
    Model model;

    model.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

    Body null_body(0., Vector3d(0., 0., 0.), Vector3d(0., 0., 0.));
    null_body.mIsVirtual = true;
    Body base_body(1., Vector3d(0., 0.5, 0.), Vector3d(1., 1., 1.));

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));
    Joint joint_rot_y(SpatialVector(0., 1., 0., 0., 0., 0.));
    Joint joint_rot_x(SpatialVector(1., 0., 0., 0., 0., 0.));

    unsigned int base_id_rot_z, base_id_rot_y;

    // we can reuse both bodies and joints as they are copied
    base_id_rot_z = model.addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, null_body);
    base_id_rot_y = model.addBody(base_id_rot_z, Xtrans(Vector3d(0., 0., 0.)), joint_rot_y, null_body);
    model.addBody(base_id_rot_y, Xtrans(Vector3d(0., 0., 0.)), joint_rot_x, base_body);

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Constant((size_t)model.dof_count, 0.);
    VectorNd QDot = VectorNd::Constant((size_t)model.dof_count, 0.);
    VectorNd Tau = VectorNd::Constant((size_t)model.dof_count, 0.);

    VectorNd QDDot = VectorNd::Constant((size_t)model.dof_count, 0.);
    VectorNd QDDot_ref = VectorNd::Constant((size_t)model.dof_count, 0.);

    Q[0] = 1.;

    forwardDynamics(model, Q, QDot, Tau, QDDot);

    QDDot_ref[0] = 3.301932144386186;

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_ref, QDDot, unit_test_utils::E_MINUS_13));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));
}

/*
 * Another simple 3 dof model test which showed some problems when
 * computing forward dynamics with the Lagrangian formulation. A proplem
 * occured as the CRBA does not update the kinematics of the model, hence
 * invalid body transformations and joint axis were used in the CRBA.
 * Running the CRBA after the InverseDynamics calculation fixes this. This
 * test ensures that the error does not happen when calling ForwardLagrangian.
 */
TEST_F(RdlDynamicsFixture, TestForwardDynamics3DoFModelLagrangian)
{
    Model model;

    model.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

    Body null_body(0., Vector3d(0., 0., 0.), Vector3d(0., 0., 0.));
    null_body.mIsVirtual = true;
    Body base_body(1., Vector3d(0., 0.5, 0.), Vector3d(1., 1., 1.));

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));
    Joint joint_rot_y(SpatialVector(0., 1., 0., 0., 0., 0.));
    Joint joint_rot_x(SpatialVector(1., 0., 0., 0., 0., 0.));

    unsigned int base_id_rot_z, base_id_rot_y;

    // we can reuse both bodies and joints as they are copied
    base_id_rot_z = model.addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, null_body);
    base_id_rot_y = model.addBody(base_id_rot_z, Xtrans(Vector3d(0., 0., 0.)), joint_rot_y, null_body);
    model.addBody(base_id_rot_y, Xtrans(Vector3d(0., 0., 0.)), joint_rot_x, base_body);

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Constant((size_t)model.dof_count, 0.);
    VectorNd QDot = VectorNd::Constant((size_t)model.dof_count, 0.);
    VectorNd Tau = VectorNd::Constant((size_t)model.dof_count, 0.);

    VectorNd QDDot_ab = VectorNd::Constant((size_t)model.dof_count, 0.);
    VectorNd QDDot_lagrangian = VectorNd::Constant((size_t)model.dof_count, 0.);

    Q[1] = 1.;

    Q[0] = 0.;
    Q[1] = 1.;
    Q[2] = 0.;
    RobotDynamics::Math::MatrixNd H(model.dof_count, model.dof_count);
    RobotDynamics::Math::VectorNd C(model.dof_count);

    forwardDynamicsLagrangian(model, Q, QDot, Tau, QDDot_lagrangian, H, C);
    forwardDynamics(model, Q, QDot, Tau, QDDot_ab);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_ab, QDDot_lagrangian, unit_test_utils::E_MINUS_13));

    Q[0] = 0.;
    Q[1] = 0.;
    Q[2] = 1.;
    forwardDynamicsLagrangian(model, Q, QDot, Tau, QDDot_lagrangian, H, C);
    forwardDynamics(model, Q, QDot, Tau, QDDot_ab);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_ab, QDDot_lagrangian, unit_test_utils::E_MINUS_13));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));
}

/*
 * This is a test for a model where I detected incosistencies between the
 * Lagragian method and the ABA.
 */
TEST_F(RdlDynamicsFixture, TestForwardDynamicsTwoLegModelLagrangian)
{
    Model* model = NULL;

    unsigned int hip_id, foot_right_id, foot_left_id;
    Body hip_body, upper_leg_right_body, lower_leg_right_body, foot_right_body, upper_leg_left_body, lower_leg_left_body, foot_left_body;

    Joint joint_rot_z, joint_rot_y, joint_rot_x;
    Joint joint_trans_z, joint_trans_y, joint_trans_x;

    ConstraintSet CS_right;
    ConstraintSet CS_left;
    ConstraintSet CS_both;

    model = new Model();

    model->gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

    joint_rot_z = Joint(SpatialVector(0., 0., 1., 0., 0., 0.));
    joint_rot_y = Joint(SpatialVector(0., 1., 0., 0., 0., 0.));
    joint_rot_x = Joint(SpatialVector(1., 0., 0., 0., 0., 0.));

    joint_trans_z = Joint(SpatialVector(0., 0., 0., 0., 0., 1.));
    joint_trans_y = Joint(SpatialVector(0., 0., 0., 0., 1., 0.));
    joint_trans_x = Joint(SpatialVector(0., 0., 0., 1., 0., 0.));

    Body null_body(0., Vector3d(0., 0., 0.), Vector3d(0., 0., 0.));
    null_body.mIsVirtual = true;

    // hip
    hip_body = Body(1., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.));

    // lateral right
    upper_leg_right_body = Body(1., Vector3d(0., -0.25, 0.), Vector3d(1., 1., 1.));
    lower_leg_right_body = Body(1., Vector3d(0., -0.25, 0.), Vector3d(1., 1., 1.));
    foot_right_body = Body(1., Vector3d(0.15, -0.1, 0.), Vector3d(1., 1., 1.));

    // lateral left
    upper_leg_left_body = Body(1., Vector3d(0., -0.25, 0.), Vector3d(1., 1., 1.));
    lower_leg_left_body = Body(1., Vector3d(0., -0.25, 0.), Vector3d(1., 1., 1.));
    foot_left_body = Body(1., Vector3d(0.15, -0.1, 0.), Vector3d(1., 1., 1.));

    // temporary value to store most recent body id
    unsigned int temp_id;

    // add hip to the model (planar, 3 DOF)
    temp_id = model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_trans_x, null_body);
    temp_id = model->addBody(temp_id, Xtrans(Vector3d(0., 0., 0.)), joint_trans_y, null_body);
    hip_id = model->addBody(temp_id, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, hip_body);

    //
    // right leg
    //

    // add right upper leg
    temp_id = model->addBody(hip_id, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, upper_leg_right_body);

    // add the right lower leg (only one DOF)
    temp_id = model->addBody(temp_id, Xtrans(Vector3d(0., -0.5, 0.)), joint_rot_z, lower_leg_right_body);

    // add the right foot (1 DOF)
    temp_id = model->addBody(temp_id, Xtrans(Vector3d(0., -0.5, 0.)), joint_rot_z, foot_right_body);
    foot_right_id = temp_id;

    //
    // left leg
    //

    // add left upper leg
    temp_id = model->addBody(hip_id, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, upper_leg_left_body);

    // add the left lower leg (only one DOF)
    temp_id = model->addBody(temp_id, Xtrans(Vector3d(0., -0.5, 0.)), joint_rot_z, lower_leg_left_body);

    // add the left foot (1 DOF)
    temp_id = model->addBody(temp_id, Xtrans(Vector3d(0., -0.5, 0.)), joint_rot_z, foot_left_body);
    foot_left_id = temp_id;

    // contact data
    CS_right.addConstraint(foot_right_id, Vector3d(0., 0., 0.), Vector3d(1., 0., 0.), "foot_right_x");
    CS_right.addConstraint(foot_right_id, Vector3d(0., 0., 0.), Vector3d(0., 1., 0.), "foot_right_y");
    CS_right.addConstraint(foot_right_id, Vector3d(0., 0., 0.), Vector3d(0., 0., 1.), "foot_right_z");

    CS_left.addConstraint(foot_left_id, Vector3d(0., 0., 0.), Vector3d(1., 0., 0.), "foot_left_x");
    CS_left.addConstraint(foot_left_id, Vector3d(0., 0., 0.), Vector3d(0., 1., 0.), "foot_left_y");
    CS_left.addConstraint(foot_left_id, Vector3d(0., 0., 0.), Vector3d(0., 0., 1.), "foot_left_z");

    CS_both.addConstraint(foot_right_id, Vector3d(0., 0., 0.), Vector3d(1., 0., 0.), "foot_right_x");
    CS_both.addConstraint(foot_right_id, Vector3d(0., 0., 0.), Vector3d(0., 1., 0.), "foot_right_y");
    CS_both.addConstraint(foot_right_id, Vector3d(0., 0., 0.), Vector3d(0., 0., 1.), "foot_right_z");
    CS_both.addConstraint(foot_left_id, Vector3d(0., 0., 0.), Vector3d(1., 0., 0.), "foot_left_x");
    CS_both.addConstraint(foot_left_id, Vector3d(0., 0., 0.), Vector3d(0., 1., 0.), "foot_left_y");
    CS_both.addConstraint(foot_left_id, Vector3d(0., 0., 0.), Vector3d(0., 0., 1.), "foot_left_z");

    CS_right.bind(*model);
    CS_left.bind(*model);
    CS_both.bind(*model);

    VectorNd Q(model->dof_count);
    VectorNd QDot(model->dof_count);
    VectorNd Tau(model->dof_count);
    VectorNd QDDot(model->dof_count);
    VectorNd QDDotABA(model->dof_count);

    Q[0] = 0.8;
    Q[1] = -7.76326e-06;
    Q[2] = -1.58205e-07;
    Q[3] = 1.57391e-07;
    Q[4] = -1.03029e-09;
    Q[5] = 7.92143e-08;
    Q[6] = 1.57391e-07;
    Q[7] = -1.03029e-09;
    Q[8] = 7.92143e-08;

    QDot[0] = -1.77845e-06;
    QDot[1] = -0.00905283;
    QDot[2] = -0.000184484;
    QDot[3] = 0.000183536;
    QDot[4] = -1.20144e-06;
    QDot[5] = 9.23727e-05;
    QDot[6] = 0.000183536;
    QDot[7] = -1.20144e-06;
    QDot[8] = 9.23727e-05;

    Tau[0] = 0;
    Tau[1] = 0;
    Tau[2] = 0;
    Tau[3] = 0.1;
    Tau[4] = 0.1;
    Tau[5] = 0.1;
    Tau[6] = 0.1;
    Tau[7] = 0.1;
    Tau[8] = 0.1;

    RobotDynamics::Math::MatrixNd H(model->dof_count, model->dof_count);
    RobotDynamics::Math::VectorNd C(model->dof_count);
    H.setZero();
    C.setZero();

    forwardDynamics(*model, Q, QDot, Tau, QDDotABA);
    forwardDynamicsLagrangian(*model, Q, QDot, Tau, QDDot, H, C);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDotABA, QDDot, unit_test_utils::E_MINUS_13));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(*model));

    // run it again to make sure the calculations give the same results and
    // no invalid state information lingering in the model structure is being used
    forwardDynamics(*model, Q, QDot, Tau, QDDotABA);
    forwardDynamicsLagrangian(*model, Q, QDot, Tau, QDDot, H, C);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDotABA, QDDot, unit_test_utils::E_MINUS_13));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(*model));

    delete model;
}

TEST_F(FixedAndMovableJoint, TestInverseDynamicsFixedJoint)
{
    Q_fixed[0] = 1.1;
    Q_fixed[1] = 2.2;

    QDot_fixed[0] = -3.2;
    QDot_fixed[1] = -2.3;

    QDDot_fixed[0] = 1.2;
    QDDot_fixed[1] = 2.1;

    Q = CreateDofVectorFromReducedVector(Q_fixed);
    QDot = CreateDofVectorFromReducedVector(QDot_fixed);
    QDDot = CreateDofVectorFromReducedVector(QDDot_fixed);

    inverseDynamics(*model_movable, Q, QDot, QDDot, Tau);
    inverseDynamics(*model_fixed, Q_fixed, QDot_fixed, QDDot_fixed, Tau_fixed);

    VectorNd Tau_2dof(2);
    Tau_2dof[0] = Tau[0];
    Tau_2dof[1] = Tau[2];

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(Tau_2dof, Tau_fixed, unit_test_utils::E_MINUS_13));
}

TEST_F(FloatingBase12DoF, TestForwardDynamicsLagrangianPrealloc)
{
    for (unsigned int i = 0; i < model->dof_count; i++)
    {
        Q[i] = static_cast<double>(i + 1) * 0.1;
        QDot[i] = static_cast<double>(i + 1) * 1.1;
        Tau[i] = static_cast<double>(i + 1) * -1.2;
    }

    MatrixNd H(MatrixNd::Zero(model->dof_count, model->dof_count));
    VectorNd C(VectorNd::Zero(model->dof_count));

    forwardDynamicsLagrangian(*model, Q, QDot, Tau, QDDot, H, C, Math::LinearSolverPartialPivLU, NULL);

    H = MatrixNd::Zero(model->dof_count, model->dof_count);
    C = VectorNd::Zero(model->dof_count);
    VectorNd QDDot_prealloc(VectorNd::Zero(model->dof_count));
    forwardDynamicsLagrangian(*model, Q, QDot, Tau, QDDot_prealloc, H, C, Math::LinearSolverPartialPivLU, NULL);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEq(QDDot, QDDot_prealloc));
}

TEST_F(FixedBase3DoF, SolveMInvTimesTau)
{
    for (unsigned int i = 0; i < model->dof_count; i++)
    {
        Q[i] = rand() / static_cast<double>(RAND_MAX);
        QDot[i] = rand() / static_cast<double>(RAND_MAX);
        Tau[i] = rand() / static_cast<double>(RAND_MAX);
    }

    MatrixNd M(MatrixNd::Zero(model->dof_count, model->dof_count));
    compositeRigidBodyAlgorithm(*model, Q, M);

    VectorNd qddot_solve_llt = M.llt().solve(Tau);

    VectorNd qddot_minv(Q);
    calcMInvTimesTau(*model, Q, Tau, qddot_minv);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_solve_llt, qddot_minv, unit_test_utils::E_MINUS_13));
}

TEST_F(FixedBase3DoF, SolveMInvTimesTauNoKinematics)
{
    for (unsigned int i = 0; i < model->dof_count; i++)
    {
        Q[i] = rand() / static_cast<double>(RAND_MAX);
        QDot[i] = rand() / static_cast<double>(RAND_MAX);
        Tau[i] = rand() / static_cast<double>(RAND_MAX);
    }

    updateKinematicsCustom(*model, &Q, &QDot, nullptr);
    MatrixNd M(MatrixNd::Zero(model->dof_count, model->dof_count));
    compositeRigidBodyAlgorithm(*model, Q, M, false);

    VectorNd qddot_solve_llt = M.llt().solve(Tau);

    VectorNd qddot_minv(Q);
    calcMInvTimesTau(*model, Q, Tau, qddot_minv, false);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_solve_llt, qddot_minv, unit_test_utils::E_MINUS_13));

    M.setZero();
    compositeRigidBodyAlgorithm(*model, Q, M, false);

    qddot_solve_llt.setZero();

    qddot_solve_llt = M.llt().solve(Tau);

    qddot_minv.setZero();
    calcMInvTimesTau(*model, Q, Tau, qddot_minv, false);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_solve_llt, qddot_minv, unit_test_utils::E_MINUS_13));
}

TEST_F(FixedBase3DoF, SolveMInvTimesTauReuse)
{
    for (unsigned int i = 0; i < model->dof_count; i++)
    {
        Q[i] = rand() / static_cast<double>(RAND_MAX);
        Tau[i] = rand() / static_cast<double>(RAND_MAX);
    }

    MatrixNd M(MatrixNd::Zero(model->dof_count, model->dof_count));
    compositeRigidBodyAlgorithm(*model, Q, M);

    VectorNd qddot_solve_llt = M.llt().solve(Tau);

    VectorNd qddot_minv(Q);
    calcMInvTimesTau(*model, Q, Tau, qddot_minv);

    for (unsigned int j = 0; j < 1; j++)
    {
        for (unsigned int i = 0; i < model->dof_count; i++)
        {
            Tau[i] = rand() / static_cast<double>(RAND_MAX);
        }

        compositeRigidBodyAlgorithm(*model, Q, M);
        qddot_solve_llt = M.llt().solve(Tau);

        calcMInvTimesTau(*model, Q, Tau, qddot_minv, false);
        EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_solve_llt, qddot_minv, unit_test_utils::E_MINUS_13));
    }
}

TEST_F(FixedBase3DoF, SolveMInvTimesTauReuseNoKinematics)
{
    for (unsigned int i = 0; i < model->dof_count; i++)
    {
        Q[i] = rand() / static_cast<double>(RAND_MAX);
        QDot[i] = rand() / static_cast<double>(RAND_MAX);
        Tau[i] = rand() / static_cast<double>(RAND_MAX);
    }

    updateKinematicsCustom(*model, &Q, &QDot, nullptr);
    MatrixNd M(MatrixNd::Zero(model->dof_count, model->dof_count));
    compositeRigidBodyAlgorithm(*model, Q, M);

    VectorNd qddot_solve_llt = M.llt().solve(Tau);

    VectorNd qddot_minv(Q);

    for (unsigned int j = 0; j < 1; j++)
    {
        for (unsigned int i = 0; i < model->dof_count; i++)
        {
            Tau[i] = rand() / static_cast<double>(RAND_MAX);
        }

        compositeRigidBodyAlgorithm(*model, Q, M, false);
        qddot_solve_llt = M.llt().solve(Tau);

        calcMInvTimesTau(*model, Q, Tau, qddot_minv, false);
        EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_solve_llt, qddot_minv, unit_test_utils::E_MINUS_13));
    }
}

TEST_F(Human36, GravityEffects)
{
    randomizeStates();

    updateKinematics(*model, q, qdot, qddot);

    RobotDynamics::Math::VectorNd G1 = RobotDynamics::Math::VectorNd::Zero(model->dof_count);
    RobotDynamics::Math::VectorNd G2 = RobotDynamics::Math::VectorNd::Zero(model->dof_count);

    qdot.setZero();
    nonlinearEffects(*model, q, qdot, G1);
    gravityEffects(*model, G2);

    EXPECT_TRUE(G1.isApprox(G2, unit_test_utils::E_MINUS_14));

    updateKinematics(*model_3dof, q, qdot, qddot);

    G1 = RobotDynamics::Math::VectorNd::Zero(model->dof_count);
    G2 = RobotDynamics::Math::VectorNd::Zero(model->dof_count);

    qdot.setZero();
    nonlinearEffects(*model_3dof, q, qdot, G1);
    gravityEffects(*model_3dof, G2);

    EXPECT_TRUE(G1.isApprox(G2, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlDynamicsFixture, gravityWrenchSimple)
{
    RobotDynamics::Body b(1., Vector3d(0., 1., 0.), Vector3d(1., 1., 1.));
    RobotDynamics::Joint j(RobotDynamics::JointType::JointTypeRevoluteX);
    model->gravity = MotionVector(0., 0., 0., 0., 0., -9.81);

    unsigned int id1 = model->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b1");
    (void)id1;
    unsigned int id2 = model->appendBody(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0., 2., 0.)), j, b, "b2");
    RobotDynamics::Math::VectorNd q = RobotDynamics::Math::VectorNd::Zero(model->q_size);
    updateKinematicsCustom(*model, &q, nullptr, nullptr);

    RobotDynamics::Math::SpatialForce wrench;
    calcBodyGravityWrench(*model, id2, wrench);

    EXPECT_TRUE(wrench.isApprox(RobotDynamics::Math::ForceVector(-9.81, 0., 0., 0., 0., -9.81), unit_test_utils::E_MINUS_14));
}

TEST_F(Human36, GravityWrench)
{
    for (unsigned i = 1; i < model->mBodies.size() - 1; i++)
    {
        randomizeStates();
        qdot.setZero();
        updateKinematicsCustom(*model, &q, nullptr, nullptr);

        RobotDynamics::Math::VectorNd G = RobotDynamics::Math::VectorNd::Zero(model->dof_count);

        RobotDynamics::Math::SpatialForce wrench;
        gravityEffects(*model, G);

        calcBodyGravityWrench(*model, i, wrench);

        if (model->mJoints[i].mJointType != JointTypeCustom)
        {
            if (model->mJoints[i].mDoFCount == 1)
            {
                EXPECT_NEAR(G[model->mJoints[i].q_index], -model->S[i].dot(wrench), unit_test_utils::E_MINUS_14);
            }
            else if (model->mJoints[i].mDoFCount == 3)
            {
                G.block<3, 1>(model->mJoints[i].q_index, 0).isApprox(-model->multdof3_S[i].transpose() * wrench, unit_test_utils::E_MINUS_14);
            }
        }
        else if (model->mJoints[i].mJointType == JointTypeCustom)
        {
            unsigned int k = model->mJoints[i].custom_joint_index;
            G.block(model->mJoints[i].q_index, 0, model->mCustomJoints[k]->mDoFCount, 1)
                .isApprox(-model->mCustomJoints[k]->S.transpose() * wrench, unit_test_utils::E_MINUS_14);
        }
    }
}

TEST_F(Human36, TestForwardDynamicsWithExternalForces)
{
    randomizeStates();

    MatrixNd M(model_emulated->qdot_size, model_emulated->qdot_size);
    VectorNd N(model_emulated->qdot_size);
    M.setZero();
    N.setZero();

    unsigned int foot_r_id = model_emulated->GetBodyId("foot_r");
    SpatialForce F(model_emulated->bodyFrames[foot_r_id], 0.1, 0.3, -0.8, 82., -23., 500.1);

    MatrixNd J_r_foot(6, model_emulated->qdot_size);
    J_r_foot.setZero();

    updateKinematics(*model_emulated, q, qdot, qddot);
    calcBodySpatialJacobian(*model_emulated, q, foot_r_id, J_r_foot, false);
    compositeRigidBodyAlgorithm(*model_emulated, q, M);
    nonlinearEffects(*model_emulated, q, qdot, N);

    VectorNd qddot_cf(model_emulated->qdot_size), qddot_fd(model_emulated->qdot_size), qddot_test(model_emulated->qdot_size);
    qddot_cf.setZero();

    qddot_cf = M.inverse() * (tau + J_r_foot.transpose() * F - N);

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

    forwardDynamics(*model_emulated, q, qdot, tau, qddot_fd, f_ext.get());

    if (!unit_test_utils::checkVectorNdEpsilonClose(qddot_cf, qddot_fd, 1e-7))
    {
        std::cout << (qddot_cf - qddot_fd).transpose();
    }

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_cf, qddot_fd, 1e-8));
}

TEST_F(Human36, TestForwardDynamicsWithExternalForcesNoKinematics)
{
    randomizeStates();

    MatrixNd M(model_emulated->qdot_size, model_emulated->qdot_size);
    VectorNd N(model_emulated->qdot_size);
    M.setZero();
    N.setZero();

    unsigned int foot_r_id = model_emulated->GetBodyId("foot_r");
    SpatialForce F(model_emulated->bodyFrames[foot_r_id], 0.1, 0.3, -0.8, 82., -23., 500.1);

    MatrixNd J_r_foot(6, model_emulated->qdot_size);
    J_r_foot.setZero();

    updateKinematics(*model_emulated, q, qdot, qddot);
    calcBodySpatialJacobian(*model_emulated, q, foot_r_id, J_r_foot, false);
    compositeRigidBodyAlgorithm(*model_emulated, q, M, false);
    nonlinearEffects(*model_emulated, q, qdot, N, false);

    VectorNd qddot_cf(model_emulated->qdot_size), qddot_fd(model_emulated->qdot_size), qddot_test(model_emulated->qdot_size);
    qddot_cf.setZero();

    qddot_cf = M.inverse() * (tau + J_r_foot.transpose() * F - N);

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

    forwardDynamics(*model_emulated, q, qdot, tau, qddot_fd, f_ext.get(), false);

    if (!unit_test_utils::checkVectorNdEpsilonClose(qddot_cf, qddot_fd, 1e-7))
    {
        std::cout << (qddot_cf - qddot_fd).transpose();
    }

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_cf, qddot_fd, 1e-8));
}

TEST_F(Human36, TestMInvTimesTau)
{
    randomizeStates();

    MatrixNd M(MatrixNd::Zero(model_3dof->dof_count, model_3dof->dof_count));
    compositeRigidBodyAlgorithm(*model_3dof, q, M);

    VectorNd qddot_solve_llt = M.llt().solve(tau);

    VectorNd qddot_minv(q);
    calcMInvTimesTau(*model_3dof, q, tau, qddot_minv);

    for (unsigned int j = 0; j < 10; j++)
    {
        randomizeStates();

        compositeRigidBodyAlgorithm(*model_3dof, q, M);
        qddot_solve_llt = M.llt().solve(tau);

        calcMInvTimesTau(*model_3dof, q, tau, qddot_minv);

        EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_solve_llt, qddot_minv, unit_test_utils::E_MINUS_13 * qddot_solve_llt.norm()));
    }
}

TEST_F(RdlDynamicsFixture, coriolisEffects)
{
    model->gravity.SpatialVector::set(0., 0., 0., 0., 0., -9.81);

    RobotDynamics::Math::Vector3d b1_center(0.0, 0., 0.1);
    double b1_mass = 0.1;
    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1));
    RobotDynamics::Joint j_x(RobotDynamics::JointTypeRevoluteX);
    RobotDynamics::Joint j_y(RobotDynamics::JointTypeRevoluteY);
    RobotDynamics::Joint j_z(RobotDynamics::JointTypeRevoluteZ);

    model->appendBody(Xtrans(Vector3d(0., 0., 0.)), RobotDynamics::Joint(JointTypeFloatingBase), b, "floating_base");
    model->appendBody(Xtrans(Vector3d(0., 0., 0.)), j_x, b, "body1");
    model->appendBody(Xtrans(Vector3d(1., 0., 0.)), j_y, b, "body2");
    model->appendBody(Xtrans(Vector3d(0.1, 0.2, 0.3)), RobotDynamics::Joint(RobotDynamics::JointTypeEulerZYX), b, "euler_body");
    model->appendBody(Xtrans(Vector3d(0., 1., -1.)), j_z, b, "body3");

    RobotDynamics::Math::VectorNd q(model->q_size);
    RobotDynamics::Math::VectorNd qdot(model->qdot_size);

    Math::VectorNd N = Math::VectorNd::Zero(model->qdot_size);
    Math::VectorNd G = Math::VectorNd::Zero(model->qdot_size);
    Math::VectorNd C = Math::VectorNd::Zero(model->qdot_size);
    N.setZero();
    G.setZero();
    C.setZero();

    q.setZero();
    qdot.setZero();

    for (unsigned int i = 0; i < model->qdot_size; i++)
    {
        // At q=0, com is directly vertical and aligned with the joint
        q[i] = 0.3 * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.3;
        qdot[i] = 0.5 * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 1.0;
    }

    nonlinearEffects(*model, q, qdot, N, true);
    coriolisEffects(*model, q, qdot, C, true);
    gravityEffects(*model, G);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(C + G, N, unit_test_utils::E_MINUS_11));
}

TEST_F(Human36, coriolisEffects)
{
    randomizeStates();
    model_emulated->gravity.SpatialVector::set(0., 0., 0., 0., 0., -9.81);

    Math::VectorNd N = Math::VectorNd::Zero(model_emulated->qdot_size);
    Math::VectorNd G = Math::VectorNd::Zero(model_emulated->qdot_size);
    Math::VectorNd C = Math::VectorNd::Zero(model_emulated->qdot_size);
    N.setZero();
    G.setZero();
    C.setZero();

    nonlinearEffects(*model_emulated, q, qdot, N, true);
    coriolisEffects(*model_emulated, q, qdot, C, true);
    gravityEffects(*model_emulated, G);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(C + G, N, unit_test_utils::E_MINUS_11));
}

TEST_F(Human36, TestMInvTimesTauNoKinematics)
{
    randomizeStates();

    MatrixNd M(MatrixNd::Zero(model_3dof->dof_count, model_3dof->dof_count));
    updateKinematicsCustom(*model_3dof, &q, nullptr, nullptr);
    compositeRigidBodyAlgorithm(*model_3dof, q, M, false);

    VectorNd qddot_solve_llt = M.llt().solve(tau);

    VectorNd qddot_minv(q);
    calcMInvTimesTau(*model_3dof, q, tau, qddot_minv);

    for (unsigned int j = 0; j < 10; j++)
    {
        randomizeStates();

        updateKinematicsCustom(*model_3dof, &q, nullptr, nullptr);
        compositeRigidBodyAlgorithm(*model_3dof, q, M, false);
        qddot_solve_llt = M.llt().solve(tau);

        calcMInvTimesTau(*model_3dof, q, tau, qddot_minv);

        EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_solve_llt, qddot_minv, unit_test_utils::E_MINUS_13 * qddot_solve_llt.norm()));
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
