//
// RDL - Robot Dynamics Library
// Modifications Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
//
// Licensed under the zlib license. See LICENSE for more details.
//

#include <gtest/gtest.h>

#include "Human36Fixture.hpp"
#include <rdl_dynamics/Dynamics.hpp>
#include <rdl_dynamics/HydroDynamics.hpp>
#include <rdl_dynamics/Kinematics.hpp>
#include "UnitTestUtils.hpp"

struct HydroDynamicsTests : public testing::Test
{
    HydroDynamicsTests()
    {
        srand(time(NULL));
    }

    void SetUp()
    {
    }

    void TearDown()
    {
    }
};

struct HydroCustomEulerZYXJoint : public CustomJoint
{
    HydroCustomEulerZYXJoint()
    {
        mDoFCount = 3;
        S = MatrixNd::Zero(6, 3);
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

        model.X_J[joint_id] = SpatialTransform(
            Matrix3d(c0 * c1, s0 * c1, -s1, c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2, c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2),
            Vector3d(0., 0., 0.));

        S(0, 0) = -s1;
        S(0, 2) = 1.;

        S(1, 0) = c1 * s2;
        S(1, 1) = c2;

        S(2, 0) = c1 * c2;
        S(2, 1) = -s2;

        double qdot0 = qdot[model.mJoints[joint_id].q_index];
        double qdot1 = qdot[model.mJoints[joint_id].q_index + 1];
        double qdot2 = qdot[model.mJoints[joint_id].q_index + 2];

        model.v_J[joint_id].set(S * Vector3d(qdot0, qdot1, qdot2));

        model.c_J[joint_id].set(-c1 * qdot0 * qdot1, -s1 * s2 * qdot0 * qdot1 + c1 * c2 * qdot0 * qdot2 - s2 * qdot1 * qdot2,
                                -s1 * c2 * qdot0 * qdot1 - c1 * s2 * qdot0 * qdot2 - c2 * qdot1 * qdot2, 0., 0., 0.);
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
    }
};

TEST_F(HydroDynamicsTests, testBuoyancyOnSingleBody)
{
    RobotDynamics::Model model;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -9.81);

    double volume = 0.00037699111;

    double cob_x = 0.09;
    RobotDynamics::Body b(0.1, RobotDynamics::Math::Vector3d(0.1, 0., 0.), RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), RobotDynamics::Math::Vector3d(cob_x, 0., 0.),
                          volume, Math::SpatialMatrixZero, DragData());
    RobotDynamics::Joint j(RobotDynamics::JointTypeRevoluteY);

    model.addBody(0, RobotDynamics::Math::Xtrans(Math::Vector3d(0.1, -0.1, 0.)), j, b, "body1");

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_raw(model.qdot_size);

    q.setZero();
    qdot.setZero();
    tau.setZero();

    /**
     * @ q = -pi/2, the center of buoyancy is aligned with gravity, so while it will generate forces
     * due to buoyancy, when mapped into the joints space the force should not result in a torque,
     * meaning that dynamics w/o hydrodynamics should be the same as with hydrodynamics
     */
    q[0] = -M_PI_2;
    qdot.setZero();
    tau.setZero();

    RobotDynamics::forwardHydroDynamics(model, q, qdot, tau, qddot, nullptr);
    RobotDynamics::forwardDynamics(model, q, qdot, tau, qddot_raw, nullptr);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_raw, qddot, unit_test_utils::E_MINUS_14));
}

TEST_F(HydroDynamicsTests, testBuoyancyOnMultiBodyChain)
{
    /**
     * If buoyancy is correctly modeled and every body's center of mass and center of buoyancy are colocated AND
     * the mass of each body is equal to the weight of the water displaced by its volume, then the buoyancy forces
     * should exactly cancel out the effects of gravity. So in this test, a fairly random robot is built with
     * p_com = p_cob, then qdot=0, and tau=0, and the expected result from forward dynamics is that all
     * joint accelerations should be zero.
     */
    RobotDynamics::Model model;
    double a_g = 9.81;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -a_g);

    double fluidDensity = model.fluidDensity;

    RobotDynamics::Math::Vector3d b1_center(0.1, 0., 0.);
    double b1_mass = 0.1;
    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b1_center, b1_mass / fluidDensity, Math::SpatialMatrixZero, DragData());
    RobotDynamics::Joint j(RobotDynamics::JointTypeRevoluteY);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), j, b, "body1");

    RobotDynamics::Math::Vector3d b2_center(0.05, 0., 0.);
    double b2_mass = 0.3;
    b = RobotDynamics::Body(b2_mass, b2_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b2_center, b2_mass / fluidDensity, Math::SpatialMatrixZero, DragData());
    j = RobotDynamics::Joint(RobotDynamics::JointTypeRevoluteY);

    model.appendBody(Xtrans(Vector3d(0.1, 0., 1.)), j, b, "body2");

    RobotDynamics::Math::Vector3d b3_center(0.3, -0.04, 0.005);
    double b3_mass = 1.;
    b = RobotDynamics::Body(b3_mass, b3_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b3_center, b3_mass / fluidDensity, Math::SpatialMatrixZero, DragData());
    j = RobotDynamics::Joint(RobotDynamics::JointTypePrismatic, RobotDynamics::Math::Vector3d(1., 0., 0.));

    model.appendBody(Xtrans(Vector3d(0.1, -0.1, -0.2)), j, b, "body3");

    RobotDynamics::Math::Vector3d b4_center(1.3, -1.04, 0.105);
    double b4_mass = 1.5;
    b = RobotDynamics::Body(b4_mass, b4_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b4_center, b4_mass / fluidDensity, Math::SpatialMatrixZero, DragData());
    j = RobotDynamics::Joint(RobotDynamics::JointTypeRevoluteZ);

    model.appendBody(Xtrans(Vector3d(0.1, -0.1, -0.2)), j, b, "body4");

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);

    RobotDynamics::Math::VectorNd zero_v(model.qdot_size);
    zero_v.setZero();

    q.setZero();
    qdot.setZero();
    tau.setZero();

    q[0] = 0.1;
    q[1] = 0.35;
    q[2] = -0.11;

    RobotDynamics::forwardHydroDynamics(model, q, qdot, tau, qddot, nullptr);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot, zero_v, unit_test_utils::E_MINUS_14));
}

TEST_F(HydroDynamicsTests, testBuoyancyOnMultiBodyChainWith3DofJoints)
{
    /**
     * Similar to the above test, but with a few 3dof joints to make sure buoyancy works with an array of joint types
     */
    RobotDynamics::Model model;
    double a_g = 9.81;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -a_g);

    double fluidDensity = model.fluidDensity;

    RobotDynamics::Math::Vector3d b1_center(0.1, 0., 0.);
    double b1_mass = 0.1;
    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b1_center, b1_mass / fluidDensity, Math::SpatialMatrixZero, DragData());
    RobotDynamics::Joint j(RobotDynamics::JointTypeEulerXYZ);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), j, b, "body1");

    RobotDynamics::Math::Vector3d b2_center(0.05, 0., 0.);
    double b2_mass = 0.3;
    b = RobotDynamics::Body(b2_mass, b2_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b2_center, b2_mass / fluidDensity, Math::SpatialMatrixZero, DragData());
    j = RobotDynamics::Joint(RobotDynamics::JointTypeEulerZYX);

    model.appendBody(Xtrans(Vector3d(0.1, 0., 1.)), j, b, "body2");

    RobotDynamics::Math::Vector3d b3_center(0.3, -0.04, 0.005);
    double b3_mass = 1.;
    b = RobotDynamics::Body(b3_mass, b3_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b3_center, b3_mass / fluidDensity, Math::SpatialMatrixZero, DragData());
    j = RobotDynamics::Joint(RobotDynamics::JointTypePrismatic, RobotDynamics::Math::Vector3d(1., 0., 0.));

    model.appendBody(Xtrans(Vector3d(0.1, -0.1, -0.2)), j, b, "body3");

    RobotDynamics::Math::Vector3d b4_center(1.3, -1.04, 0.105);
    double b4_mass = 1.5;
    b = RobotDynamics::Body(b4_mass, b4_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b4_center, b4_mass / fluidDensity, Math::SpatialMatrixZero, DragData());
    j = RobotDynamics::Joint(RobotDynamics::JointTypeRevoluteZ);

    model.appendBody(Xtrans(Vector3d(0.1, -0.1, -0.2)), j, b, "body4");

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);

    RobotDynamics::Math::VectorNd zero_v(model.qdot_size);
    zero_v.setZero();

    q.setZero();
    qdot.setZero();
    tau.setZero();
    qddot.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        // At q=0, com is directly vertical and aligned with the joint
        q[i] = 0.4 * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.4 * M_PI_2;  // q from -pi/2 -> pi/2
    }

    RobotDynamics::forwardHydroDynamics(model, q, qdot, tau, qddot, nullptr);

    // Precision here is e-12, so had to multiply the precision by 100. My guess it's due to singularity problems with the euler joints. That's just a guess though
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot, zero_v, unit_test_utils::E_MINUS_12));
}

TEST_F(HydroDynamicsTests, testDynamicsWithCustomJoints)
{
    RobotDynamics::Model model, model_cust;
    double a_g = 9.81;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -a_g);

    SpatialVector drag_lin(0.15, 0.15, 0.15, 0.15, 0.15, 0.15);
    SpatialVector drag_quad = drag_lin;
    DragData dragData(drag_lin, drag_quad);
    double fluidDensity = model.fluidDensity;

    RobotDynamics::Math::Vector3d b1_center(0.1, 0., 0.);
    double b1_mass = 0.1;
    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b1_center, b1_mass / fluidDensity, SpatialMatrixZero, dragData);
    RobotDynamics::Joint j(RobotDynamics::JointTypeEulerZYX);
    HydroCustomEulerZYXJoint j_cust;

    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), j, b, "body1");
    model_cust.addBodyCustomJoint(0, Xtrans(Vector3d(0., 0., 0.)), &j_cust, b, "body1");

    RobotDynamics::Math::Vector3d b2_center(0.05, 0., 0.);
    double b2_mass = 0.3;
    b = RobotDynamics::Body(b2_mass, b2_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b2_center, b2_mass / fluidDensity, SpatialMatrixZero, dragData);
    j = RobotDynamics::Joint(RobotDynamics::JointTypeRevoluteY);

    model.appendBody(Xtrans(Vector3d(0.1, 0., 1.)), j, b, "body2");
    model_cust.appendBody(Xtrans(Vector3d(0.1, 0., 1.)), j, b, "body2");

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd tau_cust(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_cust(model.qdot_size);
    RobotDynamics::Math::VectorNd N(model.qdot_size);
    RobotDynamics::Math::VectorNd N_cust(model.qdot_size);
    RobotDynamics::Math::MatrixNd H(model.qdot_size, model.qdot_size);
    RobotDynamics::Math::MatrixNd H_cust(model.qdot_size, model.qdot_size);

    q.setZero();
    qdot.setZero();
    tau.setZero();
    tau_cust.setZero();
    qddot.setZero();
    N.setZero();
    N_cust.setZero();
    H.setZero();
    H_cust.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        // At q=0, com is directly vertical and aligned with the joint
        q[i] = 0.4 * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.4 * M_PI_2;
        qdot[i] = 0.4 * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.4 * M_PI_2;
        tau[i] = 0.4 * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.4 * M_PI_2;
    }

    RobotDynamics::forwardHydroDynamics(model, q, qdot, tau, qddot, nullptr);
    RobotDynamics::forwardHydroDynamics(model_cust, q, qdot, tau, qddot_cust, nullptr);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot, qddot_cust, unit_test_utils::E_MINUS_14));

    RobotDynamics::nonlinearEffectsHydro(model, q, qdot, N);
    RobotDynamics::nonlinearEffectsHydro(model_cust, q, qdot, N_cust);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(N, N_cust, unit_test_utils::E_MINUS_14));

    RobotDynamics::compositeRigidBodyAlgorithmHydro(model, q, H);
    RobotDynamics::compositeRigidBodyAlgorithmHydro(model_cust, q, H_cust);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(H, H_cust, unit_test_utils::E_MINUS_14));

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        // At q=0, com is directly vertical and aligned with the joint
        qddot[i] = 0.4 * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.4 * M_PI_2;
    }

    tau.setZero();

    RobotDynamics::inverseDynamicsHydro(model, q, qdot, qddot, tau);
    RobotDynamics::inverseDynamicsHydro(model_cust, q, qdot, qddot, tau_cust);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(tau, tau_cust, unit_test_utils::E_MINUS_14));
}

TEST_F(HydroDynamicsTests, testForwardDynamicsInertiaMatrixSingleBody)
{
    RobotDynamics::Model model;
    double a_g = 9.81;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -a_g);

    double fluidDensity = model.fluidDensity;
    SpatialMatrix I_added_mass = SpatialMatrixIdentity * 100.0;
    RobotDynamics::Math::Vector3d b1_center(0.0, 0., 0.1);
    double b1_mass = 0.1;
    // Bouyancy effects should be exactly counteracted by gravity
    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b1_center, b1_mass / fluidDensity, I_added_mass, DragData());
    RobotDynamics::Joint j(RobotDynamics::JointTypeRevoluteY);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), j, b, "body1");

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_no_hydro(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_crb(model.qdot_size);

    Math::MatrixNd H = Math::MatrixNd::Zero(model.qdot_size, model.qdot_size);
    H.setZero();

    q.setZero();
    qdot.setZero();
    qddot.setZero();
    tau.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        // At q=0, com is directly vertical and aligned with the joint
        q[i] = 2 * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI;  // q from 0 -> 2*pi
        tau[i] = 10. * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX));            // Tau from 0 -> 10.
    }

    forwardHydroDynamics(model, q, qdot, tau, qddot);
    model.gravity.setZero();  // Gravity effects ignored for the no hydro calcs to keep it from influencing the accelerations when q<0.
    forwardDynamics(model, q, qdot, tau, qddot_no_hydro);

    // If the sign on the added mass term f= I^A * a_f_g is correct, then with positive torque, zero velocity,
    // and buoyancy completely canceled out by gravity, the resulting accelerations from a positive torque without
    // added mass effect should always be greater than the resulting accelerations if added mass effects are included
    for (int i = 0; i < qddot.size(); i++)
    {
        EXPECT_TRUE(qddot_no_hydro[i] >= qddot[i]);
    }

    compositeRigidBodyAlgorithmHydro(model, q, H);

    qddot_crb = H.llt().solve(tau);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot, qddot_crb,
                                                           unit_test_utils::E_MINUS_13));  // Most of the time precise to e-15, but occasionally dips to e-14

    // Flip directions from above and make sure the opposite is true
    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        // At q=0, com is directly vertical and aligned with the joint
        q[i] = 2 * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI;  // q from -pi -> pi
        tau[i] = -10. * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX));           // Tau from -10 -> 0.
    }

    forwardHydroDynamics(model, q, qdot, tau, qddot);
    model.gravity.setZero();  // Gravity effects ignored for the no hydro calcs to keep it from influencing the accelerations when q<0.
    forwardDynamics(model, q, qdot, tau, qddot_no_hydro);

    // If the sign on the added mass term f= I^A * a_f_g is correct, then with positive torque, zero velocity,
    // and buoyancy completely canceled out by gravity, the resulting accelerations from a negative torque without
    // added mass effect should always be less than the resulting accelerations if added mass effects are included
    for (int i = 0; i < qddot.size(); i++)
    {
        EXPECT_TRUE(qddot[i] >= qddot_no_hydro[i]);
    }

    compositeRigidBodyAlgorithmHydro(model, q, H);

    qddot_crb = H.llt().solve(tau);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot, qddot_crb,
                                                           unit_test_utils::E_MINUS_13));  // Most of the time precise to e-15, but occasionally dips to e-14
}

TEST_F(HydroDynamicsTests, testForwardDynamicsAddedInertiaMatrixSinglePrismaticBody)
{
    RobotDynamics::Model model;
    double a_g = 9.81;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -a_g);

    double fluidDensity = model.fluidDensity;
    SpatialMatrix I_added_mass = SpatialMatrixIdentity * 100.0;

    RobotDynamics::Math::Vector3d b1_center(0.0, 0., 0.1);
    double b1_mass = 0.1;
    // Bouyancy effects should be exactly counteracted by gravity
    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b1_center, b1_mass / fluidDensity, I_added_mass, DragData());
    RobotDynamics::Joint j(RobotDynamics::JointTypePrismatic, RobotDynamics::Math::Vector3d(0., 0., 1.));

    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), j, b, "body1");

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_no_hydro(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_crb(model.qdot_size);

    Math::MatrixNd H = Math::MatrixNd::Zero(model.qdot_size, model.qdot_size);
    H.setZero();

    q.setZero();
    qdot.setZero();
    qddot.setZero();
    tau.setZero();

    model.a_f = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., 10.);  // Accelerate the fluid upward. This should cause a positive joint acceleration

    forwardHydroDynamics(model, q, qdot, tau, qddot);
    EXPECT_NEAR(qddot[0], 10., unit_test_utils::E_MINUS_14);

    model.a_f = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -10.);  // Accelerate the fluid upward. This should cause a positive joint acceleration

    forwardHydroDynamics(model, q, qdot, tau, qddot);
    EXPECT_NEAR(qddot[0], -10., unit_test_utils::E_MINUS_14);
}

TEST_F(HydroDynamicsTests, testForwardDynamicsInertiaMatrixMultiBodyChain)
{
    // Create a series of links in which each body's COM and joints are aligned with gravity, that way if
    // qdot=0, then all torques are from inertial effects if qddot != 0.

    RobotDynamics::Model model;
    double a_g = 9.81;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -a_g);

    double fluidDensity = model.fluidDensity;
    SpatialMatrix I_added_mass = SpatialMatrixIdentity * 20.0;

    RobotDynamics::Math::Vector3d b1_center(0.0, 0., 0.1);
    double b1_mass = 0.1;
    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b1_center, b1_mass / fluidDensity, I_added_mass, DragData());
    RobotDynamics::Joint j(RobotDynamics::JointTypeRevoluteY);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), j, b, "body1");

    RobotDynamics::Math::Vector3d b2_center(0.0, 0., 0.14);
    double b2_mass = 0.2;
    b = Body(b2_mass, b2_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b2_center, b2_mass / fluidDensity, I_added_mass, DragData());
    j = Joint(RobotDynamics::JointTypeRevoluteY);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.2)), j, b, "body2");

    RobotDynamics::Math::Vector3d b3_center(0.0, 0., 0.124);
    double b3_mass = 0.21;
    b = Body(b3_mass, b3_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b3_center, b3_mass / fluidDensity, I_added_mass, DragData());
    j = Joint(RobotDynamics::JointTypeRevoluteY);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.12)), j, b, "body3");

    RobotDynamics::Math::Vector3d b4_center(0.0, 0., 0.07);
    double b4_mass = 0.021;
    b = Body(b4_mass, b4_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b4_center, b4_mass / fluidDensity, I_added_mass, DragData());
    j = Joint(RobotDynamics::JointTypeRevoluteY);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.12)), j, b, "body4");

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_crb(model.qdot_size);

    Math::MatrixNd H = Math::MatrixNd::Zero(model.qdot_size, model.qdot_size);
    H.setZero();

    q.setZero();
    qdot.setZero();
    qddot.setZero();
    tau.setZero();

    tau[0] = 0.1;
    tau[1] = -1.;
    tau[2] = 0.24;
    tau[3] = 3.1;

    forwardHydroDynamics(model, q, qdot, tau, qddot);

    compositeRigidBodyAlgorithmHydro(model, q, H);

    qddot_crb = H.llt().solve(tau);

    // Precicion here had to be downgraded to e-13. Not really sure why.
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot, qddot_crb, unit_test_utils::E_MINUS_13));
}

TEST_F(HydroDynamicsTests, testForwardDynamicsInertiaMatrixMultiBodyChainNoKinematics)
{
    // Create a series of links in which each body's COM and joints are aligned with gravity, that way if
    // qdot=0, then all torques are from inertial effects if qddot != 0.

    RobotDynamics::Model model;
    double a_g = 9.81;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -a_g);

    double fluidDensity = model.fluidDensity;
    SpatialMatrix I_added_mass = SpatialMatrixIdentity * 20.0;

    RobotDynamics::Math::Vector3d b1_center(0.0, 0., 0.1);
    double b1_mass = 0.1;
    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b1_center, b1_mass / fluidDensity, I_added_mass, DragData());
    RobotDynamics::Joint j(RobotDynamics::JointTypeRevoluteY);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), j, b, "body1");

    RobotDynamics::Math::Vector3d b2_center(0.0, 0., 0.14);
    double b2_mass = 0.2;
    b = Body(b2_mass, b2_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b2_center, b2_mass / fluidDensity, I_added_mass, DragData());
    j = Joint(RobotDynamics::JointTypeRevoluteY);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.2)), j, b, "body2");

    RobotDynamics::Math::Vector3d b3_center(0.0, 0., 0.124);
    double b3_mass = 0.21;
    b = Body(b3_mass, b3_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b3_center, b3_mass / fluidDensity, I_added_mass, DragData());
    j = Joint(RobotDynamics::JointTypeRevoluteY);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.12)), j, b, "body3");

    RobotDynamics::Math::Vector3d b4_center(0.0, 0., 0.07);
    double b4_mass = 0.021;
    b = Body(b4_mass, b4_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b4_center, b4_mass / fluidDensity, I_added_mass, DragData());
    j = Joint(RobotDynamics::JointTypeRevoluteY);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.12)), j, b, "body4");

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_crb(model.qdot_size);

    Math::MatrixNd H = Math::MatrixNd::Zero(model.qdot_size, model.qdot_size);
    H.setZero();

    q.setZero();
    qdot.setZero();
    qddot.setZero();
    tau.setZero();

    tau[0] = 0.1;
    tau[1] = -1.;
    tau[2] = 0.24;
    tau[3] = 3.1;

    updateKinematics(model, q, qdot, qddot);

    forwardHydroDynamics(model, q, qdot, tau, qddot, nullptr, false);

    compositeRigidBodyAlgorithmHydro(model, q, H, false);

    qddot_crb = H.llt().solve(tau);

    // Precicion here had to be downgraded to e-13. Not really sure why.
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot, qddot_crb, unit_test_utils::E_MINUS_13));
}

TEST_F(HydroDynamicsTests, testAddedMassVelocityDependentEffects)
{
    RobotDynamics::Model model, model_only_hydro;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -9.81);

    double body_mass = 1.;
    Math::Vector3d p_com(0.1, 0., 0.);
    Math::Vector3d radius_of_gyration = Math::Vector3d(1., 1., 1.);
    SpatialMatrix I_added_mass = SpatialMatrixIdentity * 20.0;

    RobotDynamics::Body b(body_mass, p_com, radius_of_gyration, p_com, body_mass / model.fluidDensity, I_added_mass, DragData());
    RobotDynamics::Joint j(RobotDynamics::JointTypeRevoluteY);

    unsigned int id = model.addBody(0, RobotDynamics::Math::Xtrans(Math::Vector3d(0.1, 0., 0.)), j, b, "body1");

    RobotDynamics::Body b2(body_mass, p_com, radius_of_gyration, p_com, body_mass / model.fluidDensity, I_added_mass, DragData());
    RobotDynamics::Joint j2(RobotDynamics::JointTypeRevoluteY);

    id = model.addBody(id, RobotDynamics::Math::Xtrans(Math::Vector3d(0.1, 0., 0.)), j2, b2, "body2");

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_2(model.qdot_size);
    RobotDynamics::Math::VectorNd N(model.qdot_size);

    Math::MatrixNd H = Math::MatrixNd::Zero(model.qdot_size, model.qdot_size);
    H.setZero();

    N.setZero();
    q.setZero();
    qdot.setZero();
    qddot.setZero();
    qddot_2.setZero();
    tau.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        q[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI_2);
    }

    // For hydro effects, bouyancy is cancelling out gravity
    forwardHydroDynamics(model, q, qdot, tau, qddot);
    nonlinearEffectsHydro(model, q, qdot, N);
    compositeRigidBodyAlgorithmHydro(model, q, H);

    qddot_2 = H.llt().solve(tau - N);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot, qddot_2, unit_test_utils::E_MINUS_14));

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        q[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI_2);
        qdot[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI_2);
        tau[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI_2);
    }

    // For hydro effects, bouyancy is cancelling out gravity
    forwardHydroDynamics(model, q, qdot, tau, qddot);
    nonlinearEffectsHydro(model, q, qdot, N);
    compositeRigidBodyAlgorithmHydro(model, q, H);

    qddot_2 = H.llt().solve(tau - N);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot, qddot_2, unit_test_utils::E_MINUS_14));
}

TEST_F(Human36, compositeRigidBodyNoHydro)
{
    randomizeStates();

    MatrixNd M(MatrixNd::Zero(model_emulated->dof_count, model_emulated->dof_count)), M_hydro(MatrixNd::Zero(model_emulated->dof_count, model_emulated->dof_count));

    compositeRigidBodyAlgorithm(*model_emulated, q, M);
    compositeRigidBodyAlgorithmHydro(*model_emulated, q, M_hydro);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(M, M_hydro, 10. * unit_test_utils::E_MINUS_14));

    M = MatrixNd::Zero(model_3dof->dof_count, model_3dof->dof_count);
    M_hydro = MatrixNd::Zero(model_3dof->dof_count, model_3dof->dof_count);

    compositeRigidBodyAlgorithmHydro(*model_3dof, q, M_hydro);
    compositeRigidBodyAlgorithm(*model_3dof, q, M);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(M, M_hydro, 10. * unit_test_utils::E_MINUS_14));
}

TEST_F(HydroDynamicsTests, testDragOnRevoluteJointsReducesAcceleration)
{
    /**
     * In the absense of better ideas, I can at least test that a single rotational body with drag experiences
     * lower resulting accelerations than one without drag
     */
    Model model, model_nodrag;
    double a_g = 9.81;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -a_g);
    RobotDynamics::Math::Vector3d com(0.1, 0.1, -0.1);
    RobotDynamics::Math::Vector3d radii_of_gyration(1., 1., 1.);
    SpatialVector drag_ones(-1., -1., -1., -1., -1., -1.);
    DragData dragData(drag_ones, drag_ones);

    RobotDynamics::Body b(1., com, radii_of_gyration, com, 0.001, SpatialMatrixZero, dragData);
    RobotDynamics::Body b_nodrag(1., com, radii_of_gyration, com, 0.001, SpatialMatrixZero, DragData());
    RobotDynamics::Joint j_rev_y(RobotDynamics::JointTypeRevoluteY);

    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), j_rev_y, b);

    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), j_rev_y, b_nodrag);

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_nodrag(model.qdot_size);

    q.setZero();
    qdot.setZero();
    qddot.setZero();
    qddot_nodrag.setZero();
    tau.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        q[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
        qdot[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
    }

    forwardHydroDynamics(model, q, qdot, tau, qddot);
    forwardHydroDynamics(model_nodrag, q, qdot, tau, qddot_nodrag);

    for (unsigned int i = 0; i < qddot.size(); i++)
    {
        if (qdot[i] > 0.)
        {
            EXPECT_TRUE(qddot_nodrag[i] >= qddot[i]);
        }
        else if (qdot[i] < 0.)
        {
            EXPECT_TRUE(qddot_nodrag[i] <= qddot[i]);
        }
    }
}

TEST_F(HydroDynamicsTests, testDragOnLinearJointsReducesAcceleration)
{
    Model model, model_nodrag;
    double a_g = 0;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -a_g);
    model_nodrag.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -a_g);
    RobotDynamics::Math::Vector3d com(0., 0., 0.1);
    RobotDynamics::Math::Vector3d radii_of_gyration(1., 1., 1.);

    SpatialVector drag_ones(-1., -1., -1., -1., -1., -1.);
    DragData dragData(drag_ones, drag_ones);

    RobotDynamics::Body b(1., com, radii_of_gyration, com, 0.001, SpatialMatrixZero, dragData);
    RobotDynamics::Body b_nodrag(1., com, radii_of_gyration, com, 0.001, SpatialMatrixZero, DragData());
    RobotDynamics::Joint joint_pris_z(RobotDynamics::JointTypePrismatic, RobotDynamics::Math::Vector3d(0., 0., 1.));

    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0., 0., 0.))), joint_pris_z, b);

    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0., 0., 0.))), joint_pris_z, b_nodrag);

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd tau_nodrag(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_nodrag(model.qdot_size);

    q.setZero();
    qdot.setZero();
    qddot.setZero();
    qddot_nodrag.setZero();
    tau.setZero();
    tau_nodrag.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        q[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
        qdot[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
    }

    qdot[0] = 10.0;

    forwardHydroDynamics(model, q, qdot, tau, qddot);
    forwardHydroDynamics(model_nodrag, q, qdot, tau, qddot_nodrag);

    for (unsigned int i = 0; i < qddot.size(); i++)
    {
        if (qdot[i] > 0.)
        {
            EXPECT_TRUE(qddot_nodrag[i] >= qddot[i]);
        }
        else if (qdot[i] < 0.)
        {
            EXPECT_TRUE(qddot_nodrag[i] <= qddot[i]);
        }
    }

    q.setZero();
    qdot.setZero();
    qddot.setZero();
    qddot_nodrag.setZero();
    tau.setZero();
    tau_nodrag.setZero();

    qdot[0] = 10.0;

    inverseDynamicsHydro(model, q, qdot, qddot, tau);
    inverseDynamicsHydro(model_nodrag, q, qdot, qddot, tau_nodrag);

    EXPECT_TRUE(tau[0] > tau_nodrag[0]);
}

TEST_F(HydroDynamicsTests, testDragOnMultipleLinearJointsReducesAcceleration)
{
    Model model, model_nodrag;
    double a_g = 9.81;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -a_g);
    RobotDynamics::Math::Vector3d com(0.1, 0.1, -0.1);
    RobotDynamics::Math::Vector3d radii_of_gyration(1., 1., 1.);

    SpatialVector drag_ones(-1., -1., -1., -1., -1., -1.);
    DragData dragData(drag_ones, drag_ones);

    RobotDynamics::Body b(1., com, radii_of_gyration, com, 0.001, SpatialMatrixZero, dragData);
    RobotDynamics::Body b_nodrag(1., com, radii_of_gyration, com, 0.001, SpatialMatrixZero, DragData());
    RobotDynamics::Joint joint_pris_z(RobotDynamics::JointTypePrismatic, RobotDynamics::Math::Vector3d(0., 0., 1.));

    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_z, b);
    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_z, b);
    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_z, b);
    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_z, b);
    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_z, b);

    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_z, b_nodrag);
    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_z, b_nodrag);
    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_z, b_nodrag);
    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_z, b_nodrag);
    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_z, b_nodrag);

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_nodrag(model.qdot_size);

    q.setZero();
    qdot.setZero();
    qddot.setZero();
    qddot_nodrag.setZero();
    tau.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        q[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
        qdot[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
    }

    forwardHydroDynamics(model, q, qdot, tau, qddot);
    forwardHydroDynamics(model_nodrag, q, qdot, tau, qddot_nodrag);

    for (unsigned int i = 0; i < qddot.size(); i++)
    {
        if (qdot[i] > 0.)
        {
            EXPECT_TRUE(qddot_nodrag[i] >= qddot[i]);
        }
        else if (qdot[i] < 0.)
        {
            EXPECT_TRUE(qddot_nodrag[i] <= qddot[i]);
        }
    }
}

TEST_F(HydroDynamicsTests, testDragOnMultipleLinearJointsReducesAccelerationOnlyLinearDrag)
{
    Model model, model_nodrag;
    double a_g = 9.81;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -a_g);
    RobotDynamics::Math::Vector3d com(0.1, 0.1, -0.1);
    RobotDynamics::Math::Vector3d radii_of_gyration(1., 1., 1.);

    SpatialVector drag_ones(-1., -1., -1., -1., -1., -1.);
    DragData dragData(drag_ones, Math::SpatialVectorZero);

    RobotDynamics::Body b(1., com, radii_of_gyration, com, 0.001, SpatialMatrixZero, dragData);
    RobotDynamics::Body b_nodrag(1., com, radii_of_gyration, com, 0.001, SpatialMatrixZero, DragData());
    RobotDynamics::Joint joint_pris_y(RobotDynamics::JointTypePrismatic, RobotDynamics::Math::Vector3d(0., 0., 1.));

    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b);
    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b);
    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b);
    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b);
    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b);

    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b_nodrag);
    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b_nodrag);
    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b_nodrag);
    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b_nodrag);
    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b_nodrag);

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_nodrag(model.qdot_size);

    q.setZero();
    qdot.setZero();
    qddot.setZero();
    qddot_nodrag.setZero();
    tau.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        q[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
        qdot[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
    }

    forwardHydroDynamics(model, q, qdot, tau, qddot);
    forwardHydroDynamics(model_nodrag, q, qdot, tau, qddot_nodrag);

    for (unsigned int i = 0; i < qddot.size(); i++)
    {
        if (qdot[i] > 0.)
        {
            EXPECT_TRUE(qddot_nodrag[i] >= qddot[i]);
        }
        else if (qdot[i] < 0.)
        {
            EXPECT_TRUE(qddot_nodrag[i] <= qddot[i]);
        }
    }
}

TEST_F(HydroDynamicsTests, testDragOnMultipleLinearJointsReducesAccelerationOnlyQuadraticDrag)
{
    Model model, model_nodrag;
    double a_g = 9.81;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -a_g);
    RobotDynamics::Math::Vector3d com(0.1, 0.1, -0.1);
    RobotDynamics::Math::Vector3d radii_of_gyration(1., 1., 1.);

    SpatialVector drag_ones(-1., -1., -1., -1., -1., -1.);
    DragData dragData(Math::SpatialVectorZero, drag_ones);

    RobotDynamics::Body b(1., com, radii_of_gyration, com, 0.001, SpatialMatrixZero, dragData);
    RobotDynamics::Body b_nodrag(1., com, radii_of_gyration, com, 0.001, SpatialMatrixZero, DragData());
    RobotDynamics::Joint joint_pris_y(RobotDynamics::JointTypePrismatic, RobotDynamics::Math::Vector3d(0., 0., 1.));

    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b);
    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b);
    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b);
    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b);
    model.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b);

    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b_nodrag);
    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b_nodrag);
    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b_nodrag);
    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b_nodrag);
    model_nodrag.appendBody(RobotDynamics::Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.12, -0.1, 0.32))), joint_pris_y, b_nodrag);

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_nodrag(model.qdot_size);

    q.setZero();
    qdot.setZero();
    qddot.setZero();
    qddot_nodrag.setZero();
    tau.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        q[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
        qdot[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
    }

    forwardHydroDynamics(model, q, qdot, tau, qddot);
    forwardHydroDynamics(model_nodrag, q, qdot, tau, qddot_nodrag);

    for (unsigned int i = 0; i < qddot.size(); i++)
    {
        if (qdot[i] > 0.)
        {
            EXPECT_TRUE(qddot_nodrag[i] >= qddot[i]);
        }
        else if (qdot[i] < 0.)
        {
            EXPECT_TRUE(qddot_nodrag[i] <= qddot[i]);
        }
    }
}

TEST_F(HydroDynamicsTests, inverseDynamics)
{
    RobotDynamics::Model model;
    double a_g = 9.81;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -a_g);

    double fluidDensity = model.fluidDensity;
    SpatialMatrix I = SpatialMatrixIdentity * 10.0;

    SpatialVector drag_ones(-1., -1., -1., -1., -1., -1.);
    DragData dragData(drag_ones, drag_ones);

    RobotDynamics::Math::Vector3d b1_center(0.0, 0., 0.1);
    double b1_mass = 0.1;
    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b1_center, b1_mass / fluidDensity, I, dragData);
    RobotDynamics::Joint j(RobotDynamics::JointTypeEulerXYZ);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), j, b, "body1");

    RobotDynamics::Math::Vector3d b2_center(0.0, 0., 0.14);
    double b2_mass = 0.2;
    b = Body(b2_mass, b2_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b2_center, b2_mass / fluidDensity, I, dragData);
    j = Joint(RobotDynamics::JointTypeRevoluteY);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.2)), j, b, "body2");

    RobotDynamics::Math::Vector3d b3_center(0.0, 0., 0.124);
    double b3_mass = 0.21;
    b = Body(b3_mass, b3_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b3_center, b3_mass / fluidDensity, I, dragData);
    j = Joint(RobotDynamics::JointTypePrismatic, RobotDynamics::Math::Vector3d(0., 1., 0.));

    model.appendBody(Xtrans(Vector3d(0., 0., 0.12)), j, b, "body3");

    RobotDynamics::Math::Vector3d b4_center(0.0, 0., 0.07);
    double b4_mass = 0.021;
    b = Body(b4_mass, b4_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b4_center, b4_mass / fluidDensity, I, dragData);
    j = Joint(RobotDynamics::JointTypeRevoluteZ);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.12)), j, b, "body4");

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau_id(model.qdot_size);

    Math::MatrixNd H = Math::MatrixNd::Zero(model.qdot_size, model.qdot_size);
    H.setZero();

    q.setZero();
    qdot.setZero();
    qddot.setZero();
    tau_id.setZero();
    tau.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        q[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
        qdot[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
        tau[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
    }

    model.v_f = SpatialVector(0., 0., 0., 1., 2., 3.);
    forwardHydroDynamics(model, q, qdot, tau, qddot);
    inverseDynamicsHydro(model, q, qdot, qddot, tau_id);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(tau, tau_id, unit_test_utils::E_MINUS_10));
}

TEST_F(HydroDynamicsTests, inverseDynamicsNoKinematics)
{
    RobotDynamics::Model model;
    double a_g = 9.81;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -a_g);

    double fluidDensity = model.fluidDensity;
    SpatialMatrix I = SpatialMatrixIdentity * 10.0;

    SpatialVector drag_ones(-1., -1., -1., -1., -1., -1.);
    DragData dragData(drag_ones, drag_ones);

    RobotDynamics::Math::Vector3d b1_center(0.0, 0., 0.1);
    double b1_mass = 0.1;
    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b1_center, b1_mass / fluidDensity, I, dragData);
    RobotDynamics::Joint j(RobotDynamics::JointTypeEulerXYZ);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), j, b, "body1");

    RobotDynamics::Math::Vector3d b2_center(0.0, 0., 0.14);
    double b2_mass = 0.2;
    b = Body(b2_mass, b2_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b2_center, b2_mass / fluidDensity, I, dragData);
    j = Joint(RobotDynamics::JointTypeRevoluteY);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.2)), j, b, "body2");

    RobotDynamics::Math::Vector3d b3_center(0.0, 0., 0.124);
    double b3_mass = 0.21;
    b = Body(b3_mass, b3_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b3_center, b3_mass / fluidDensity, I, dragData);
    j = Joint(RobotDynamics::JointTypePrismatic, RobotDynamics::Math::Vector3d(0., 1., 0.));

    model.appendBody(Xtrans(Vector3d(0., 0., 0.12)), j, b, "body3");

    RobotDynamics::Math::Vector3d b4_center(0.0, 0., 0.07);
    double b4_mass = 0.021;
    b = Body(b4_mass, b4_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b4_center, b4_mass / fluidDensity, I, dragData);
    j = Joint(RobotDynamics::JointTypeRevoluteZ);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.12)), j, b, "body4");

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau_id(model.qdot_size);

    Math::MatrixNd H = Math::MatrixNd::Zero(model.qdot_size, model.qdot_size);
    H.setZero();

    q.setZero();
    qdot.setZero();
    qddot.setZero();
    tau_id.setZero();
    tau.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        q[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
        qdot[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
        tau[i] = M_PI * ((static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.5);
    }

    model.v_f = SpatialVector(0., 0., 0., 1., 2., 3.);

    updateKinematics(model, q, qdot, qddot);
    forwardHydroDynamics(model, q, qdot, tau, qddot, nullptr, false);
    inverseDynamicsHydro(model, q, qdot, qddot, tau_id, nullptr, false);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(tau, tau_id, unit_test_utils::E_MINUS_10));
}

TEST_F(HydroDynamicsTests, nonlinearEffects)
{
    RobotDynamics::Model model;
    model.gravity.setZero();

    RobotDynamics::Math::Vector3d b1_center(0.0, 0., 0.1);
    double b1_mass = 0.1;
    SpatialVector drag(-1., -1., -1., -1., -1., -1.);
    DragData dragData(drag, drag);
    // Bouyancy effects should be exactly counteracted by gravity
    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b1_center, 0., SpatialMatrixZero, dragData);
    RobotDynamics::Joint j_x(RobotDynamics::JointTypeRevoluteX);
    RobotDynamics::Joint j_y(RobotDynamics::JointTypeRevoluteY);
    RobotDynamics::Joint j_z(RobotDynamics::JointTypeRevoluteZ);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), j_x, b, "body1");
    model.appendBody(Xtrans(Vector3d(1., 0., 0.)), j_y, b, "body2");
    model.appendBody(Xtrans(Vector3d(0., 1., -1.)), j_z, b, "body3");

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_calc(model.qdot_size);

    Math::VectorNd N = Math::VectorNd::Zero(model.qdot_size);
    Math::MatrixNd H = Math::MatrixNd::Zero(model.qdot_size, model.qdot_size);
    N.setZero();
    H.setZero();

    q.setZero();
    qdot.setZero();
    qddot.setZero();
    qddot_calc.setZero();
    tau.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        // At q=0, com is directly vertical and aligned with the joint
        q[i] = 2 * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI;      // q from 0 -> 2*pi
        qdot[i] = 2. * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI;  // q from 0 -> 2*pi
        tau[i] = 10. * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX));                // Tau from 0 -> 10.
    }

    model.v_f = SpatialVector(0., 0., 0., 1., 2., 3.);

    forwardHydroDynamics(model, q, qdot, tau, qddot);
    compositeRigidBodyAlgorithmHydro(model, q, H);
    nonlinearEffectsHydro(model, q, qdot, N);

    compositeRigidBodyAlgorithmHydro(model, q, H);

    qddot_calc = H.llt().solve(-N + tau);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot, qddot_calc, unit_test_utils::E_MINUS_12));
}

TEST_F(HydroDynamicsTests, buoyancy)
{
    RobotDynamics::Model model;
    model.gravity.SpatialVector::set(0., 0., 0., 0., 0., -9.81);

    RobotDynamics::Math::Vector3d b1_center(0.0, 0., 0.1);
    double b1_mass = 0.1;
    SpatialVector drag(-1., -1., -1., -1., -1., -1.);
    DragData dragData(drag, drag);
    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b1_center, 0.123, SpatialMatrixZero, dragData);
    RobotDynamics::Joint j_x(RobotDynamics::JointTypeRevoluteX);
    RobotDynamics::Joint j_y(RobotDynamics::JointTypeRevoluteY);
    RobotDynamics::Joint j_z(RobotDynamics::JointTypeRevoluteZ);

    unsigned int fbid = model.appendBody(Xtrans(Vector3d(0., 0., 0.)), RobotDynamics::Joint(JointTypeFloatingBase), b, "floating_base");
    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), j_x, b, "body1");
    model.appendBody(Xtrans(Vector3d(1., 0., 0.)), j_y, b, "body2");
    model.appendBody(Xtrans(Vector3d(0., 1., -1.)), j_z, b, "body3");

    RobotDynamics::Math::VectorNd q(model.q_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);

    Math::VectorNd N = Math::VectorNd::Zero(model.qdot_size);
    Math::VectorNd B = Math::VectorNd::Zero(model.qdot_size);
    Math::VectorNd G = Math::VectorNd::Zero(model.qdot_size);
    N.setZero();
    B.setZero();
    G.setZero();

    q.setZero();
    qdot.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        // At q=0, com is directly vertical and aligned with the joint
        q[i] = 2 * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI;  // q from 0 -> 2*pi
        qdot[i] = 0.0;
    }

    RobotDynamics::Math::Quaternion o = model.GetQuaternion(fbid, q);
    o.normalize();
    model.SetQuaternion(fbid, o, q);

    nonlinearEffectsHydro(model, q, qdot, N);
    buoyancyEffects(model, B);
    updateKinematicsCustom(model, &q, &qdot, nullptr);
    gravityEffects(model, G);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(N, (G + B), unit_test_utils::E_MINUS_11));
}

TEST_F(HydroDynamicsTests, dragEffects)
{
    RobotDynamics::Model model;
    model.gravity.SpatialVector::set(0., 0., 0., 0., 0., -9.81);

    RobotDynamics::Math::Vector3d b1_center(0.0, 0., 0.1);
    double b1_mass = 0.1;
    SpatialVector drag(-1., -1., -1., -1., -1., -1.);
    DragData dragData(drag, drag);
    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b1_center, 0., SpatialMatrixZero, dragData);
    RobotDynamics::Joint j_x(RobotDynamics::JointTypeRevoluteX);
    RobotDynamics::Joint j_y(RobotDynamics::JointTypeRevoluteY);
    RobotDynamics::Joint j_z(RobotDynamics::JointTypeRevoluteZ);

    unsigned int fbid = model.appendBody(Xtrans(Vector3d(0., 0., 0.)), RobotDynamics::Joint(JointTypeFloatingBase), b, "floating_base");
    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), j_x, b, "body1");
    model.appendBody(Xtrans(Vector3d(1., 0., 0.)), j_y, b, "body2");
    model.appendBody(Xtrans(Vector3d(0., 1., -1.)), j_z, b, "body3");

    RobotDynamics::Math::VectorNd q(model.q_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_calc(model.qdot_size);

    Math::VectorNd N = Math::VectorNd::Zero(model.qdot_size);
    Math::VectorNd D = Math::VectorNd::Zero(model.qdot_size);
    Math::VectorNd B = Math::VectorNd::Zero(model.qdot_size);
    Math::VectorNd N_hydro = Math::VectorNd::Zero(model.qdot_size);
    N.setZero();
    N_hydro.setZero();
    D.setZero();
    B.setZero();

    q.setZero();
    qdot.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        // At q=0, com is directly vertical and aligned with the joint
        q[i] = 2 * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI;  // q from 0 -> 2*pi
        qdot[i] = 0.5 * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 1.0;     // q from 0 -> 2*pi
    }

    RobotDynamics::Math::Quaternion o = model.GetQuaternion(fbid, q);
    o.normalize();
    model.SetQuaternion(fbid, o, q);

    model.v_f = SpatialVector(0., 0., 0., 0.1, 0.2, 0.3);

    nonlinearEffects(model, q, qdot, N, true);
    dragEffects(model, q, qdot, D, true);
    nonlinearEffectsHydro(model, q, qdot, N_hydro, true);
    buoyancyEffects(model, B);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(D, (N_hydro - N - B), unit_test_utils::E_MINUS_9));
}

TEST_F(HydroDynamicsTests, nonlinearEffectsNoKinematics)
{
    RobotDynamics::Model model;
    model.gravity.setZero();

    RobotDynamics::Math::Vector3d b1_center(0.0, 0., 0.1);
    double b1_mass = 0.1;
    SpatialVector drag(-1., -1., -1., -1., -1., -1.);
    DragData dragData(drag, drag);
    // Bouyancy effects should be exactly counteracted by gravity
    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b1_center, 0., SpatialMatrixZero, dragData);
    RobotDynamics::Joint j_x(RobotDynamics::JointTypeRevoluteX);
    RobotDynamics::Joint j_y(RobotDynamics::JointTypeRevoluteY);
    RobotDynamics::Joint j_z(RobotDynamics::JointTypeRevoluteZ);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), j_x, b, "body1");
    model.appendBody(Xtrans(Vector3d(1., 0., 0.)), j_y, b, "body2");
    model.appendBody(Xtrans(Vector3d(0., 1., -1.)), j_z, b, "body3");

    RobotDynamics::Math::VectorNd q(model.qdot_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_calc(model.qdot_size);

    Math::VectorNd N = Math::VectorNd::Zero(model.qdot_size);
    Math::MatrixNd H = Math::MatrixNd::Zero(model.qdot_size, model.qdot_size);
    N.setZero();
    H.setZero();

    q.setZero();
    qdot.setZero();
    qddot.setZero();
    qddot_calc.setZero();
    tau.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        // At q=0, com is directly vertical and aligned with the joint
        q[i] = 2 * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI;      // q from 0 -> 2*pi
        qdot[i] = 2. * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI;  // q from 0 -> 2*pi
        tau[i] = 10. * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX));                // Tau from 0 -> 10.
    }

    model.v_f = SpatialVector(0., 0., 0., 1., 2., 3.);
    updateKinematicsCustom(model, &q, &qdot, &qddot);

    forwardHydroDynamics(model, q, qdot, tau, qddot, nullptr, false);
    compositeRigidBodyAlgorithmHydro(model, q, H, false);
    nonlinearEffectsHydro(model, q, qdot, N, false);

    compositeRigidBodyAlgorithmHydro(model, q, H, false);

    qddot_calc = H.llt().solve(-N + tau);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot, qddot_calc, unit_test_utils::E_MINUS_12));
}

TEST_F(HydroDynamicsTests, coriolisEffectsHydro)
{
    RobotDynamics::Model model;
    model.gravity.SpatialVector::set(0., 0., 0., 0., 0., -9.81);

    RobotDynamics::Math::Vector3d b1_center(0.0, 0., 0.1);
    double b1_mass = 0.1;
    SpatialVector drag(-1., -1., -1., -1., -1., -1.);
    DragData dragData(drag, drag);
    RobotDynamics::Math::SpatialMatrix Iadd;
    Iadd(0, 0) = 100.;
    Iadd(1, 1) = 200.;
    Iadd(2, 2) = 300.;
    Iadd(3, 3) = 400.;
    Iadd(4, 4) = 500.;
    Iadd(5, 5) = 600.;

    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), RobotDynamics::Math::Vector3d(0.02, -0.003, -0.01), 0.2, Iadd, dragData);
    RobotDynamics::Joint j_x(RobotDynamics::JointTypeRevoluteX);
    RobotDynamics::Joint j_y(RobotDynamics::JointTypeRevoluteY);
    RobotDynamics::Joint j_z(RobotDynamics::JointTypeRevoluteZ);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), RobotDynamics::Joint(JointTypeFloatingBase), b, "floating_base");
    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), j_x, b, "body1");
    model.appendBody(Xtrans(Vector3d(0.1, 0.2, 0.3)), RobotDynamics::Joint(RobotDynamics::JointTypeEulerZYX), b, "euler_body");
    model.appendBody(Xtrans(Vector3d(1., 0., 0.)), j_y, b, "body2");
    model.appendBody(Xtrans(Vector3d(0., 1., -1.)), j_z, b, "body3");

    RobotDynamics::Math::VectorNd q(model.q_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);
    RobotDynamics::Math::VectorNd tau(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot(model.qdot_size);
    RobotDynamics::Math::VectorNd qddot_calc(model.qdot_size);

    Math::VectorNd N = Math::VectorNd::Zero(model.qdot_size);
    Math::VectorNd D = Math::VectorNd::Zero(model.qdot_size);
    Math::VectorNd B = Math::VectorNd::Zero(model.qdot_size);
    Math::VectorNd G = Math::VectorNd::Zero(model.qdot_size);
    Math::VectorNd C = Math::VectorNd::Zero(model.qdot_size);
    N.setZero();
    D.setZero();
    B.setZero();
    G.setZero();
    C.setZero();

    q.setZero();
    qdot.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        // At q=0, com is directly vertical and aligned with the joint
        q[i] = 0.2 * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.2;
        qdot[i] = 0.5 * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 1.0;
    }

    model.v_f = SpatialVector(0., 0., 0., 0.1, 0.2, 0.3);

    nonlinearEffectsHydro(model, q, qdot, N, true);
    dragEffects(model, q, qdot, D, true);
    coriolisEffectsHydro(model, q, qdot, C, true);
    buoyancyEffects(model, B);
    gravityEffects(model, G);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(D + B + C + G, N, unit_test_utils::E_MINUS_5));
}

TEST_F(HydroDynamicsTests, potentialEffects)
{
    RobotDynamics::Model model;
    model.a_f.SpatialVector::set(0., 0., 0., 0.1, -0.1, 0.15);
    model.v_f.SpatialVector::set(0., 0., 0., 0.2, -0.2, 0.25);
    model.gravity.SpatialVector::set(0., 0., 0., 0., 0., -9.81);

    RobotDynamics::Math::Vector3d b1_center(0.0, 0., 0.1);
    double b1_mass = 0.1;
    SpatialVector drag(0., 0., 0., 0., 0., 0);
    DragData dragData(drag, drag);
    RobotDynamics::Body b(b1_mass, b1_center, RobotDynamics::Math::Vector3d(0.1, 0.1, 0.1), b1_center, 0., SpatialMatrixZero, dragData);
    RobotDynamics::Joint j_x(RobotDynamics::JointTypeRevoluteX);
    RobotDynamics::Joint j_y(RobotDynamics::JointTypeRevoluteY);
    RobotDynamics::Joint j_z(RobotDynamics::JointTypeRevoluteZ);

    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), RobotDynamics::Joint(JointTypeFloatingBase), b, "floating_base");
    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), j_x, b, "body1");
    model.appendBody(Xtrans(Vector3d(1., 0., 0.)), j_y, b, "body2");
    model.appendBody(Xtrans(Vector3d(0.1, 0.2, 0.3)), RobotDynamics::Joint(RobotDynamics::JointTypeEulerZYX), b, "euler_body");
    model.appendBody(Xtrans(Vector3d(0., 1., -1.)), j_z, b, "body3");

    RobotDynamics::Math::VectorNd q(model.q_size);
    RobotDynamics::Math::VectorNd qdot(model.qdot_size);

    Math::VectorNd G = Math::VectorNd::Zero(model.qdot_size);
    Math::VectorNd B = Math::VectorNd::Zero(model.qdot_size);
    Math::VectorNd P = Math::VectorNd::Zero(model.qdot_size);
    G.setZero();
    B.setZero();
    P.setZero();

    q.setZero();
    qdot.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        // At q=0, com is directly vertical and aligned with the joint
        q[i] = 0.3 * M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 0.3;
        qdot[i] = 0.5 * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - 1.0;
    }

    potentialEffects(model, P);
    gravityEffects(model, G);
    buoyancyEffects(model, B);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(B + G, P, unit_test_utils::E_MINUS_11));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
