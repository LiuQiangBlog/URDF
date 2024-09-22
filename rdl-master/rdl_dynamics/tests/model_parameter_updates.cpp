// Copyright (c) 2023 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>

#include "UnitTestUtils.hpp"
#include <rdl_dynamics/Dynamics.hpp>
#include <rdl_dynamics/HydroDynamics.hpp>
#include <rdl_dynamics/Model.hpp>
#include <rdl_dynamics/rdl_hydro_utils.hpp>
#include <rdl_dynamics/rdl_utils.hpp>

using namespace RobotDynamics;
using namespace RobotDynamics::Math;

TEST(model_parameter_updates, fixed_base)
{
    rclcpp::Node::SharedPtr n = rclcpp::Node::make_shared("simple_fixed_base");
    Model::SharedPtr m1(new Model(n->create_sub_node("m1"))), m2(new Model(n->create_sub_node("m2")));
    Body b1(101., Vector3d(-0.1, 0.2, 0.3), Matrix3d(1299.099143, -16.243531, -19.235748, -16.243531, 23463.178046, 10.491054, -19.235748, 10.491054, 24147.394567));
    Body b2(37.3, Vector3d(0.1, -0.9, 0.2), Vector3d(12.1, 9.7, 3.3));
    Body b3(12.2, Vector3d(0.9, -1.2, 0.73), Vector3d(1.1, 2.7, 2.1));
    Body b4(19.2, Vector3d(-1.9, 2.2, 0.543), Vector3d(2.1, 3.7, 4.1));

    // build the first model
    m1->appendBody(Xrotz(-0.2) * Xroty(0.3) * Xrotx(0.7) * Xtrans(Vector3d(1.8, -2.2, 0.7)), Joint(JointTypePrismatic, Vector3d(0., 1., 0.)), b1, "a");
    unsigned int b_id = m1->appendBody(Xrotz(0.1) * Xroty(-0.6) * Xrotx(1.7) * Xtrans(Vector3d(0.8, 1.2, 1.7)), Joint(JointTypeRevoluteX), b2, "b");
    m1->appendBody(Xrotz(-2.2) * Xroty(1.3) * Xrotx(2.7) * Xtrans(Vector3d(2.8, 12.2, 2.7)), Joint(JointTypePrismatic, Vector3d(0., 0., 1.)), b3, "c");
    m1->addBody(b_id, Xrotz(-2.2) * Xroty(1.3) * Xrotx(2.7) * Xtrans(Vector3d(2.8, 12.2, 2.7)), Joint(JointTypeFixed), b4, "d");

    // build the second model, the last link is the only thing different from the first one
    m2->appendBody(Xrotz(-0.2) * Xroty(0.3) * Xrotx(0.7) * Xtrans(Vector3d(1.8, -2.2, 0.7)), Joint(JointTypePrismatic, Vector3d(0., 1., 0.)), b1, "a");
    b_id = m2->appendBody(Xrotz(0.1) * Xroty(-0.6) * Xrotx(1.7) * Xtrans(Vector3d(0.8, 1.2, 1.7)), Joint(JointTypeRevoluteX), b2, "b");
    m2->appendBody(Xrotz(-2.2) * Xroty(1.3) * Xrotx(2.7) * Xtrans(Vector3d(2.8, 12.2, 2.7)), Joint(JointTypePrismatic, Vector3d(0., 0., 1.)), b4, "c");
    m2->addBody(b_id, Xrotz(-2.2) * Xroty(1.3) * Xrotx(2.7) * Xtrans(Vector3d(2.8, 12.2, 2.7)), Joint(JointTypeFixed), b4, "d");

    m1->setupRosParameters();
    m2->setupRosParameters();

    rclcpp::AsyncParametersClient::SharedPtr client = std::make_shared<rclcpp::AsyncParametersClient>(n, n->get_name());

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Zero(m1->q_size);
    VectorNd QDot = VectorNd::Zero(m1->dof_count);
    VectorNd Tau = VectorNd::Zero(m1->dof_count);
    VectorNd QDDot_m1 = VectorNd::Zero(m1->dof_count);
    VectorNd QDDot_m2 = VectorNd::Zero(m1->dof_count);
    MatrixNd H_m1 = MatrixNd::Zero(m1->dof_count, m1->dof_count);
    MatrixNd H_m2 = MatrixNd::Zero(m1->dof_count, m1->dof_count);
    VectorNd G_m1 = VectorNd::Zero(m1->dof_count);
    VectorNd G_m2 = VectorNd::Zero(m1->dof_count);
    VectorNd N_m1 = VectorNd::Zero(m1->dof_count);
    VectorNd N_m2 = VectorNd::Zero(m1->dof_count);
    Vector3d com_m1, com_m2;

    Q << -0.9, 0.3, 1.3;
    QDot << -1.7, -2.1, 0.89;
    Tau << 25.2, -76.22, 33.1;

    forwardDynamics(*m1, Q, QDot, Tau, QDDot_m1);
    forwardDynamics(*m2, Q, QDot, Tau, QDDot_m2);
    compositeRigidBodyAlgorithm(*m1, Q, H_m1, false);
    compositeRigidBodyAlgorithm(*m2, Q, H_m2, false);
    gravityEffects(*m1, G_m1);
    gravityEffects(*m2, G_m2);
    nonlinearEffects(*m1, Q, QDot, N_m1, false);
    nonlinearEffects(*m2, Q, QDot, N_m2, false);
    Utils::calcCenterOfMass(*m1, Q, com_m1, false);
    Utils::calcCenterOfMass(*m2, Q, com_m2, false);

    EXPECT_FALSE(H_m1.isApprox(H_m2, unit_test_utils::E_MINUS_9));
    EXPECT_FALSE(QDDot_m1.isApprox(QDDot_m2, unit_test_utils::E_MINUS_9));
    EXPECT_FALSE(G_m1.isApprox(G_m2, unit_test_utils::E_MINUS_9));
    EXPECT_FALSE(N_m1.isApprox(N_m2, unit_test_utils::E_MINUS_9));
    EXPECT_FALSE(com_m1.isApprox(com_m2, unit_test_utils::E_MINUS_9));

    // set the mass params of link3 on model2 to be the same as on model1 then compare dynamics again.
    rclcpp::Parameter m("m2/c_mass", b3.mMass), comx("m2/c_comx", b3.mCenterOfMass.x()), comy("m2/c_comy", b3.mCenterOfMass.y()), comz("m2/c_comz", b3.mCenterOfMass.z()),
        ixx("m2/c_ixx", b3.mInertia(0, 0)), ixy("m2/c_ixy", b3.mInertia(0, 1)), ixz("m2/c_ixz", b3.mInertia(0, 2)), iyy("m2/c_iyy", b3.mInertia(1, 1)),
        iyz("m2/c_iyz", b3.mInertia(1, 2)), izz("m2/c_izz", b3.mInertia(2, 2));

    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future =
        client->set_parameters(std::vector<rclcpp::Parameter>({ m, comx, comy, comz, ixx, ixy, ixz, iyy, iyz, izz }));

    EXPECT_EQ(rclcpp::spin_until_future_complete(n, future), rclcpp::FutureReturnCode::SUCCESS);

    forwardDynamics(*m1, Q, QDot, Tau, QDDot_m1);
    forwardDynamics(*m2, Q, QDot, Tau, QDDot_m2);
    compositeRigidBodyAlgorithm(*m1, Q, H_m1, false);
    compositeRigidBodyAlgorithm(*m2, Q, H_m2, false);
    gravityEffects(*m1, G_m1);
    gravityEffects(*m2, G_m2);
    nonlinearEffects(*m1, Q, QDot, N_m1, false);
    nonlinearEffects(*m2, Q, QDot, N_m2, false);
    Utils::calcCenterOfMass(*m1, Q, com_m1, false);
    Utils::calcCenterOfMass(*m2, Q, com_m2, false);

    EXPECT_TRUE(QDDot_m1.isApprox(QDDot_m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(QDDot_m1.isApprox(QDDot_m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(G_m1.isApprox(G_m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(N_m1.isApprox(N_m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(com_m1.isApprox(com_m2, unit_test_utils::E_MINUS_14));
}

TEST(model_parameter_updates, floating_base)
{
    rclcpp::Node::SharedPtr n = rclcpp::Node::make_shared("floating_base");
    Model::SharedPtr m1(new Model(n->create_sub_node("m1"))), m2(new Model(n->create_sub_node("m2")));
    Body b1(101., Vector3d(-0.1, 0.2, 0.3), Matrix3d(1299.099143, -16.243531, -19.235748, -16.243531, 23463.178046, 10.491054, -19.235748, 10.491054, 24147.394567));
    Body b2(37.3, Vector3d(0.1, -0.9, 0.2), Vector3d(12.1, 9.7, 3.3));
    Body b3(12.2, Vector3d(0.9, -1.2, 0.73), Vector3d(1.1, 2.7, 2.1));
    Body b4(b1.mMass / 3., Vector3d(b1.mCenterOfMass / 3.), Matrix3d(b1.mInertia / 3.));

    // build the first model
    unsigned int root_body_id = m1->appendBody(Xrotz(-0.2) * Xroty(0.3) * Xrotx(0.7) * Xtrans(Vector3d(1.8, -2.2, 0.7)), Joint(JointTypeFloatingBase), b1, "a");
    m1->appendBody(Xrotz(0.1) * Xroty(-0.6) * Xrotx(1.7) * Xtrans(Vector3d(0.8, 1.2, 1.7)), Joint(JointTypeRevoluteZ), b2, "b");
    m1->appendBody(Xrotz(-2.2) * Xroty(1.3) * Xrotx(2.7) * Xtrans(Vector3d(2.8, 12.2, 2.7)), Joint(JointTypePrismatic, Vector3d(0., 0., 1.)), b3, "c");
    m1->addBody(root_body_id, Xrotz(-0.1) * Xroty(0.9) * Xrotx(-0.7) * Xtrans(Vector3d(-0.8, -1.2, -1.7)), Joint(JointTypeRevoluteY), b2, "d");
    m1->appendBody(Xrotz(-1.9) * Xroty(-2.3) * Xrotx(-1.17) * Xtrans(Vector3d(-2.8, -12.4, 1.7)), Joint(JointTypePrismatic, Vector3d(1., 0., 0.)), b3, "e");

    // build the second model, the first floating base link is the only thing different from the first one
    root_body_id = m2->appendBody(Xrotz(-0.2) * Xroty(0.3) * Xrotx(0.7) * Xtrans(Vector3d(1.8, -2.2, 0.7)), Joint(JointTypeFloatingBase), b4, "a");
    m2->appendBody(Xrotz(0.1) * Xroty(-0.6) * Xrotx(1.7) * Xtrans(Vector3d(0.8, 1.2, 1.7)), Joint(JointTypeRevoluteZ), b2, "b");
    m2->appendBody(Xrotz(-2.2) * Xroty(1.3) * Xrotx(2.7) * Xtrans(Vector3d(2.8, 12.2, 2.7)), Joint(JointTypePrismatic, Vector3d(0., 0., 1.)), b3, "c");
    m2->addBody(root_body_id, Xrotz(-0.1) * Xroty(0.9) * Xrotx(-0.7) * Xtrans(Vector3d(-0.8, -1.2, -1.7)), Joint(JointTypeRevoluteY), b2, "d");
    m2->appendBody(Xrotz(-1.9) * Xroty(-2.3) * Xrotx(-1.17) * Xtrans(Vector3d(-2.8, -12.4, 1.7)), Joint(JointTypePrismatic, Vector3d(1., 0., 0.)), b3, "e");

    m1->setupRosParameters();
    m2->setupRosParameters();

    rclcpp::AsyncParametersClient::SharedPtr client = std::make_shared<rclcpp::AsyncParametersClient>(n, n->get_name());

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Zero(m1->q_size);
    VectorNd QDot = VectorNd::Zero(m1->dof_count);
    VectorNd Tau = VectorNd::Zero(m1->dof_count);
    VectorNd QDDot_m1 = VectorNd::Zero(m1->dof_count);
    VectorNd QDDot_m2 = VectorNd::Zero(m1->dof_count);
    MatrixNd H_m1 = MatrixNd::Zero(m1->dof_count, m1->dof_count);
    MatrixNd H_m2 = MatrixNd::Zero(m1->dof_count, m1->dof_count);
    VectorNd G_m1 = VectorNd::Zero(m1->dof_count);
    VectorNd G_m2 = VectorNd::Zero(m1->dof_count);
    VectorNd N_m1 = VectorNd::Zero(m1->dof_count);
    VectorNd N_m2 = VectorNd::Zero(m1->dof_count);
    Vector3d com_m1, com_m2;

    Quaternion o(0.2, 0.3, 0.4, 0.5);
    o.normalize();
    Q << 1.1, 1.3, 2.4, o.x(), o.y(), o.z(), 3.2, -0.9, 0.3, 1.3, o.w();
    QDot << -1.7, -2.1, 0.89, 9.2, 1.3, -0.9, 1.1, 3.3, 4.1, 12.1;
    Tau << 25.2, -76.22, 33.1, -9.9, -0.255, 1.2, 100.11, 0.9981, 1.33, 57.2;

    forwardDynamics(*m1, Q, QDot, Tau, QDDot_m1);
    forwardDynamics(*m2, Q, QDot, Tau, QDDot_m2);
    compositeRigidBodyAlgorithm(*m1, Q, H_m1, false);
    compositeRigidBodyAlgorithm(*m2, Q, H_m2, false);
    gravityEffects(*m1, G_m1);
    gravityEffects(*m2, G_m2);
    nonlinearEffects(*m1, Q, QDot, N_m1, false);
    nonlinearEffects(*m2, Q, QDot, N_m2, false);
    Utils::calcCenterOfMass(*m1, Q, com_m1, false);
    Utils::calcCenterOfMass(*m2, Q, com_m2, false);

    EXPECT_FALSE(H_m1.isApprox(H_m2, unit_test_utils::E_MINUS_9));
    EXPECT_FALSE(QDDot_m1.isApprox(QDDot_m2, unit_test_utils::E_MINUS_9));
    EXPECT_FALSE(G_m1.isApprox(G_m2, unit_test_utils::E_MINUS_9));
    EXPECT_FALSE(N_m1.isApprox(N_m2, unit_test_utils::E_MINUS_9));
    EXPECT_FALSE(com_m1.isApprox(com_m2, unit_test_utils::E_MINUS_9));

    // set the mass params of the floating base link on each model to be the same now
    rclcpp::Parameter m("m2/a_mass", b1.mMass), comx("m2/a_comx", b1.mCenterOfMass.x()), comy("m2/a_comy", b1.mCenterOfMass.y()), comz("m2/a_comz", b1.mCenterOfMass.z()),
        ixx("m2/a_ixx", b1.mInertia(0, 0)), ixy("m2/a_ixy", b1.mInertia(0, 1)), ixz("m2/a_ixz", b1.mInertia(0, 2)), iyy("m2/a_iyy", b1.mInertia(1, 1)),
        iyz("m2/a_iyz", b1.mInertia(1, 2)), izz("m2/a_izz", b1.mInertia(2, 2));

    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future =
        client->set_parameters(std::vector<rclcpp::Parameter>({ m, comx, comy, comz, ixx, ixy, ixz, iyy, iyz, izz }));

    EXPECT_EQ(rclcpp::spin_until_future_complete(n, future), rclcpp::FutureReturnCode::SUCCESS);

    forwardDynamics(*m1, Q, QDot, Tau, QDDot_m1);
    forwardDynamics(*m2, Q, QDot, Tau, QDDot_m2);
    compositeRigidBodyAlgorithm(*m1, Q, H_m1, false);
    compositeRigidBodyAlgorithm(*m2, Q, H_m2, false);
    gravityEffects(*m1, G_m1);
    gravityEffects(*m2, G_m2);
    nonlinearEffects(*m1, Q, QDot, N_m1, false);
    nonlinearEffects(*m2, Q, QDot, N_m2, false);
    Utils::calcCenterOfMass(*m1, Q, com_m1, false);
    Utils::calcCenterOfMass(*m2, Q, com_m2, false);

    EXPECT_TRUE(QDDot_m1.isApprox(QDDot_m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(H_m1.isApprox(H_m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(G_m1.isApprox(G_m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(N_m1.isApprox(N_m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(com_m1.isApprox(com_m2, unit_test_utils::E_MINUS_14));
}

TEST(model_parameter_updates, floating_base_hydro)
{
    rclcpp::Node::SharedPtr n = rclcpp::Node::make_shared("floating_base_hydro");

    Model::SharedPtr m1(new Model(n->create_sub_node("m1"))), m2(new Model(n->create_sub_node("m2")));
    Body b1(101., Vector3d(-0.1, 0.2, 0.3), Matrix3d(1299.099143, -16.243531, -19.235748, -16.243531, 23463.178046, 10.491054, -19.235748, 10.491054, 24147.394567),
            Vector3d(1.1, -0.98, 0.7), 351.3, SpatialMatrixZero,
            DragData(SpatialVector(-18.1, -21.2, -16.3, -20.4, -34.5, -212.3), SpatialVector(-222.1, -37.2, -41.3, -56.4, -68.5, -114.3)));
    Body b2(37.3, Vector3d(0.1, -0.9, 0.2), Vector3d(12.1, 9.7, 3.3));
    Body b3(12.2, Vector3d(0.9, -1.2, 0.73), Vector3d(1.1, 2.7, 2.1));
    Body b4(b1.mMass / 3., Vector3d(b1.mCenterOfMass / 3.), Matrix3d(b1.mInertia / 3.), b1.mCenterOfBuoyancy / 3., b1.volume / 3., SpatialMatrixZero,
            DragData(b1.dragData.linearDrag * 3., b1.dragData.quadraticDrag * 3.));

    // build the first model
    unsigned int root_body_id = m1->appendBody(Xrotz(-0.2) * Xroty(0.3) * Xrotx(0.7) * Xtrans(Vector3d(1.8, -2.2, 0.7)), Joint(JointTypeFloatingBase), b1, "a");
    m1->appendBody(Xrotz(0.1) * Xroty(-0.6) * Xrotx(1.7) * Xtrans(Vector3d(0.8, 1.2, 1.7)), Joint(JointTypeRevoluteZ), b2, "b");
    m1->appendBody(Xrotz(-2.2) * Xroty(1.3) * Xrotx(2.7) * Xtrans(Vector3d(2.8, 12.2, 2.7)), Joint(JointTypePrismatic, Vector3d(0., 0., 1.)), b3, "c");
    m1->addBody(root_body_id, Xrotz(-0.1) * Xroty(0.9) * Xrotx(-0.7) * Xtrans(Vector3d(-0.8, -1.2, -1.7)), Joint(JointTypeRevoluteY), b2, "d");
    m1->appendBody(Xrotz(-1.9) * Xroty(-2.3) * Xrotx(-1.17) * Xtrans(Vector3d(-2.8, -12.4, 1.7)), Joint(JointTypePrismatic, Vector3d(1., 0., 0.)), b3, "e");

    // build the second model, the first floating base link is the only thing different from the first one
    root_body_id = m2->appendBody(Xrotz(-0.2) * Xroty(0.3) * Xrotx(0.7) * Xtrans(Vector3d(1.8, -2.2, 0.7)), Joint(JointTypeFloatingBase), b4, "a");
    m2->appendBody(Xrotz(0.1) * Xroty(-0.6) * Xrotx(1.7) * Xtrans(Vector3d(0.8, 1.2, 1.7)), Joint(JointTypeRevoluteZ), b2, "b");
    m2->appendBody(Xrotz(-2.2) * Xroty(1.3) * Xrotx(2.7) * Xtrans(Vector3d(2.8, 12.2, 2.7)), Joint(JointTypePrismatic, Vector3d(0., 0., 1.)), b3, "c");
    m2->addBody(root_body_id, Xrotz(-0.1) * Xroty(0.9) * Xrotx(-0.7) * Xtrans(Vector3d(-0.8, -1.2, -1.7)), Joint(JointTypeRevoluteY), b2, "d");
    m2->appendBody(Xrotz(-1.9) * Xroty(-2.3) * Xrotx(-1.17) * Xtrans(Vector3d(-2.8, -12.4, 1.7)), Joint(JointTypePrismatic, Vector3d(1., 0., 0.)), b3, "e");

    m1->setupRosParameters();
    m2->setupRosParameters();

    rclcpp::AsyncParametersClient::SharedPtr client = std::make_shared<rclcpp::AsyncParametersClient>(n, n->get_name());

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Zero(m1->q_size);
    VectorNd QDot = VectorNd::Zero(m1->dof_count);
    VectorNd Tau = VectorNd::Zero(m1->dof_count);
    VectorNd QDDot_m1 = VectorNd::Zero(m1->dof_count);
    VectorNd QDDot_m2 = VectorNd::Zero(m1->dof_count);
    MatrixNd H_m1 = MatrixNd::Zero(m1->dof_count, m1->dof_count);
    MatrixNd H_m2 = MatrixNd::Zero(m1->dof_count, m1->dof_count);
    VectorNd G_m1 = VectorNd::Zero(m1->dof_count);
    VectorNd G_m2 = VectorNd::Zero(m1->dof_count);
    VectorNd B_m1 = VectorNd::Zero(m1->dof_count);
    VectorNd B_m2 = VectorNd::Zero(m1->dof_count);
    VectorNd N_m1 = VectorNd::Zero(m1->dof_count);
    VectorNd N_m2 = VectorNd::Zero(m1->dof_count);
    Vector3d com_m1, com_m2;
    FramePoint cob_m1, cob_m2;

    Quaternion o(0.2, 0.3, 0.4, 0.5);
    o.normalize();
    Q << 1.1, 1.3, 2.4, o.x(), o.y(), o.z(), 3.2, -0.9, 0.3, 1.3, o.w();
    QDot << -1.7, -2.1, 0.89, 9.2, 1.3, -0.9, 1.1, 3.3, 4.1, 12.1;
    Tau << 25.2, -76.22, 33.1, -9.9, -0.255, 1.2, 100.11, 0.9981, 1.33, 57.2;

    forwardHydroDynamics(*m1, Q, QDot, Tau, QDDot_m1);
    forwardHydroDynamics(*m2, Q, QDot, Tau, QDDot_m2);
    compositeRigidBodyAlgorithmHydro(*m1, Q, H_m1, false);
    compositeRigidBodyAlgorithmHydro(*m2, Q, H_m2, false);
    gravityEffects(*m1, G_m1);
    gravityEffects(*m2, G_m2);
    buoyancyEffects(*m1, B_m1);
    buoyancyEffects(*m2, B_m2);
    nonlinearEffectsHydro(*m1, Q, QDot, N_m1, false);
    nonlinearEffectsHydro(*m2, Q, QDot, N_m2, false);
    Utils::calcCenterOfMass(*m1, Q, com_m1, false);
    Utils::calcCenterOfMass(*m2, Q, com_m2, false);
    HydroUtils::calcCenterOfBuoyancy(*m1, Q, cob_m1, false);
    HydroUtils::calcCenterOfBuoyancy(*m2, Q, cob_m2, false);

    EXPECT_FALSE(H_m1.isApprox(H_m2, unit_test_utils::E_MINUS_9));
    EXPECT_FALSE(QDDot_m1.isApprox(QDDot_m2, unit_test_utils::E_MINUS_9));
    EXPECT_FALSE(G_m1.isApprox(G_m2, unit_test_utils::E_MINUS_9));
    EXPECT_FALSE(B_m1.isApprox(B_m2, unit_test_utils::E_MINUS_9));
    EXPECT_FALSE(N_m1.isApprox(N_m2, unit_test_utils::E_MINUS_9));
    EXPECT_FALSE(com_m1.isApprox(com_m2, unit_test_utils::E_MINUS_9));
    EXPECT_FALSE(cob_m1.isApprox(cob_m2, unit_test_utils::E_MINUS_9));

    // set the mass params of the floating base link on each model to be the same now
    rclcpp::Parameter m("m2/a_mass", b1.mMass), comx("m2/a_comx", b1.mCenterOfMass.x()), comy("m2/a_comy", b1.mCenterOfMass.y()), comz("m2/a_comz", b1.mCenterOfMass.z()),
        ixx("m2/a_ixx", b1.mInertia(0, 0)), ixy("m2/a_ixy", b1.mInertia(0, 1)), ixz("m2/a_ixz", b1.mInertia(0, 2)), iyy("m2/a_iyy", b1.mInertia(1, 1)),
        iyz("m2/a_iyz", b1.mInertia(1, 2)), izz("m2/a_izz", b1.mInertia(2, 2)), v("m2/a_volume", b1.volume), cobx("m2/a_cobx", b1.mCenterOfBuoyancy.x()),
        coby("m2/a_coby", b1.mCenterOfBuoyancy.y()), cobz("m2/a_cobz", b1.mCenterOfBuoyancy.z()), drag_linear_fx("m2/a_drag_linear_fx", b1.dragData.linearDrag[3]),
        drag_linear_fy("m2/a_drag_linear_fy", b1.dragData.linearDrag[4]), drag_linear_fz("m2/a_drag_linear_fz", b1.dragData.linearDrag[5]),
        drag_linear_mx("m2/a_drag_linear_mx", b1.dragData.linearDrag[0]), drag_linear_my("m2/a_drag_linear_my", b1.dragData.linearDrag[1]),
        drag_linear_mz("m2/a_drag_linear_mz", b1.dragData.linearDrag[2]), drag_quadratic_fx("m2/a_drag_quadratic_fx", b1.dragData.quadraticDrag[3]),
        drag_quadratic_fy("m2/a_drag_quadratic_fy", b1.dragData.quadraticDrag[4]), drag_quadratic_fz("m2/a_drag_quadratic_fz", b1.dragData.quadraticDrag[5]),
        drag_quadratic_mx("m2/a_drag_quadratic_mx", b1.dragData.quadraticDrag[0]), drag_quadratic_my("m2/a_drag_quadratic_my", b1.dragData.quadraticDrag[1]),
        drag_quadratic_mz("m2/a_drag_quadratic_mz", b1.dragData.quadraticDrag[2]);

    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future = client->set_parameters(std::vector<rclcpp::Parameter>({ m,
                                                                                                                                               comx,
                                                                                                                                               comy,
                                                                                                                                               comz,
                                                                                                                                               ixx,
                                                                                                                                               ixy,
                                                                                                                                               ixz,
                                                                                                                                               iyy,
                                                                                                                                               iyz,
                                                                                                                                               izz,
                                                                                                                                               v,
                                                                                                                                               cobx,
                                                                                                                                               coby,
                                                                                                                                               cobz,
                                                                                                                                               drag_linear_fx,
                                                                                                                                               drag_linear_fy,
                                                                                                                                               drag_linear_fz,
                                                                                                                                               drag_linear_mx,
                                                                                                                                               drag_linear_my,
                                                                                                                                               drag_linear_mz,
                                                                                                                                               drag_quadratic_fx,
                                                                                                                                               drag_quadratic_fy,
                                                                                                                                               drag_quadratic_fz,
                                                                                                                                               drag_quadratic_mx,
                                                                                                                                               drag_quadratic_my,
                                                                                                                                               drag_quadratic_mz }));

    EXPECT_EQ(rclcpp::spin_until_future_complete(n, future), rclcpp::FutureReturnCode::SUCCESS);

    forwardHydroDynamics(*m1, Q, QDot, Tau, QDDot_m1);
    forwardHydroDynamics(*m2, Q, QDot, Tau, QDDot_m2);
    compositeRigidBodyAlgorithmHydro(*m1, Q, H_m1, false);
    compositeRigidBodyAlgorithmHydro(*m2, Q, H_m2, false);
    gravityEffects(*m1, G_m1);
    gravityEffects(*m2, G_m2);
    buoyancyEffects(*m1, B_m1);
    buoyancyEffects(*m2, B_m2);
    nonlinearEffectsHydro(*m1, Q, QDot, N_m1, false);
    nonlinearEffectsHydro(*m2, Q, QDot, N_m2, false);
    Utils::calcCenterOfMass(*m1, Q, com_m1, false);
    Utils::calcCenterOfMass(*m2, Q, com_m2, false);
    HydroUtils::calcCenterOfBuoyancy(*m1, Q, cob_m1, false);
    HydroUtils::calcCenterOfBuoyancy(*m2, Q, cob_m2, false);

    EXPECT_TRUE(QDDot_m1.isApprox(QDDot_m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(H_m1.isApprox(H_m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(G_m1.isApprox(G_m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(B_m1.isApprox(B_m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(N_m1.isApprox(N_m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(com_m1.isApprox(com_m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(cob_m1.isApprox(cob_m2, unit_test_utils::E_MINUS_14));
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
