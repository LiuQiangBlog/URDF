// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>

#include "UnitTestUtils.hpp"
#include "Fixtures.hpp"
#include "Human36Fixture.hpp"
#include <rdl_dynamics/Kinematics.hpp>
#include <rdl_dynamics/rdl_hydro_utils.hpp>
#include <rdl_dynamics/rdl_utils.hpp>

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

struct RdlUtilsTests : public testing::Test
{
};

TEST_F(RdlUtilsTests, testGetDofName)
{
    SpatialVector rx(1., 0., 0., 0., 0., 0.);
    SpatialVector ry(0., 1., 0., 0., 0., 0.);
    SpatialVector rz(0., 0., 1., 0., 0., 0.);
    SpatialVector tx(0., 0., 0., 1., 0., 0.);
    SpatialVector ty(0., 0., 0., 0., 1., 0.);
    SpatialVector tz(0., 0., 0., 0., 0., 1.);

    EXPECT_STREQ("RX", Utils::getDofName(rx).c_str());
    EXPECT_STREQ("RY", Utils::getDofName(ry).c_str());
    EXPECT_STREQ("RZ", Utils::getDofName(rz).c_str());
    EXPECT_STREQ("TX", Utils::getDofName(tx).c_str());
    EXPECT_STREQ("TY", Utils::getDofName(ty).c_str());
    EXPECT_STREQ("TZ", Utils::getDofName(tz).c_str());

    SpatialVector c(1., 0., 0., 0., 0., 1.);

    ostringstream dof_stream(ostringstream::out);
    dof_stream << "custom (" << c.transpose() << ")";
    EXPECT_STREQ(dof_stream.str().c_str(), Utils::getDofName(c).c_str());
}

TEST_F(RdlUtilsTests, testGetBodyName)
{
    Model model;
    Body b1;
    b1.mIsVirtual = true;
    Joint j(JointType6DoF);
    j.mJointAxes[0] = SpatialVector(1., 0., 0., 0., 0., 0.);
    j.mJointAxes[1] = SpatialVector(0., 1., 0., 0., 0., 0.);
    j.mJointAxes[2] = SpatialVector(0., 0., 1., 0., 0., 0.);
    j.mJointAxes[3] = SpatialVector(0., 0., 0., 1., 0., 0.);
    j.mJointAxes[4] = SpatialVector(0., 0., 0., 0., 1., 0.);
    j.mJointAxes[5] = SpatialVector(0., 0., 0., 0., 0., 1.);
    unsigned int b_id = model.addBody(0, SpatialTransform(), j, b1, "b1");
    Body b2(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));
    ;
    unsigned int b_id2 = model.addBody(3, SpatialTransform(), Joint(JointTypeRevoluteY), b2, "b2");

    EXPECT_STREQ("b2", Utils::getBodyName(model, b_id2).c_str());
    EXPECT_STREQ("", Utils::getBodyName(model, b_id).c_str());
}

TEST_F(FloatingBase12DoF, TestKineticEnergy)
{
    VectorNd q = VectorNd::Zero(model->q_size);
    VectorNd qdot = VectorNd::Zero(model->q_size);

    for (unsigned int i = 0; i < q.size(); i++)
    {
        q[i] = 0.1 * i;
        qdot[i] = 0.3 * i;
    }

    MatrixNd H = MatrixNd::Zero(model->q_size, model->q_size);
    compositeRigidBodyAlgorithm(*model, q, H, true);

    double kinetic_energy_ref = 0.5 * qdot.transpose() * H * qdot;
    double ke = Utils::calcKineticEnergy(*model, q, qdot);

    EXPECT_EQ(ke, kinetic_energy_ref);
}

TEST_F(RdlUtilsTests, TestPotentialEnergy)
{
    Model model;
    Matrix3d inertia = Matrix3d::Identity(3, 3);
    Body body(0.5, Vector3d(0., 0., 0.), inertia);
    Joint joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.));

    model.appendBody(Xtrans(Vector3d::Zero()), joint, body);

    VectorNd q = VectorNd::Zero(model.q_size);
    double potential_energy_zero = Utils::calcPotentialEnergy(model, q);
    EXPECT_EQ(0., potential_energy_zero);

    q[1] = 1.;
    double potential_energy_lifted = Utils::calcPotentialEnergy(model, q);
    EXPECT_EQ(4.905, potential_energy_lifted);
}

TEST_F(RdlUtilsTests, TestCOMSimple)
{
    Model model;
    Matrix3d inertia = Matrix3d::Identity(3, 3);
    Body body(123., Vector3d(1., 2., 3.), inertia);
    Joint joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.));

    model.appendBody(Xtrans(Vector3d::Zero()), joint, body);

    VectorNd q = VectorNd::Zero(model.q_size);
    VectorNd qdot = VectorNd::Zero(model.qdot_size);

    Vector3d com;
    Vector3d com_velocity;
    FramePoint p_com, pcom_2;
    FrameVector v_com;
    Utils::calcCenterOfMass(model, q, qdot, com, &com_velocity);
    Utils::updateCenterOfMassFrame(model, com);
    Matrix3d eye = RobotDynamics::Math::Matrix3d::Identity();
    EXPECT_TRUE(model.comFrame->getTransformFromParent().r.isApprox(com, unit_test_utils::E_MINUS_10));
    EXPECT_TRUE(model.comFrame->getTransformFromParent().E.isApprox(eye, unit_test_utils::E_MINUS_10));
    Utils::calcCenterOfMass(model, q, qdot, p_com, v_com);
    Utils::updateCenterOfMassFrame(model, p_com.vec());
    EXPECT_TRUE(model.comFrame->getTransformFromParent().r.isApprox(p_com.vec(), unit_test_utils::E_MINUS_10));
    EXPECT_TRUE(model.comFrame->getTransformFromParent().E.isApprox(eye, unit_test_utils::E_MINUS_10));
    Utils::calcCenterOfMass(model, q, pcom_2);
    Utils::updateCenterOfMassFrame(model, pcom_2.vec());
    EXPECT_TRUE(model.comFrame->getTransformFromParent().r.isApprox(pcom_2.vec(), unit_test_utils::E_MINUS_10));
    EXPECT_TRUE(model.comFrame->getTransformFromParent().E.isApprox(eye, unit_test_utils::E_MINUS_10));

    EXPECT_EQ(model.mass, 123.);
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(Vector3d(1., 2., 3.), com));
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(pcom_2.vec(), com));
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(p_com.vec(), Vector3d(1., 2., 3.)));
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(Vector3d(0., 0., 0.), com_velocity));

    q[1] = 1.;
    Utils::calcCenterOfMass(model, q, qdot, com, &com_velocity);
    Utils::updateCenterOfMassFrame(model, com);
    EXPECT_TRUE(model.comFrame->getTransformFromParent().r.isApprox(com, unit_test_utils::E_MINUS_10));
    EXPECT_TRUE(model.comFrame->getTransformFromParent().E.isApprox(eye, unit_test_utils::E_MINUS_10));
    Utils::calcCenterOfMass(model, q, qdot, p_com);
    Utils::updateCenterOfMassFrame(model, p_com.vec());
    EXPECT_TRUE(model.comFrame->getTransformFromParent().r.isApprox(p_com.vec(), unit_test_utils::E_MINUS_10));
    EXPECT_TRUE(model.comFrame->getTransformFromParent().E.isApprox(eye, unit_test_utils::E_MINUS_10));
    Utils::calcCenterOfMass(model, q, pcom_2);
    Utils::updateCenterOfMassFrame(model, pcom_2.vec());
    EXPECT_TRUE(model.comFrame->getTransformFromParent().r.isApprox(pcom_2.vec(), unit_test_utils::E_MINUS_10));
    EXPECT_TRUE(model.comFrame->getTransformFromParent().E.isApprox(eye, unit_test_utils::E_MINUS_10));

    EXPECT_TRUE(unit_test_utils::checkVector3dEq(Vector3d(1., 3., 3.), p_com.vec()));
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(pcom_2.vec(), com));
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(Vector3d(1., 3., 3.), com));
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(Vector3d(0., 0., 0.), com_velocity));

    qdot[1] = 1.;
    Utils::calcCenterOfMass(model, q, qdot, com, &com_velocity);
    Utils::calcCenterOfMass(model, q, qdot, p_com);
    Utils::calcCenterOfMass(model, q, pcom_2);
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(Vector3d(1., 3., 3.), com));
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(pcom_2.vec(), com));
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(Vector3d(1., 3., 3.), p_com.vec()));
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(Vector3d(0., 1., 0.), com_velocity));
}

TEST_F(RdlUtilsTests, TestCOMWrench)
{
    Model model;
    double g = -9.81;
    model.gravity = MotionVector(0., 0., 0., 0., 0., g);
    Matrix3d inertia = Matrix3d::Identity(3, 3);
    Body b1(2., Vector3d(0., 0., 0.), inertia);
    Joint tx(JointTypePrismatic, Math::Vector3d(1., 0., 0.));

    model.appendBody(Xtrans(Vector3d(1., 0., 0.)), tx, b1);

    VectorNd q = VectorNd::Zero(model.q_size);
    VectorNd qdot = VectorNd::Zero(model.qdot_size);

    Vector3d com;
    Utils::calcCenterOfMass(model, q, qdot, com, nullptr);
    Utils::updateCenterOfMassFrame(model, com);
    SpatialForce gravity_wrench = Utils::calcGravityWrenchOnCenterOfMass(model, q);
    EXPECT_TRUE(gravity_wrench.isApprox(SpatialVector(0., 0., 0., 0., 0., 2. * g), unit_test_utils::E_MINUS_10));

    EXPECT_TRUE(com.isApprox(Vector3d(1., 0., 0.), 1.e-12));

    model.addBody(0, Xtrans(Vector3d(-1., 0., 0.)), tx, b1);
    q = VectorNd::Zero(model.q_size);
    qdot = VectorNd::Zero(model.qdot_size);

    Utils::calcCenterOfMass(model, q, qdot, com, nullptr);
    Utils::updateCenterOfMassFrame(model, com);
    gravity_wrench = Utils::calcGravityWrenchOnCenterOfMass(model, q);
    EXPECT_TRUE(gravity_wrench.isApprox(SpatialVector(0., 0., 0., 0., 0., 4. * g), unit_test_utils::E_MINUS_10));

    EXPECT_NEAR(com[0], 0., 1.e-12);
    EXPECT_NEAR(com[1], 0., 1.e-12);
    EXPECT_NEAR(com[2], 0., 1.e-12);

    model.addBody(0, Xtrans(Vector3d(0., 1., 0.)), tx, b1);
    model.addBody(0, Xtrans(Vector3d(0., -1., 0.)), tx, b1);
    q = VectorNd::Zero(model.q_size);
    qdot = VectorNd::Zero(model.qdot_size);

    Utils::calcCenterOfMass(model, q, qdot, com, nullptr);
    Utils::updateCenterOfMassFrame(model, com);
    gravity_wrench = Utils::calcGravityWrenchOnCenterOfMass(model, q);
    EXPECT_TRUE(gravity_wrench.isApprox(SpatialVector(0., 0., 0., 0., 0., 8. * g), unit_test_utils::E_MINUS_10));

    EXPECT_NEAR(com[0], 0., 1.e-12);
    EXPECT_NEAR(com[1], 0., 1.e-12);
    EXPECT_NEAR(com[2], 0., 1.e-12);
}

TEST_F(RdlUtilsTests, TestCOMWrenchYawRotatedCOM)
{
    Model model;
    double g = -9.81;
    double mass = 2.;
    model.gravity = MotionVector(0., 0., 0., 0., 0., g);
    Matrix3d inertia = Matrix3d::Identity(3, 3);
    Body b1(mass, Vector3d(0., 0., 0.), inertia);
    Joint tx(JointTypePrismatic, Math::Vector3d(1., 0., 0.));

    model.appendBody(Xtrans(Vector3d(1., 0., 0.)), tx, b1);

    VectorNd q = VectorNd::Zero(model.q_size);
    VectorNd qdot = VectorNd::Zero(model.qdot_size);

    Vector3d com;
    Utils::calcCenterOfMass(model, q, qdot, com, nullptr);
    Utils::updateCenterOfMassFrame(model, com, Vector3d(M_PI, 0., 0.));
    SpatialForce gravity_wrench = Utils::calcGravityWrenchOnCenterOfMass(model, q);
    EXPECT_TRUE(gravity_wrench.isApprox(SpatialVector(0., 0., 0., 0., 0., model.mass * g), unit_test_utils::E_MINUS_10));
}

TEST_F(RdlUtilsTests, TestCOMWrenchRollRotatedCOM)
{
    Model model;
    double g = -9.81;
    double mass = 2.;
    model.gravity = MotionVector(0., 0., 0., 0., 0., g);
    Matrix3d inertia = Matrix3d::Identity(3, 3);
    Body b1(mass, Vector3d(0., 0., 0.), inertia);
    Joint tx(JointTypePrismatic, Math::Vector3d(1., 0., 0.));

    model.appendBody(Xtrans(Vector3d(1., 0., 0.)), tx, b1);

    VectorNd q = VectorNd::Zero(model.q_size);
    VectorNd qdot = VectorNd::Zero(model.qdot_size);

    Vector3d com;
    Utils::calcCenterOfMass(model, q, qdot, com, nullptr);
    Utils::updateCenterOfMassFrame(model, com, Vector3d(0., 0., M_PI));
    SpatialForce gravity_wrench = Utils::calcGravityWrenchOnCenterOfMass(model, q);
    EXPECT_TRUE(gravity_wrench.isApprox(SpatialVector(0., 0., 0., 0., 0., -model.mass * g), unit_test_utils::E_MINUS_10));
}

TEST_F(RdlUtilsTests, TestCOMWrenchRollRotatedQuaternionCOM)
{
    Model model;
    double g = -9.81;
    double mass = 2.;
    model.gravity = MotionVector(0., 0., 0., 0., 0., g);
    Matrix3d inertia = Matrix3d::Identity(3, 3);
    Body b1(mass, Vector3d(0., 0., 0.), inertia);
    Joint tx(JointTypePrismatic, Math::Vector3d(1., 0., 0.));

    model.appendBody(Xtrans(Vector3d(1., 0., 0.)), tx, b1);

    VectorNd q = VectorNd::Zero(model.q_size);
    VectorNd qdot = VectorNd::Zero(model.qdot_size);

    Vector3d com;
    Utils::calcCenterOfMass(model, q, qdot, com, nullptr);
    Utils::updateCenterOfMassFrame(model, com, Math::intrinsicZYXAnglesToQuaternion(Vector3d(0., 0., M_PI)));
    SpatialForce gravity_wrench = Utils::calcGravityWrenchOnCenterOfMass(model, q);
    EXPECT_TRUE(gravity_wrench.isApprox(SpatialVector(0., 0., 0., 0., 0., -model.mass * g), unit_test_utils::E_MINUS_10));
}

TEST_F(RdlUtilsTests, TestCOMWrenchPitchRotatedCOM)
{
    Model model;
    double g = -9.81;
    double mass = 2.;
    model.gravity = MotionVector(0., 0., 0., 0., 0., g);
    Matrix3d inertia = Matrix3d::Identity(3, 3);
    Body b1(mass, Vector3d(0., 0., 0.), inertia);
    Joint tx(JointTypePrismatic, Math::Vector3d(1., 0., 0.));

    model.appendBody(Xtrans(Vector3d(1., 0., 0.)), tx, b1);

    VectorNd q = VectorNd::Zero(model.q_size);
    VectorNd qdot = VectorNd::Zero(model.qdot_size);

    Vector3d com;
    Utils::calcCenterOfMass(model, q, qdot, com, nullptr);
    Utils::updateCenterOfMassFrame(model, com, Vector3d(0., M_PI, 0.));
    SpatialForce gravity_wrench = Utils::calcGravityWrenchOnCenterOfMass(model, q);
    EXPECT_TRUE(gravity_wrench.isApprox(SpatialVector(0., 0., 0., 0., 0., -model.mass * g), unit_test_utils::E_MINUS_10));
}

TEST_F(RdlUtilsTests, TestCOMWrenchPitchRotatedCOMQuaternion)
{
    Model model;
    double g = -9.81;
    double mass = 2.;
    model.gravity = MotionVector(0., 0., 0., 0., 0., g);
    Matrix3d inertia = Matrix3d::Identity(3, 3);
    Body b1(mass, Vector3d(0., 0., 0.), inertia);
    Joint tx(JointTypePrismatic, Math::Vector3d(1., 0., 0.));

    model.appendBody(Xtrans(Vector3d(1., 0., 0.)), tx, b1);

    VectorNd q = VectorNd::Zero(model.q_size);
    VectorNd qdot = VectorNd::Zero(model.qdot_size);

    Vector3d com;
    Utils::calcCenterOfMass(model, q, qdot, com, nullptr);
    Utils::updateCenterOfMassFrame(model, com, Math::intrinsicZYXAnglesToQuaternion(Vector3d(0., M_PI, 0.)));
    SpatialForce gravity_wrench = Utils::calcGravityWrenchOnCenterOfMass(model, q);
    EXPECT_TRUE(gravity_wrench.isApprox(SpatialVector(0., 0., 0., 0., 0., -model.mass * g), unit_test_utils::E_MINUS_10));
}

TEST_F(Human36, TestCOM)
{
    randomizeStates();

    Vector3d com, com_velocity;
    Utils::calcCenterOfMass(*model, q, qdot, com, &com_velocity);

    FrameVector com_v2;

    FramePoint p_com, pcom_2;
    FrameVector v_com;
    Utils::calcCenterOfMass(*model, q, qdot, p_com, v_com);
    Utils::calcCenterOfMassVelocity(*model, q, qdot, com_v2);
    Utils::updateCenterOfMassFrame(*model, p_com.vec());
    Matrix3d eye = RobotDynamics::Math::Matrix3d::Identity();
    EXPECT_TRUE(model->comFrame->getTransformFromParent().r.isApprox(p_com.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(model->comFrame->getTransformFromParent().E.isApprox(eye, unit_test_utils::E_MINUS_14));
    Utils::calcCenterOfMass(*model, q, pcom_2);
    Utils::updateCenterOfMassFrame(*model, pcom_2.vec());
    EXPECT_TRUE(model->comFrame->getTransformFromParent().r.isApprox(pcom_2.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(model->comFrame->getTransformFromParent().E.isApprox(eye, unit_test_utils::E_MINUS_14));

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(com, p_com.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(pcom_2.vec(), p_com.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(com_velocity, v_com, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(com_velocity, com_v2, unit_test_utils::E_MINUS_14));

    randomizeStates();

    Utils::calcCenterOfMass(*model_3dof, q, qdot, com, &com_velocity);
    Utils::updateCenterOfMassFrame(*model_3dof, com);
    Utils::calcCenterOfMassVelocity(*model, q, qdot, com_v2);
    EXPECT_TRUE(model_3dof->comFrame->getTransformFromParent().r.isApprox(com, unit_test_utils::E_MINUS_10));
    EXPECT_TRUE(model_3dof->comFrame->getTransformFromParent().E.isApprox(eye, unit_test_utils::E_MINUS_10));
    Utils::calcCenterOfMass(*model_3dof, q, qdot, p_com, v_com);
    Utils::updateCenterOfMassFrame(*model_3dof, p_com.vec());
    EXPECT_TRUE(model_3dof->comFrame->getTransformFromParent().r.isApprox(p_com.vec(), unit_test_utils::E_MINUS_10));
    EXPECT_TRUE(model_3dof->comFrame->getTransformFromParent().E.isApprox(eye, unit_test_utils::E_MINUS_10));
    Utils::calcCenterOfMass(*model_3dof, q, pcom_2);
    Utils::updateCenterOfMassFrame(*model_3dof, pcom_2.vec());
    EXPECT_TRUE(model_3dof->comFrame->getTransformFromParent().r.isApprox(pcom_2.vec(), unit_test_utils::E_MINUS_10));
    EXPECT_TRUE(model_3dof->comFrame->getTransformFromParent().E.isApprox(eye, unit_test_utils::E_MINUS_10));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(com, p_com.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(pcom_2.vec(), p_com.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(com_velocity, v_com, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(com_velocity, com_v2, unit_test_utils::E_MINUS_14));

    randomizeStates();

    Utils::calcCenterOfMass(*model_emulated, q, qdot, com, &com_velocity);
    Utils::calcCenterOfMass(*model_emulated, q, qdot, p_com, v_com);
    Utils::calcCenterOfMass(*model_emulated, q, pcom_2);
    Utils::calcCenterOfMassVelocity(*model, q, qdot, com_v2);
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(com, p_com.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(pcom_2.vec(), p_com.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(com_velocity, v_com, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(com_v2, v_com, unit_test_utils::E_MINUS_14));
}

TEST_F(Human36, TestCOB)
{
    // Human model is setup such that the COM and COB should be
    // at the same spot
    randomizeStates();

    RobotDynamics::Math::FramePoint com, cob;
    cob = HydroUtils::updateCenterOfBuoyancyFrame(*model, q);
    Utils::calcCenterOfMass(*model, q, qdot, com, nullptr, false);

    EXPECT_TRUE(com.isApprox(cob, unit_test_utils::E_MINUS_14));
}

TEST_F(Human36, TestComAndCob)
{
    // Human model is setup such that the COM and COB should be
    // at the same spot
    Vector3d com, cob, com_velocity;
    HydroUtils::calcCenterOfMassAndCenterOfBuoyancy(*model, q, qdot, com, cob, com_velocity);
    Utils::calcCenterOfMassVelocity(*model, com_velocity);

    EXPECT_TRUE(com.isApprox(cob, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlUtilsTests, TestCOBWrenchOnCOM)
{
    RobotDynamics::Model model;
    model.fluidDensity = 1000.;
    double g = -9.81;
    model.gravity = MotionVector(0., 0., 0., 0., 0., g);

    RobotDynamics::Body b1(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.), Vector3d(-1., 0., 0.), 0.001, SpatialMatrixZero);

    model.appendBody(SpatialTransform(), Joint(JointTypeRevoluteX), b1, "b1");

    VectorNd q = VectorNd::Zero(model.q_size);
    VectorNd qdot = VectorNd::Zero(model.qdot_size);

    RobotDynamics::Utils::updateCenterOfMassFrame(model, q);
    SpatialForce buoyancy_wrench = RobotDynamics::HydroUtils::calcBuoyancyWrenchOnCenterOfMass(model, q, false);
    double y_axis_moment = -model.fluidDensity * g * 0.001 * 2.;
    double z_axis_force = -model.fluidDensity * g * 0.001;
    EXPECT_TRUE(buoyancy_wrench.isApprox(SpatialVector(0., y_axis_moment, 0., 0., 0., z_axis_force), unit_test_utils::E_MINUS_10));

    RobotDynamics::Utils::updateCenterOfMassFrame(model, q, Vector3d(0., M_PI_2, 0.));

    buoyancy_wrench = RobotDynamics::HydroUtils::calcBuoyancyWrenchOnCenterOfMass(model, q, false);
    y_axis_moment = -model.fluidDensity * g * 0.001 * 2.;
    double x_axis_force = model.fluidDensity * g * 0.001;  // com x-axis is down, so have to negate it
    EXPECT_TRUE(buoyancy_wrench.isApprox(SpatialVector(0., y_axis_moment, 0., x_axis_force, 0., 0.), unit_test_utils::E_MINUS_10));
}

TEST_F(RdlUtilsTests, TestCOBWrenchOnCOMTwoBodies)
{
    RobotDynamics::Model model;
    model.fluidDensity = 1000.;
    double g = -9.81;
    model.gravity = MotionVector(0., 0., 0., 0., 0., g);

    RobotDynamics::Body b1(1., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.), Vector3d(-1., 0., 0.), 0.001, SpatialMatrixZero);
    RobotDynamics::Body b2(1., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.), Vector3d(0., -1., 0.), 0.001, SpatialMatrixZero);

    model.appendBody(SpatialTransform(), Joint(JointTypeRevoluteX), b1, "b1");
    model.addBody(0, SpatialTransform(), Joint(JointTypeRevoluteX), b2, "b2");

    VectorNd q = VectorNd::Zero(model.q_size);
    VectorNd qdot = VectorNd::Zero(model.qdot_size);

    RobotDynamics::Utils::updateCenterOfMassFrame(model, q);
    SpatialForce buoyancy_wrench = RobotDynamics::HydroUtils::calcBuoyancyWrenchOnCenterOfMass(model, q, false);
    double y_axis_moment = -model.fluidDensity * g * 0.001 * 1.;
    double x_axis_moment = model.fluidDensity * g * 0.001 * 1.;
    double z_axis_force = -2. * model.fluidDensity * g * 0.001;  // two buoyant  bodies
    EXPECT_TRUE(buoyancy_wrench.isApprox(SpatialVector(x_axis_moment, y_axis_moment, 0., 0., 0., z_axis_force), unit_test_utils::E_MINUS_10));

    RobotDynamics::Utils::updateCenterOfMassFrame(model, q, Vector3d(0., M_PI_2, 0.), false);
    buoyancy_wrench = RobotDynamics::HydroUtils::calcBuoyancyWrenchOnCenterOfMass(model, q, false);
    y_axis_moment = -model.fluidDensity * g * 0.001 * 1.;
    double z_axis_moment = model.fluidDensity * g * 0.001 * 1.;
    double x_axis_force = 2. * model.fluidDensity * g * 0.001;  // two buoyant  bodies, x is pointing down
    EXPECT_TRUE(buoyancy_wrench.isApprox(SpatialVector(0., y_axis_moment, z_axis_moment, x_axis_force, 0., 0.), unit_test_utils::E_MINUS_10));
}

TEST_F(RdlUtilsTests, TestCOBWrenchOnCOMTwoBodiesQuaternion)
{
    RobotDynamics::Model model;
    model.fluidDensity = 1000.;
    double g = -9.81;
    model.gravity = MotionVector(0., 0., 0., 0., 0., g);

    RobotDynamics::Body b1(1., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.), Vector3d(-1., 0., 0.), 0.001, SpatialMatrixZero);
    RobotDynamics::Body b2(1., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.), Vector3d(0., -1., 0.), 0.001, SpatialMatrixZero);

    model.appendBody(SpatialTransform(), Joint(JointTypeRevoluteX), b1, "b1");
    model.addBody(0, SpatialTransform(), Joint(JointTypeRevoluteX), b2, "b2");

    VectorNd q = VectorNd::Zero(model.q_size);
    VectorNd qdot = VectorNd::Zero(model.qdot_size);

    RobotDynamics::Utils::updateCenterOfMassFrame(model, q);
    SpatialForce buoyancy_wrench = RobotDynamics::HydroUtils::calcBuoyancyWrenchOnCenterOfMass(model, q, false);
    double y_axis_moment = -model.fluidDensity * g * 0.001 * 1.;
    double x_axis_moment = model.fluidDensity * g * 0.001 * 1.;
    double z_axis_force = -2. * model.fluidDensity * g * 0.001;  // two buoyant  bodies
    EXPECT_TRUE(buoyancy_wrench.isApprox(SpatialVector(x_axis_moment, y_axis_moment, 0., 0., 0., z_axis_force), unit_test_utils::E_MINUS_10));

    RobotDynamics::Utils::updateCenterOfMassFrame(model, q, intrinsicZYXAnglesToQuaternion(Vector3d(0., M_PI_2, 0.)), false);
    buoyancy_wrench = RobotDynamics::HydroUtils::calcBuoyancyWrenchOnCenterOfMass(model, q, false);
    y_axis_moment = -model.fluidDensity * g * 0.001 * 1.;
    double z_axis_moment = model.fluidDensity * g * 0.001 * 1.;
    double x_axis_force = 2. * model.fluidDensity * g * 0.001;  // two buoyant  bodies, x is pointing down
    EXPECT_TRUE(buoyancy_wrench.isApprox(SpatialVector(0., y_axis_moment, z_axis_moment, x_axis_force, 0., 0.), unit_test_utils::E_MINUS_10));
}

TEST_F(RdlUtilsTests, DragWrenchOnCOM)
{
    RobotDynamics::Model model;
    model.fluidDensity = 1000.;
    double g = -9.81;
    model.gravity = MotionVector(0., 0., 0., 0., 0., g);
    DragData data;
    data.linearDrag << 1., 2., 3., 4., 5., 6.;
    data.quadraticDrag << 6., 5., 4., 3., 2., 1.;
    RobotDynamics::Body b1(1., Vector3d(0., 2., 0.), Vector3d(1., 1., 1.), Vector3dZero, 0.001, SpatialMatrixZero, data);
    RobotDynamics::Body b2(1., Vector3d(0., -2., 0.), Vector3d(1., 1., 1.), Vector3dZero, 0.001, SpatialMatrixZero, data);

    model.appendBody(SpatialTransform(), Joint(JointTypePrismatic, Vector3d(1., 0., 0.)), b1, "b1");

    VectorNd q = VectorNd::Zero(model.q_size);
    VectorNd qdot = VectorNd::Zero(model.qdot_size);
    Utils::updateCenterOfMassFrame(model, q);
    Math::SpatialForce drag_wrench = HydroUtils::calcDragWrenchOnCenterOfMass(model, q, qdot);
    EXPECT_NEAR(drag_wrench.mx(), 0., unit_test_utils::E_MINUS_10);
    EXPECT_NEAR(drag_wrench.my(), 0., unit_test_utils::E_MINUS_10);
    EXPECT_NEAR(drag_wrench.mz(), 0., unit_test_utils::E_MINUS_10);
    EXPECT_NEAR(drag_wrench.fx(), 0., unit_test_utils::E_MINUS_10);
    EXPECT_NEAR(drag_wrench.fy(), 0., unit_test_utils::E_MINUS_10);
    EXPECT_NEAR(drag_wrench.fz(), 0., unit_test_utils::E_MINUS_10);

    qdot[0] = 2.;

    drag_wrench = HydroUtils::calcDragWrenchOnCenterOfMass(model, q, qdot);
    EXPECT_TRUE(drag_wrench.isApprox(SpatialVector(0., 0., 2. * (data.linearDrag[3] * qdot[0] + data.quadraticDrag[3] * std::abs(qdot[0]) * qdot[0]),
                                                   data.linearDrag[3] * qdot[0] + data.quadraticDrag[3] * std::abs(qdot[0]) * qdot[0], 0., 0.),
                                     unit_test_utils::E_MINUS_10));

    model.addBody(0, SpatialTransform(), Joint(JointTypePrismatic, Vector3d(1., 0., 0.)), b2, "b2");

    q = VectorNd::Zero(model.q_size);
    qdot = VectorNd::Zero(model.qdot_size);
    Utils::updateCenterOfMassFrame(model, q);

    qdot[0] = 2.;
    qdot[1] = 2.;
    drag_wrench = HydroUtils::calcDragWrenchOnCenterOfMass(model, q, qdot);
    // The moments should cancel out and we should just be left with twice the force
    EXPECT_TRUE(drag_wrench.isApprox(SpatialVector(0., 0., 0., 40., 0., 0.), unit_test_utils::E_MINUS_10));
}

TEST_F(Human36, TestCOMCalcSubtreeMass)
{
    double subtreeMass;
    Vector3d com, com_velocity;

    Utils::calcCenterOfMass(*model_emulated, q, qdot, com, nullptr);
    subtreeMass = Utils::calcSubtreeMass(*model_emulated, 0);

    EXPECT_EQ(subtreeMass, model_emulated->mass);

    subtreeMass = Utils::calcSubtreeMass(*model_emulated, body_id_emulated[BodyPelvis] + 1);
    EXPECT_EQ(subtreeMass, 10.3368 + 3.1609 + 1.001);
}

TEST_F(Human36, calcSubtreeVolume)
{
    Model model;
    model.fluidDensity = 100.0;

    RobotDynamics::Body b1(1, Vector3dZero, Vector3d(1., 1., 1.), Vector3dZero, 0.5, SpatialMatrixZero);
    RobotDynamics::Body b2(2, Vector3dZero, Vector3d(1., 1., 1.), Vector3dZero, 0.4, SpatialMatrixZero);
    RobotDynamics::Body b3(3, Vector3dZero, Vector3d(1., 1., 1.), Vector3dZero, 0.3, SpatialMatrixZero);
    RobotDynamics::Body b4(4, Vector3dZero, Vector3d(1., 1., 1.), Vector3dZero, 0.2, SpatialMatrixZero);
    RobotDynamics::Body b5(5, Vector3dZero, Vector3d(1., 1., 1.), Vector3dZero, 0.1, SpatialMatrixZero);

    model.appendBody(SpatialTransform(), Joint(JointTypeRevoluteX), b1, "");
    model.addBody(1, SpatialTransform(), Joint(JointTypeRevoluteX), b2, "");
    model.addBody(0, SpatialTransform(), Joint(JointTypeRevoluteX), b3, "");
    model.addBody(2, SpatialTransform(), Joint(JointTypeRevoluteX), b4, "");
    model.addBody(0, SpatialTransform(), Joint(JointTypeRevoluteX), b5, "");

    EXPECT_NEAR(RobotDynamics::Utils::calcSubtreeMass(model, 0), 15., unit_test_utils::E_MINUS_10);
    EXPECT_NEAR(RobotDynamics::HydroUtils::calcSubtreeVolume(model, 0), 1.5, unit_test_utils::E_MINUS_10);
    EXPECT_NEAR(RobotDynamics::Utils::calcSubtreeMass(model, 0) - 100.0 * RobotDynamics::HydroUtils::calcSubtreeVolume(model, 0),
                RobotDynamics::HydroUtils::calcSubtreeWeightInFluid(model, 0), unit_test_utils::E_MINUS_10);

    RobotDynamics::Body b6(6, Vector3dZero, Vector3d(1., 1., 1.));
    model.addBody(1, SpatialTransform(), Joint(JointTypeFixed), b6, "");

    EXPECT_NEAR(RobotDynamics::Utils::calcSubtreeMass(model, 0), 21., unit_test_utils::E_MINUS_10);
    EXPECT_NEAR(RobotDynamics::HydroUtils::calcSubtreeVolume(model, 0), 1.5, unit_test_utils::E_MINUS_10);
    EXPECT_NEAR(RobotDynamics::Utils::calcSubtreeMass(model, 0) - 100.0 * RobotDynamics::HydroUtils::calcSubtreeVolume(model, 0),
                RobotDynamics::HydroUtils::calcSubtreeWeightInFluid(model, 0), unit_test_utils::E_MINUS_10);
}

TEST_F(RdlUtilsTests, TestCOMJacobian)
{
    Model model;

    unsigned int id1 = model.addBody(0, Xtrans(Vector3d(0, 1, 0)), Joint(SpatialVector(1, 0, 0, 0, 0, 0)), Body(2, Vector3d(0, 1, 0), Vector3d(1, 1, 1)));
    model.addBody(id1, Xtrans(Vector3d(0, 1, 0)), Joint(SpatialVector(1, 0, 0, 0, 0, 0)), Body(3, Vector3d(0, 1, 0), Vector3d(1, 1, 1)));

    Vector3d com, com_velocity;

    MatrixNd J_com(3, model.qdot_size);
    J_com.setZero();

    VectorNd q(2);
    VectorNd qdot(2);

    q << 0.1, -0.4;
    qdot << 0.9, -0.125;

    Utils::calcCenterOfMassJacobian(model, q, J_com);

    Utils::calcCenterOfMass(model, q, qdot, com, &com_velocity);
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(J_com * qdot, com_velocity, unit_test_utils::E_MINUS_14));
}

TEST_F(Human36, TestCOMJacobianHuman36)
{
    randomizeStates();

    Vector3d com, com_velocity;

    MatrixNd J_com(3, model_emulated->qdot_size);
    J_com.setZero();

    Utils::calcCenterOfMassJacobian(*model, q, J_com);
    Utils::calcCenterOfMass(*model, q, qdot, com, &com_velocity);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(J_com * qdot, com_velocity, unit_test_utils::E_MINUS_14));

    J_com.setZero();

    Utils::calcCenterOfMassJacobian(*model_3dof, q, J_com);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(J_com * qdot, com_velocity, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlUtilsTests, TestAngularMomentumSimple)
{
    Model model;
    Matrix3d inertia = Matrix3d::Zero(3, 3);
    inertia(0, 0) = 1.1;
    inertia(1, 1) = 2.2;
    inertia(2, 2) = 3.3;

    Body body(0.5, Vector3d(1., 0., 0.), inertia);
    Joint joint(SpatialVector(1., 0., 0., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(0., 0., 1., 0., 0., 0.));

    model.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint, body);

    VectorNd q = VectorNd::Zero(model.q_size);
    VectorNd qdot = VectorNd::Zero(model.qdot_size);

    Vector3d com, comv;
    Vector3d angular_momentum;

    FramePoint p_com;
    FrameVector v_com, ang_momentum;

    qdot << 1., 0., 0.;
    Utils::calcCenterOfMass(model, q, qdot, com, &comv, &angular_momentum);
    Utils::calcCenterOfMass(model, q, qdot, p_com, v_com, &ang_momentum);
    EXPECT_EQ(Vector3d(1.1, 0., 0.), angular_momentum);
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1.1, 0., 0.), ang_momentum, unit_test_utils::E_MINUS_14));

    qdot << 0., 1., 0.;
    Utils::calcCenterOfMass(model, q, qdot, com, &comv, &angular_momentum);
    Utils::calcCenterOfMass(model, q, qdot, p_com, v_com, &ang_momentum);
    EXPECT_EQ(Vector3d(0., 2.2, 0.), angular_momentum);
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 2.2, 0.), ang_momentum, unit_test_utils::E_MINUS_14));

    qdot << 0., 0., 1.;
    Utils::calcCenterOfMass(model, q, qdot, com, &comv, &angular_momentum);
    Utils::calcCenterOfMass(model, q, qdot, p_com, v_com, &ang_momentum);
    EXPECT_EQ(Vector3d(0., 0., 3.3), angular_momentum);
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 0., 3.3), ang_momentum, unit_test_utils::E_MINUS_14));
}

TEST_F(TwoArms12DoF, TestAngularMomentumSimple)
{
    Vector3d com;
    Vector3d angular_momentum;

    Utils::calcCenterOfMass(*model, q, qdot, com, NULL, &angular_momentum);

    EXPECT_EQ(Vector3d(0., 0., 0.), angular_momentum);

    qdot[0] = 1.;
    qdot[1] = 2.;
    qdot[2] = 3.;
    Utils::calcCenterOfMass(*model, q, qdot, com, NULL, &angular_momentum);

    // only a rough guess from test calculation
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(3.3, 2.54, 1.5), angular_momentum, 1.0e-1));

    qdot[3] = -qdot[0];
    qdot[4] = -qdot[1];
    qdot[5] = -qdot[2];

    Utils::calcCenterOfMass(*model, q, qdot, com, NULL, &angular_momentum);

    EXPECT_TRUE(angular_momentum[0] == 0);
    EXPECT_TRUE(angular_momentum[1] < 0);
    EXPECT_TRUE(angular_momentum[2] == 0.);
}

TEST_F(TwoArms12DoF, calcCentroidalMomentumMatrix)
{
    Vector3d com, com_velocity;
    Vector3d angular_momentum;
    MatrixNd A(6, model->qdot_size);
    A.setZero();

    randomizeStates();

    Utils::calcCenterOfMass(*model, q, qdot, com, &com_velocity, &angular_momentum);
    Utils::calcCentroidalMomentumMatrix(*model, q, A);

    SpatialVector m_exp;
    m_exp.setLinearPart(com_velocity * model->mass);
    m_exp.setAngularPart(angular_momentum);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(SpatialVector(A * qdot), m_exp, unit_test_utils::E_MINUS_12));
}

TEST_F(RdlUtilsTests, calcCentroidalMomentumMatrix)
{
    Model model;

    Body b1(1., Vector3d(0., 0., -0.1), Vector3d(1., 1., 1.));
    Joint j1(JointTypeRevoluteX);

    unsigned int id = model.addBody(0, SpatialTransform(), j1, b1, "b1");

    model.addBody(id, Xtrans(Vector3d(0., 0., -1.)), j1, b1, "b2");

    VectorNd Q(model.q_size);
    VectorNd QDot(model.qdot_size);
    Q.setZero();
    QDot.setZero();
    QDot[0] = 0.1;
    QDot[1] = 0.1;
    MatrixNd A(6, model.qdot_size);
    A.setZero();

    Vector3d com, com_velocity, ang_momentum;
    Utils::calcCenterOfMass(model, Q, QDot, com, &com_velocity, &ang_momentum);

    SpatialVector m_exp;
    m_exp.setAngularPart(ang_momentum);
    m_exp.setLinearPart(com_velocity * model.mass);

    Utils::calcCentroidalMomentumMatrix(model, Q, A, true);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(SpatialVector(A * QDot), m_exp, unit_test_utils::E_MINUS_12));
}

TEST_F(FixedBase3DoFPlanar, calcCentroidalMomentumMatrix)
{
    Vector3d com, com_velocity, ang_momentum;

    randomizeStates();

    MatrixNd A(6, model->qdot_size), G(3, model->qdot_size);
    A.setZero();

    Utils::calcCenterOfMass(*model, Q, QDot, com, &com_velocity, &ang_momentum);
    Utils::calcCentroidalMomentumMatrix(*model, Q, A);

    SpatialVector m_exp;
    m_exp.setLinearPart(com_velocity * model->mass);
    m_exp.setAngularPart(ang_momentum);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(SpatialVector(A * QDot), m_exp, unit_test_utils::E_MINUS_12));
}

TEST_F(Human36, calcCentroidalMomentumMatrix)
{
    Vector3d com, com_velocity, ang_momentum;

    randomizeStates();

    MatrixNd A(6, model->qdot_size), G(3, model->qdot_size);
    A.setZero();

    Utils::calcCenterOfMass(*model, q, qdot, com, &com_velocity, &ang_momentum);
    Utils::calcCentroidalMomentumMatrix(*model, q, A);

    SpatialVector m_exp;
    m_exp.setLinearPart(com_velocity * model->mass);
    m_exp.setAngularPart(ang_momentum);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(SpatialVector(A * qdot), m_exp, unit_test_utils::E_MINUS_12));

    A.setZero();

    Utils::calcCenterOfMass(*model_3dof, q, qdot, com, &com_velocity, &ang_momentum);
    Utils::calcCentroidalMomentumMatrix(*model_3dof, q, A);

    m_exp.setLinearPart(com_velocity * model->mass);
    m_exp.setAngularPart(ang_momentum);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(SpatialVector(A * qdot), m_exp, unit_test_utils::E_MINUS_12));

    A.setZero();

    Utils::calcCenterOfMass(*model_emulated, q, qdot, com, &com_velocity, &ang_momentum);
    Utils::calcCentroidalMomentumMatrix(*model_emulated, q, A);

    m_exp.setLinearPart(com_velocity * model->mass);
    m_exp.setAngularPart(ang_momentum);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(SpatialVector(A * qdot), m_exp, unit_test_utils::E_MINUS_12));
}

TEST_F(RdlUtilsTests, calcCentroidalMomentumMatrixDot)
{
    Model model;
    model.gravity.setZero();

    Body b1(1., Vector3d(0., 0., -0.1), Vector3d(1., 1., 1.));
    Joint jx(JointTypeRevoluteX);

    model.addBody(0, SpatialTransform(), jx, b1, "b1");
    model.appendBody(Xtrans(Vector3d(0., 0., -1.)), jx, b1, "b2");

    VectorNd Q(model.q_size);
    VectorNd QDot(model.qdot_size);
    VectorNd QDDot(model.qdot_size);
    VectorNd Tau(model.qdot_size);
    Q.setZero();
    QDot.setZero();
    QDDot.setZero();
    QDot[0] = 0.1;
    QDot[1] = 0.1;
    Tau[0] = 4;
    Tau[1] = 5;
    Tau.setZero();
    MatrixNd A(6, model.qdot_size), ADot(6, model.qdot_size), ADot_num(6, model.qdot_size);
    A.setZero();
    ADot.setZero();

    Vector3d com, com_velocity, ang_momentum;
    Utils::calcCenterOfMass(model, Q, QDot, com, &com_velocity, &ang_momentum);

    SpatialVector m_exp;
    m_exp.setAngularPart(ang_momentum);
    m_exp.setLinearPart(com_velocity * model.mass);

    Utils::calcCentroidalMomentumMatrix(model, Q, A, true);

    double h = 0.00000005;
    Math::VectorNd x_euler = Math::VectorNd::Zero(model.q_size + model.qdot_size);
    x_euler.setZero();
    Math::VectorNd x_rk4 = x_euler;

    unit_test_utils::integrateRk4(model, Q, QDot, x_rk4, Tau, h);
    Q = x_rk4.head(model.q_size);
    QDot = x_rk4.tail(model.qdot_size);

    MatrixNd A2(6, model.qdot_size);
    A2.setZero();
    Utils::calcCentroidalMomentumMatrix(model, Q, A2, true);
    Utils::calcCentroidalMomentumMatrixDot(model, Q, QDot, ADot, true);

    ADot_num = ((1.0 / h) * (A2 - A));

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(ADot_num * QDot, ADot * QDot, 1e-6));
}

TEST_F(FixedBase3DoFPlanar, calcCentroidalMomentumMatrixDot)
{
    randomizeStates();
    double h = 0.00000005;

    RobotDynamics::Math::MatrixNd A1(6, model->qdot_size), A2(6, model->qdot_size), ADot(6, model->qdot_size), ADot_num(6, model->qdot_size);
    A1.setZero();
    A2.setZero();
    ADot.setZero();
    ADot_num.setZero();
    RobotDynamics::Utils::calcCentroidalMomentumMatrix(*model, Q, A1);

    Math::VectorNd x_rk4 = Math::VectorNd::Zero(model->q_size + model->qdot_size);
    unit_test_utils::integrateRk4(*model, Q, QDot, x_rk4, Tau, h);

    // integrate
    Q = x_rk4.head(model->q_size);
    QDot = x_rk4.tail(model->qdot_size);

    RobotDynamics::Utils::calcCentroidalMomentumMatrixDot(*model, Q, QDot, ADot);

    RobotDynamics::Utils::calcCentroidalMomentumMatrix(*model, Q, A2);

    ADot_num = (1.0 / h) * (A2 - A1);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(ADot_num * QDot, ADot * QDot, 1e-4));
}

TEST_F(Human36, calcCentroidalMomentumMatrixDot)
{
    randomizeStates();
    double h = 0.00000005;

    RobotDynamics::Math::MatrixNd A1(6, model_emulated->qdot_size), A2(6, model_emulated->qdot_size), ADot(6, model_emulated->qdot_size),
        ADot_num(6, model_emulated->qdot_size);
    A1.setZero();
    A2.setZero();
    ADot.setZero();
    ADot_num.setZero();
    RobotDynamics::Utils::calcCentroidalMomentumMatrix(*model_emulated, q, A1);

    Math::VectorNd x_rk4 = Math::VectorNd::Zero(model_emulated->q_size + model_emulated->qdot_size);
    unit_test_utils::integrateRk4(*model_emulated, q, qdot, x_rk4, tau, h);
    // integra
    q = x_rk4.head(model_emulated->q_size);
    qdot = x_rk4.tail(model_emulated->qdot_size);

    RobotDynamics::Utils::calcCentroidalMomentumMatrixDot(*model_emulated, q, qdot, ADot);

    RobotDynamics::Utils::calcCentroidalMomentumMatrix(*model_emulated, q, A2);

    ADot_num = (1.0 / h) * (A2 - A1);

    EXPECT_TRUE((ADot_num * qdot).isApprox(ADot * qdot, 1e-5));
}

TEST_F(Human36, calcCentroidalMomentumMatricesWithSuppliedComAndVelocity)
{
    randomizeStates();
    RobotDynamics::Math::MatrixNd A1(6, model_emulated->qdot_size), A2(6, model_emulated->qdot_size), ADot1(6, model_emulated->qdot_size),
        ADot2(6, model_emulated->qdot_size);
    RobotDynamics::updateKinematicsCustom(*model_emulated, &q, &qdot, nullptr);
    Vector3d com, com_velocity;

    RobotDynamics::Utils::calcCenterOfMass(*model_emulated, q, qdot, com, &com_velocity, nullptr, false);
    RobotDynamics::Utils::calcCentroidalMomentumMatrix(*model_emulated, q, A1, com, false);
    RobotDynamics::Utils::calcCentroidalMomentumMatrix(*model_emulated, q, A2);
    EXPECT_TRUE(A1.isApprox(A2, unit_test_utils::E_MINUS_14));

    RobotDynamics::Utils::calcCentroidalMomentumMatrixDot(*model_emulated, q, qdot, ADot1, com, com_velocity, false);
    RobotDynamics::Utils::calcCentroidalMomentumMatrixDot(*model_emulated, q, qdot, ADot2);
    EXPECT_TRUE(ADot1.isApprox(ADot2, unit_test_utils::E_MINUS_14));

    RobotDynamics::Utils::calcCentroidalMomentumMatrixAndMatrixDot(*model_emulated, q, qdot, A1, ADot1, com, com_velocity, false);
    RobotDynamics::Utils::calcCentroidalMomentumMatrixAndMatrixDot(*model_emulated, q, qdot, A2, ADot2);
    EXPECT_TRUE(A1.isApprox(A2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(ADot1.isApprox(ADot2, unit_test_utils::E_MINUS_14));
}

TEST_F(Human36, calcCentroidalMomentumMatrixAndMatrixDot)
{
    randomizeStates();

    RobotDynamics::Math::MatrixNd A1(6, model_emulated->qdot_size), A2(6, model_emulated->qdot_size), ADot1(6, model_emulated->qdot_size),
        ADot2(6, model_emulated->qdot_size);
    A1.setZero();
    A2.setZero();
    ADot1.setZero();
    ADot2.setZero();
    RobotDynamics::updateKinematicsCustom(*model_emulated, &q, &qdot);
    RobotDynamics::Utils::calcCentroidalMomentumMatrix(*model_emulated, q, A1, false);
    RobotDynamics::Utils::calcCentroidalMomentumMatrixDot(*model_emulated, q, qdot, ADot1, false);
    RobotDynamics::Utils::calcCentroidalMomentumMatrixAndMatrixDot(*model_emulated, q, qdot, A2, ADot2, false);

    EXPECT_TRUE(A1.isApprox(A2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(ADot1.isApprox(ADot2, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlUtilsTests, calcCentroidalMomentumMatrixDotSphericalJoint)
{
    Model model;
    Body b(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));
    model.appendBody(SpatialTransform(), Joint(SpatialVector(0., 0., 0., 1., 0., 0.)), b);
    model.appendBody(SpatialTransform(), Joint(SpatialVector(0., 0., 0., 0., 1., 0.)), b);
    model.appendBody(SpatialTransform(), Joint(SpatialVector(0., 0., 0., 0., 0., 1.)), b);
    unsigned int id = model.appendBody(SpatialTransform(), Joint(JointTypeSpherical), b);
    RobotDynamics::Math::Quaternion quat(1., 2., 3., 4.);
    quat.normalize();
    double h = 0.00000005;
    VectorNd q(model.q_size), qdot(model.qdot_size), tau(model.qdot_size);
    q.setZero();
    qdot.setZero();
    tau.setZero();
    RobotDynamics::Math::MatrixNd A1(6, model.qdot_size), A2(6, model.qdot_size), ADot(6, model.qdot_size), ADot_num(6, model.qdot_size);
    model.SetQuaternion(id, quat, q);
    qdot[0] = 1.;
    qdot[1] = -0.3;
    qdot[2] = 0.115;
    qdot[3] = 0.36;
    qdot[4] = -1.1;
    qdot[5] = 2.2;
    A1.setZero();
    A2.setZero();
    ADot.setZero();
    ADot_num.setZero();
    RobotDynamics::Utils::calcCentroidalMomentumMatrix(model, q, A1);

    Math::VectorNd x_rk4 = Math::VectorNd::Zero(model.q_size + model.qdot_size);
    unit_test_utils::integrateEuler(model, q, qdot, x_rk4, tau, h);
    q = x_rk4.head(model.q_size);
    qdot = x_rk4.tail(model.qdot_size);

    RobotDynamics::Utils::calcCentroidalMomentumMatrixDot(model, q, qdot, ADot);

    RobotDynamics::Utils::calcCentroidalMomentumMatrix(model, q, A2);

    ADot_num = (1.0 / h) * (A2 - A1);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(ADot_num * qdot, ADot * qdot, 1e-5));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
