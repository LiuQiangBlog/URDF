// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>

#include "UnitTestUtils.hpp"
#include "rdl_dynamics/Kinematics.hpp"

#include "Fixtures.hpp"
#include "Human36Fixture.hpp"
#include "rdl_dynamics/Dynamics.hpp"

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

struct SphericalJointFixture : testing::Test
{
    SphericalJointFixture()
    {
    }

    void SetUp()
    {
        emulated_model.gravity = SpatialVector(0., 0., 0., 0., 0., -9.81);
        multdof3_model.gravity = SpatialVector(0., 0., 0., 0., 0., -9.81);
        eulerzyx_model.gravity = SpatialVector(0., 0., 0., 0., 0., -9.81);

        body = Body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));

        joint_rot_zyx = Joint(SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.));
        joint_spherical = Joint(JointTypeSpherical);
        joint_eulerzyx = Joint(JointTypeEulerZYX);

        joint_rot_y = Joint(SpatialVector(0., 1., 0., 0., 0., 0.));

        emulated_model.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_y, body);
        emu_body_id = emulated_model.appendBody(Xtrans(Vector3d(1., 0., 0.)), joint_rot_zyx, body);
        emu_child_id = emulated_model.appendBody(Xtrans(Vector3d(1., 0., 0.)), joint_rot_y, body);

        multdof3_model.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_y, body);
        sph_body_id = multdof3_model.appendBody(Xtrans(Vector3d(1., 0., 0.)), joint_spherical, body);
        sph_child_id = multdof3_model.appendBody(Xtrans(Vector3d(1., 0., 0.)), joint_rot_y, body);

        eulerzyx_model.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_y, body);
        eulerzyx_body_id = eulerzyx_model.appendBody(Xtrans(Vector3d(1., 0., 0.)), joint_eulerzyx, body);
        eulerzyx_child_id = eulerzyx_model.appendBody(Xtrans(Vector3d(1., 0., 0.)), joint_rot_y, body);

        emuQ = VectorNd::Zero((size_t)emulated_model.q_size);
        emuQDot = VectorNd::Zero((size_t)emulated_model.qdot_size);
        emuQDDot = VectorNd::Zero((size_t)emulated_model.qdot_size);
        emuTau = VectorNd::Zero((size_t)emulated_model.qdot_size);

        sphQ = VectorNd::Zero((size_t)multdof3_model.q_size);
        sphQDot = VectorNd::Zero((size_t)multdof3_model.qdot_size);
        sphQDDot = VectorNd::Zero((size_t)multdof3_model.qdot_size);
        sphTau = VectorNd::Zero((size_t)multdof3_model.qdot_size);

        eulerzyxQ = VectorNd::Zero((size_t)eulerzyx_model.q_size);
        eulerzyxQDot = VectorNd::Zero((size_t)eulerzyx_model.qdot_size);
        eulerzyxQDDot = VectorNd::Zero((size_t)eulerzyx_model.qdot_size);
        eulerzyxTau = VectorNd::Zero((size_t)eulerzyx_model.qdot_size);
    }

    Joint joint_rot_zyx;
    Joint joint_spherical;
    Joint joint_eulerzyx;
    Joint joint_rot_y;
    Body body;

    unsigned int emu_body_id, emu_child_id, sph_body_id, sph_child_id, eulerzyx_body_id, eulerzyx_child_id;

    Model emulated_model;
    Model multdof3_model;
    Model eulerzyx_model;

    VectorNd emuQ;
    VectorNd emuQDot;
    VectorNd emuQDDot;
    VectorNd emuTau;

    VectorNd sphQ;
    VectorNd sphQDot;
    VectorNd sphQDDot;
    VectorNd sphTau;

    VectorNd eulerzyxQ;
    VectorNd eulerzyxQDot;
    VectorNd eulerzyxQDDot;
    VectorNd eulerzyxTau;
};

void ConvertQAndQDotFromEmulated(const Model& emulated_model, const VectorNd& q_emulated, const VectorNd& qdot_emulated, const Model& multdof3_model,
                                 VectorNd* q_spherical, VectorNd* qdot_spherical)
{
    (void)emulated_model;
    for (unsigned int i = 1; i < multdof3_model.mJoints.size(); i++)
    {
        unsigned int q_index = multdof3_model.mJoints[i].q_index;

        if (multdof3_model.mJoints[i].mJointType == JointTypeSpherical)
        {
            Quaternion quat(Math::intrinsicZYXAnglesToQuaternion(Vector3d(q_emulated[q_index + 0], q_emulated[q_index + 1], q_emulated[q_index + 2])));
            multdof3_model.SetQuaternion(i, quat, (*q_spherical));

            Vector3d omega = angular_velocity_from_zyx_angle_rates(Vector3d(q_emulated[q_index], q_emulated[q_index + 1], q_emulated[q_index + 2]),
                                                                   Vector3d(qdot_emulated[q_index], qdot_emulated[q_index + 1], qdot_emulated[q_index + 2]));

            (*qdot_spherical)[q_index] = omega[0];
            (*qdot_spherical)[q_index + 1] = omega[1];
            (*qdot_spherical)[q_index + 2] = omega[2];
        }
        else
        {
            (*q_spherical)[q_index] = q_emulated[q_index];
            (*qdot_spherical)[q_index] = qdot_emulated[q_index];
        }
    }
}

TEST_F(SphericalJointFixture, euler_rates_and_angular_velocity_jacobians)
{
    Vector3d w(-1.3, 1.9, -2.1);
    Vector3d angles(-0.97, 1.3, 1.9);

    Matrix3d j = angular_velocity_to_xyz_angle_rates_jacobian(angles);
    EXPECT_TRUE((j * w).isApprox(angular_velocity_to_xyz_angle_rates(angles, w), unit_test_utils::E_MINUS_14));

    j = angular_velocity_to_zyx_angle_rates_jacobian(angles);
    EXPECT_TRUE((j * w).isApprox(angular_velocity_to_zyx_angle_rates(angles, w), unit_test_utils::E_MINUS_14));
}

TEST_F(SphericalJointFixture, euler_rates_and_angular_velocity_zyx)
{
    RobotDynamics::Model model;

    RobotDynamics::Joint j(RobotDynamics::JointTypeEulerZYX);
    RobotDynamics::Body b(1., RobotDynamics::Math::Vector3d(1., 1., 1.), RobotDynamics::Math::Vector3d(1., 1., 1.));
    model.appendBody(RobotDynamics::Math::SpatialTransform(), Joint(JointTypeTranslationXYZ), b, "");
    model.appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "");

    RobotDynamics::Math::VectorNd q = RobotDynamics::Math::VectorNd::Zero(model.q_size);
    RobotDynamics::Math::VectorNd qdot = RobotDynamics::Math::VectorNd::Zero(model.qdot_size);

    q[0] = 1.1;
    q[1] = -2.2;
    q[2] = 9.7;
    q[3] = -1.0;
    q[4] = 0.3256;
    q[5] = -0.5918;
    qdot[0] = 1.7;
    qdot[1] = -0.92;
    qdot[2] = 1.2;
    qdot[3] = 0.1;
    qdot[4] = 0.2;
    qdot[5] = 0.3;

    RobotDynamics::updateKinematicsCustom(model, &q, &qdot, nullptr);
    RobotDynamics::Math::Vector3d v = model.v[2].getAngularPart();
    RobotDynamics::Math::Vector3d v2 =
        angular_velocity_from_zyx_angle_rates(RobotDynamics::Math::Vector3d(q[3], q[4], q[5]), RobotDynamics::Math::Vector3d(qdot[3], qdot[4], qdot[5]));
    EXPECT_TRUE(v.isApprox(v2, unit_test_utils::E_MINUS_14));

    RobotDynamics::Math::Vector3d euler_rates = angular_velocity_to_zyx_angle_rates(RobotDynamics::Math::Vector3d(q[3], q[4], q[5]), v2);
    EXPECT_TRUE(euler_rates.isApprox(qdot.tail(3), unit_test_utils::E_MINUS_14));
}

TEST_F(SphericalJointFixture, euler_rates_and_angular_velocity_xyz)
{
    RobotDynamics::Model model;

    RobotDynamics::Joint j(RobotDynamics::JointTypeEulerXYZ);
    RobotDynamics::Body b(1., RobotDynamics::Math::Vector3d(1., 1., 1.), RobotDynamics::Math::Vector3d(1., 1., 1.));
    model.appendBody(RobotDynamics::Math::SpatialTransform(), Joint(JointTypeTranslationXYZ), b, "");
    model.appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "");

    RobotDynamics::Math::VectorNd q = RobotDynamics::Math::VectorNd::Zero(model.q_size);
    RobotDynamics::Math::VectorNd qdot = RobotDynamics::Math::VectorNd::Zero(model.qdot_size);

    q[0] = 1.1;
    q[1] = -2.2;
    q[2] = 9.7;
    q[3] = -1.0;
    q[4] = 0.3256;
    q[5] = -0.5918;
    qdot[0] = 1.7;
    qdot[1] = -0.92;
    qdot[2] = 1.2;
    qdot[3] = 0.1;
    qdot[4] = 0.2;
    qdot[5] = 0.3;

    RobotDynamics::updateKinematicsCustom(model, &q, &qdot, nullptr);
    RobotDynamics::Math::Vector3d v = model.v[2].getAngularPart();
    RobotDynamics::Math::Vector3d v2 =
        angular_velocity_from_xyz_angle_rates(RobotDynamics::Math::Vector3d(q[3], q[4], q[5]), RobotDynamics::Math::Vector3d(qdot[3], qdot[4], qdot[5]));
    EXPECT_TRUE(v.isApprox(v2, unit_test_utils::E_MINUS_14));

    RobotDynamics::Math::Vector3d euler_rates = angular_velocity_to_xyz_angle_rates(RobotDynamics::Math::Vector3d(q[3], q[4], q[5]), v2);
    EXPECT_TRUE(euler_rates.isApprox(qdot.tail(3), unit_test_utils::E_MINUS_14));
}

TEST_F(SphericalJointFixture, TestQuaternionIntegration)
{
    double timestep = 0.001;

    Vector3d zyx_angles_t0(0.1, 0.2, 0.3);
    Vector3d zyx_rates(3., 5., 2.);
    Vector3d zyx_angles_t1 = zyx_angles_t0 + timestep * zyx_rates;
    Quaternion q_zyx_t1 = Math::intrinsicZYXAnglesToQuaternion(zyx_angles_t1);

    Quaternion q_t0 = Math::intrinsicZYXAnglesToQuaternion(zyx_angles_t0);
    Vector3d w_base = global_angular_velocity_from_rates(zyx_angles_t0, zyx_rates);
    Quaternion q_t1 = q_t0.timeStep(w_base, timestep);

    // Note: we test with a rather crude precision. My guess for the error is
    // that we compare two different things:
    //   A) integration under the assumption that the euler rates are
    //   constant
    //   B) integration under the assumption that the angular velocity is
    //   constant
    // However I am not entirely sure about this...

    EXPECT_NEAR(q_zyx_t1.x(), q_t1.x(), 1.0e-5);
    EXPECT_NEAR(q_zyx_t1.y(), q_t1.y(), 1.0e-5);
    EXPECT_NEAR(q_zyx_t1.z(), q_t1.z(), 1.0e-5);
    EXPECT_NEAR(q_zyx_t1.w(), q_t1.w(), 1.0e-5);
}

TEST_F(SphericalJointFixture, TestQIndices)
{
    EXPECT_EQ(0u, multdof3_model.mJoints[1].q_index);
    EXPECT_EQ(1u, multdof3_model.mJoints[2].q_index);
    EXPECT_EQ(4u, multdof3_model.mJoints[3].q_index);

    EXPECT_EQ(5u, emulated_model.q_size);
    EXPECT_EQ(5u, emulated_model.qdot_size);

    EXPECT_EQ(6u, multdof3_model.q_size);
    EXPECT_EQ(5u, multdof3_model.qdot_size);
    EXPECT_EQ(5u, multdof3_model.multdof3_w_index[2]);
}

TEST_F(SphericalJointFixture, TestGetQuaternion)
{
    multdof3_model.appendBody(Xtrans(Vector3d(1., 0., 0.)), joint_spherical, body);

    sphQ = VectorNd::Zero((size_t)multdof3_model.q_size);
    sphQDot = VectorNd::Zero((size_t)multdof3_model.qdot_size);
    sphQDDot = VectorNd::Zero((size_t)multdof3_model.qdot_size);
    sphTau = VectorNd::Zero((size_t)multdof3_model.qdot_size);

    EXPECT_EQ(10u, multdof3_model.q_size);
    EXPECT_EQ(8u, multdof3_model.qdot_size);

    EXPECT_EQ(0u, multdof3_model.mJoints[1].q_index);
    EXPECT_EQ(1u, multdof3_model.mJoints[2].q_index);
    EXPECT_EQ(4u, multdof3_model.mJoints[3].q_index);
    EXPECT_EQ(5u, multdof3_model.mJoints[4].q_index);

    EXPECT_EQ(8u, multdof3_model.multdof3_w_index[2]);
    EXPECT_EQ(9u, multdof3_model.multdof3_w_index[4]);

    sphQ[0] = 100.;
    sphQ[1] = 0.;
    sphQ[2] = 1.;
    sphQ[3] = 2.;
    sphQ[4] = 100.;
    sphQ[5] = -6.;
    sphQ[6] = -7.;
    sphQ[7] = -8;
    sphQ[8] = 4.;
    sphQ[9] = -9.;

    Quaternion reference_1(0., 1., 2., 4.);
    Quaternion quat_1 = multdof3_model.GetQuaternion(2, sphQ);

    EXPECT_EQ(reference_1.x(), quat_1.x());
    EXPECT_EQ(reference_1.y(), quat_1.y());
    EXPECT_EQ(reference_1.z(), quat_1.z());
    EXPECT_EQ(reference_1.w(), quat_1.w());

    Quaternion reference_3(-6., -7., -8., -9.);
    Quaternion quat_3 = multdof3_model.GetQuaternion(4, sphQ);

    EXPECT_EQ(reference_3.x(), quat_3.x());
    EXPECT_EQ(reference_3.y(), quat_3.y());
    EXPECT_EQ(reference_3.z(), quat_3.z());
    EXPECT_EQ(reference_3.w(), quat_3.w());
}

TEST_F(SphericalJointFixture, TestSetQuaternion)
{
    multdof3_model.appendBody(Xtrans(Vector3d(1., 0., 0.)), joint_spherical, body);

    sphQ = VectorNd::Zero((size_t)multdof3_model.q_size);
    sphQDot = VectorNd::Zero((size_t)multdof3_model.qdot_size);
    sphQDDot = VectorNd::Zero((size_t)multdof3_model.qdot_size);
    sphTau = VectorNd::Zero((size_t)multdof3_model.qdot_size);

    Quaternion reference_1(0., 1., 2., 3.);
    multdof3_model.SetQuaternion(2, reference_1, sphQ);
    Quaternion test = multdof3_model.GetQuaternion(2, sphQ);

    EXPECT_EQ(reference_1.x(), test.x());
    EXPECT_EQ(reference_1.y(), test.y());
    EXPECT_EQ(reference_1.z(), test.z());
    EXPECT_EQ(reference_1.w(), test.w());

    Quaternion reference_2(11., 22., 33., 44.);
    multdof3_model.SetQuaternion(4, reference_2, sphQ);
    test = multdof3_model.GetQuaternion(4, sphQ);

    EXPECT_EQ(reference_2.x(), test.x());
    EXPECT_EQ(reference_2.y(), test.y());
    EXPECT_EQ(reference_2.z(), test.z());
    EXPECT_EQ(reference_2.w(), test.w());
}

TEST_F(SphericalJointFixture, TestOrientation)
{
    emuQ[0] = 1.1;
    emuQ[1] = 1.1;
    emuQ[2] = 1.1;
    emuQ[3] = 1.1;

    for (unsigned int i = 0; i < emuQ.size(); i++)
    {
        sphQ[i] = emuQ[i];
    }

    Quaternion quat = Quaternion(Vector3d(0., 0., 1.), emuQ[0]) * Quaternion(Vector3d(0., 1., 0.), emuQ[1]) * Quaternion(Vector3d(1., 0., 0.), emuQ[2]);
    multdof3_model.SetQuaternion(2, quat, sphQ);

    updateKinematicsCustom(emulated_model, &emuQ, nullptr, nullptr);
    updateKinematicsCustom(multdof3_model, &sphQ, nullptr, nullptr);

    EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(emulated_model.bodyFrames[emu_child_id]->getInverseTransformToRoot().E,
                                                           multdof3_model.bodyFrames[sph_child_id]->getInverseTransformToRoot().E, unit_test_utils::E_MINUS_12));

    EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(emulated_model.bodyFrames[emu_child_id]->getTransformToRoot().E,
                                                           multdof3_model.bodyFrames[sph_child_id]->getTransformToRoot().E, unit_test_utils::E_MINUS_12));
}

TEST_F(SphericalJointFixture, Kinematics)
{
    emuQ[0] = 1.1;
    emuQ[1] = 1.;
    emuQ[2] = -1.2;
    emuQ[3] = 1.;
    emuQ[4] = 1.3;

    emuQDot[0] = -1.;
    emuQDot[1] = 1.1;
    emuQDot[2] = -1.3;
    emuQDot[3] = 1.;
    emuQDot[4] = -1.;

    ConvertQAndQDotFromEmulated(emulated_model, emuQ, emuQDot, multdof3_model, &sphQ, &sphQDot);
    eulerzyxQ = emuQ;
    eulerzyxQDot = emuQDot;

    updateKinematicsCustom(emulated_model, &emuQ, &emuQDot, &emuQDDot);
    updateKinematicsCustom(multdof3_model, &sphQ, &sphQDot, &sphQDDot);
    updateKinematicsCustom(eulerzyx_model, &eulerzyxQ, &eulerzyxQDot, &eulerzyxQDDot);

    EXPECT_TRUE(emulated_model.v[emu_body_id].isApprox(multdof3_model.v[sph_body_id], unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(eulerzyx_model.v[eulerzyx_body_id].isApprox(multdof3_model.v[sph_body_id], unit_test_utils::E_MINUS_12));

    EXPECT_TRUE(emulated_model.v[emu_child_id].isApprox(multdof3_model.v[sph_child_id], unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(eulerzyx_model.v[eulerzyx_child_id].isApprox(multdof3_model.v[sph_child_id], unit_test_utils::E_MINUS_12));
}

TEST_F(SphericalJointFixture, angular_acceleration_from_zyx_angle_rates)
{
    Model model;

    Joint j(RobotDynamics::JointTypeEulerZYX);
    Body b(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));
    model.appendBody(SpatialTransform(), j, b, "");

    VectorNd q = VectorNd::Zero(model.q_size);
    VectorNd qdot = VectorNd::Zero(model.qdot_size);
    VectorNd qddot = VectorNd::Zero(model.qdot_size);

    q[0] = -1.0;
    q[1] = 0.3256;
    q[2] = -0.5918;
    qdot[0] = 0.1;
    qdot[1] = 0.2;
    qdot[2] = 0.3;
    qddot[0] = 13.1;
    qddot[1] = -120.2;
    qddot[2] = 3330.3;
    Vector3d zyx_angles = Vector3d(q[0], q[1], q[2]);
    Vector3d zyx_angle_rates = Vector3d(qdot[0], qdot[1], qdot[2]);

    Vector3d w = angular_velocity_from_zyx_angle_rates(zyx_angles, zyx_angle_rates);

    updateKinematicsCustom(model, &q, &qdot, &qddot);
    EXPECT_TRUE(w.isApprox(model.v[1].getAngularPart(), unit_test_utils::E_MINUS_12));

    Vector3d a = model.a[1].getAngularPart();

    Vector3d a2 = angular_acceleration_from_zyx_angle_rates(zyx_angles, zyx_angle_rates, Vector3d(qddot[0], qddot[1], qddot[2]));
    EXPECT_TRUE(a.isApprox(a2, unit_test_utils::E_MINUS_10));

    Vector3d zyx_angle_rates_check = angular_velocity_to_zyx_angle_rates(zyx_angles, w);
    EXPECT_TRUE(zyx_angle_rates.isApprox(zyx_angle_rates_check, unit_test_utils::E_MINUS_14));
}

TEST_F(SphericalJointFixture, angular_velocity_and_acceleration_from_xyz_angle_rates)
{
    Model model;

    Joint j(RobotDynamics::JointTypeEulerXYZ);
    Body b(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));
    model.appendBody(SpatialTransform(), j, b, "");

    VectorNd q = VectorNd::Zero(model.q_size);
    VectorNd qdot = VectorNd::Zero(model.qdot_size);
    VectorNd qddot = VectorNd::Zero(model.qdot_size);

    q[0] = -1.0;
    q[1] = 0.3256;
    q[2] = -0.5918;
    qdot[0] = 0.1;
    qdot[1] = 0.2;
    qdot[2] = 0.3;
    qddot[0] = 13.1;
    qddot[1] = -120.2;
    qddot[2] = 3330.3;
    Vector3d xyz_angles = Vector3d(q[0], q[1], q[2]);
    Vector3d xyz_angle_rates = Vector3d(qdot[0], qdot[1], qdot[2]);

    Vector3d w = angular_velocity_from_xyz_angle_rates(xyz_angles, xyz_angle_rates);

    updateKinematicsCustom(model, &q, &qdot, &qddot);
    EXPECT_TRUE(w.isApprox(model.v[1].getAngularPart(), unit_test_utils::E_MINUS_12));

    Vector3d a = model.a[1].getAngularPart();

    Vector3d a2 = angular_acceleration_from_xyz_angle_rates(xyz_angles, xyz_angle_rates, Vector3d(qddot[0], qddot[1], qddot[2]));
    EXPECT_TRUE(a.isApprox(a2, unit_test_utils::E_MINUS_10));

    Vector3d xyz_angle_rates_check = angular_velocity_to_xyz_angle_rates(xyz_angles, w);
    EXPECT_TRUE(xyz_angle_rates.isApprox(xyz_angle_rates_check, unit_test_utils::E_MINUS_14));
}

TEST_F(SphericalJointFixture, TestSpatialVelocities)
{
    emuQ[0] = 1.;
    emuQ[1] = 2.;
    emuQ[2] = 3.;
    emuQ[3] = 4.;

    emuQDot[0] = 4.;
    emuQDot[1] = 2.;
    emuQDot[2] = 3.;
    emuQDot[3] = 6.;

    ConvertQAndQDotFromEmulated(emulated_model, emuQ, emuQDot, multdof3_model, &sphQ, &sphQDot);

    updateKinematicsCustom(emulated_model, &emuQ, &emuQDot, NULL);
    updateKinematicsCustom(multdof3_model, &sphQ, &sphQDot, NULL);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(emulated_model.v[emu_child_id], multdof3_model.v[sph_child_id], unit_test_utils::E_MINUS_12));
}

TEST_F(SphericalJointFixture, TestForwardDynamicsQAndQDot)
{
    emuQ[0] = 1.1;
    emuQ[1] = 1.2;
    emuQ[2] = 1.3;
    emuQ[3] = 1.4;

    emuQDot[0] = 2.2;
    emuQDot[1] = 2.3;
    emuQDot[2] = 2.4;
    emuQDot[3] = 2.5;

    ConvertQAndQDotFromEmulated(emulated_model, emuQ, emuQDot, multdof3_model, &sphQ, &sphQDot);

    forwardDynamics(emulated_model, emuQ, emuQDot, emuTau, emuQDDot);
    forwardDynamics(multdof3_model, sphQ, sphQDot, sphTau, sphQDDot);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(emulated_model.a[emu_child_id], multdof3_model.a[sph_child_id], unit_test_utils::E_MINUS_12));
}

TEST_F(SphericalJointFixture, TestDynamicsConsistencyRNEA_ABA)
{
    emuQ[0] = 1.1;
    emuQ[1] = 1.2;
    emuQ[2] = 1.3;
    emuQ[3] = 1.4;
    emuQ[4] = 1.5;

    emuQDot[0] = 1.;
    emuQDot[1] = 2.;
    emuQDot[2] = 3.;
    emuQDot[3] = 4.;
    emuQDot[4] = 5.;

    sphTau[0] = 5.;
    sphTau[1] = 4.;
    sphTau[2] = 7.;
    sphTau[3] = 3.;
    sphTau[4] = 2.;

    ConvertQAndQDotFromEmulated(emulated_model, emuQ, emuQDot, multdof3_model, &sphQ, &sphQDot);

    forwardDynamics(multdof3_model, sphQ, sphQDot, sphTau, sphQDDot);

    VectorNd tau_id(VectorNd::Zero(multdof3_model.qdot_size));
    inverseDynamics(multdof3_model, sphQ, sphQDot, sphQDDot, tau_id);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(sphTau, tau_id, unit_test_utils::E_MINUS_12));
}

TEST_F(SphericalJointFixture, TestCRBA)
{
    emuQ[0] = 1.1;
    emuQ[1] = 1.2;
    emuQ[2] = 1.3;
    emuQ[3] = 1.4;
    emuQ[4] = 1.5;

    emuQDot[0] = 1.;
    emuQDot[1] = 2.;
    emuQDot[2] = 3.;
    emuQDot[3] = 4.;
    emuQDot[4] = 5.;

    sphTau[0] = 5.;
    sphTau[1] = 4.;
    sphTau[2] = 7.;
    sphTau[3] = 3.;
    sphTau[4] = 2.;

    ConvertQAndQDotFromEmulated(emulated_model, emuQ, emuQDot, multdof3_model, &sphQ, &sphQDot);

    MatrixNd H_crba(MatrixNd::Zero(multdof3_model.qdot_size, multdof3_model.qdot_size));

    updateKinematicsCustom(multdof3_model, &sphQ, NULL, NULL);
    compositeRigidBodyAlgorithm(multdof3_model, sphQ, H_crba, false);

    MatrixNd H_id(MatrixNd::Zero(multdof3_model.qdot_size, multdof3_model.qdot_size));
    VectorNd H_col = VectorNd::Zero(multdof3_model.qdot_size);
    VectorNd QDDot_zero = VectorNd::Zero(multdof3_model.qdot_size);

    for (unsigned int i = 0; i < multdof3_model.qdot_size; i++)
    {
        // compute each column
        VectorNd delta_a = VectorNd::Zero(multdof3_model.qdot_size);
        delta_a[i] = 1.;

        // compute ID (model, q, qdot, delta_a)
        VectorNd id_delta = VectorNd::Zero(multdof3_model.qdot_size);
        inverseDynamics(multdof3_model, sphQ, sphQDot, delta_a, id_delta);

        // compute ID (model, q, qdot, zero)
        VectorNd id_zero = VectorNd::Zero(multdof3_model.qdot_size);
        inverseDynamics(multdof3_model, sphQ, sphQDot, QDDot_zero, id_zero);

        H_col = id_delta - id_zero;
        H_id.block(0, i, multdof3_model.qdot_size, 1) = H_col;
    }

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(H_id, H_crba, unit_test_utils::E_MINUS_12));
}

TEST_F(SphericalJointFixture, TestForwardDynamicsLagrangianVsABA)
{
    emuQ[0] = 1.1;
    emuQ[1] = 1.2;
    emuQ[2] = 1.3;
    emuQ[3] = 1.4;
    emuQ[4] = 1.5;

    emuQDot[0] = 1.;
    emuQDot[1] = 2.;
    emuQDot[2] = 3.;
    emuQDot[3] = 4.;
    emuQDot[4] = 5.;

    sphTau[0] = 5.;
    sphTau[1] = 4.;
    sphTau[2] = 7.;
    sphTau[3] = 3.;
    sphTau[4] = 2.;

    ConvertQAndQDotFromEmulated(emulated_model, emuQ, emuQDot, multdof3_model, &sphQ, &sphQDot);

    VectorNd QDDot_aba = VectorNd::Zero(multdof3_model.qdot_size);
    VectorNd QDDot_lag = VectorNd::Zero(multdof3_model.qdot_size);

    RobotDynamics::Math::MatrixNd H = RobotDynamics::Math::MatrixNd::Zero(multdof3_model.qdot_size, multdof3_model.qdot_size);
    RobotDynamics::Math::VectorNd C = RobotDynamics::Math::VectorNd::Zero(multdof3_model.qdot_size);
    forwardDynamicsLagrangian(multdof3_model, sphQ, sphQDot, sphTau, QDDot_lag, H, C);
    forwardDynamics(multdof3_model, sphQ, sphQDot, sphTau, QDDot_aba);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_lag, QDDot_aba, unit_test_utils::E_MINUS_12));
}

TEST_F(SphericalJointFixture, TestContactsLagrangian)
{
    ConstraintSet constraint_set_emu;

    constraint_set_emu.addConstraint(emu_child_id, Vector3d(0., 0., -1.), Vector3d(1., 0., 0.));
    constraint_set_emu.addConstraint(emu_child_id, Vector3d(0., 0., -1.), Vector3d(0., 1., 0.));
    constraint_set_emu.addConstraint(emu_child_id, Vector3d(0., 0., -1.), Vector3d(0., 0., 1.));

    constraint_set_emu.bind(emulated_model);

    ConstraintSet constraint_set_sph;

    constraint_set_sph.addConstraint(sph_child_id, Vector3d(0., 0., -1.), Vector3d(1., 0., 0.));
    constraint_set_sph.addConstraint(sph_child_id, Vector3d(0., 0., -1.), Vector3d(0., 1., 0.));
    constraint_set_sph.addConstraint(sph_child_id, Vector3d(0., 0., -1.), Vector3d(0., 0., 1.));

    constraint_set_sph.bind(multdof3_model);

    forwardDynamicsContactsDirect(emulated_model, emuQ, emuQDot, emuTau, constraint_set_emu, emuQDDot);
    VectorNd emu_force_lagrangian = constraint_set_emu.force;
    forwardDynamicsContactsDirect(multdof3_model, sphQ, sphQDot, sphTau, constraint_set_sph, sphQDDot);
    VectorNd sph_force_lagrangian = constraint_set_sph.force;

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(emu_force_lagrangian, sph_force_lagrangian, unit_test_utils::E_MINUS_12));
}

TEST_F(SphericalJointFixture, TestContacts)
{
    ConstraintSet constraint_set_emu;

    constraint_set_emu.addConstraint(emu_child_id, Vector3d(0., 0., -1.), Vector3d(1., 0., 0.));
    constraint_set_emu.addConstraint(emu_child_id, Vector3d(0., 0., -1.), Vector3d(0., 1., 0.));
    constraint_set_emu.addConstraint(emu_child_id, Vector3d(0., 0., -1.), Vector3d(0., 0., 1.));

    constraint_set_emu.bind(emulated_model);

    ConstraintSet constraint_set_sph;

    constraint_set_sph.addConstraint(sph_child_id, Vector3d(0., 0., -1.), Vector3d(1., 0., 0.));
    constraint_set_sph.addConstraint(sph_child_id, Vector3d(0., 0., -1.), Vector3d(0., 1., 0.));
    constraint_set_sph.addConstraint(sph_child_id, Vector3d(0., 0., -1.), Vector3d(0., 0., 1.));

    constraint_set_sph.bind(multdof3_model);

    forwardDynamicsContactsKokkevis(emulated_model, emuQ, emuQDot, emuTau, constraint_set_emu, emuQDDot);
    VectorNd emu_force_kokkevis = constraint_set_emu.force;
    forwardDynamicsContactsKokkevis(multdof3_model, sphQ, sphQDot, sphTau, constraint_set_sph, sphQDDot);
    VectorNd sph_force_kokkevis = constraint_set_sph.force;

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(emu_force_kokkevis, sph_force_kokkevis, unit_test_utils::E_MINUS_12));
}

TEST_F(SphericalJointFixture, TestEulerZYXvsEmulatedLagrangian)
{
    emuQ[0] = 1.1;
    emuQ[1] = 1.2;
    emuQ[2] = 1.3;
    emuQ[3] = 1.4;
    emuQ[4] = 1.5;

    emuQDot[0] = 1.;
    emuQDot[1] = 2.;
    emuQDot[2] = 3.;
    emuQDot[3] = 4.;
    emuQDot[4] = 5.;

    emuTau[0] = 5.;
    emuTau[1] = 4.;
    emuTau[2] = 7.;
    emuTau[3] = 3.;
    emuTau[4] = 2.;

    VectorNd QDDot_emu = VectorNd::Zero(emulated_model.qdot_size);
    VectorNd QDDot_eulerzyx = VectorNd::Zero(eulerzyx_model.qdot_size);

    RobotDynamics::Math::MatrixNd H = RobotDynamics::Math::MatrixNd::Zero(emulated_model.qdot_size, emulated_model.qdot_size);
    RobotDynamics::Math::VectorNd C = RobotDynamics::Math::VectorNd::Zero(emulated_model.qdot_size);
    forwardDynamicsLagrangian(emulated_model, emuQ, emuQDot, emuTau, QDDot_emu, H, C);

    H = RobotDynamics::Math::MatrixNd::Zero(eulerzyx_model.qdot_size, eulerzyx_model.qdot_size);
    C = RobotDynamics::Math::VectorNd::Zero(eulerzyx_model.qdot_size);
    forwardDynamicsLagrangian(eulerzyx_model, emuQ, emuQDot, emuTau, QDDot_eulerzyx, H, C);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_emu, QDDot_eulerzyx, unit_test_utils::E_MINUS_12));
}

TEST_F(SphericalJointFixture, TestEulerZYXvsEmulatedArticulatedBodyAlgorithm)
{
    emuQ[0] = 1.1;
    emuQ[1] = 1.2;
    emuQ[2] = 1.3;
    emuQ[3] = 1.4;
    emuQ[4] = 1.5;

    emuQDot[0] = 1.;
    emuQDot[1] = 2.;
    emuQDot[2] = 3.;
    emuQDot[3] = 4.;
    emuQDot[4] = 5.;

    emuTau[0] = 5.;
    emuTau[1] = 4.;
    emuTau[2] = 7.;
    emuTau[3] = 3.;
    emuTau[4] = 2.;

    VectorNd QDDot_emu = VectorNd::Zero(emulated_model.qdot_size);
    VectorNd QDDot_eulerzyx = VectorNd::Zero(eulerzyx_model.qdot_size);

    forwardDynamics(emulated_model, emuQ, emuQDot, emuTau, QDDot_emu);
    forwardDynamics(eulerzyx_model, emuQ, emuQDot, emuTau, QDDot_eulerzyx);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_emu, QDDot_eulerzyx, unit_test_utils::E_MINUS_12));
}

TEST_F(SphericalJointFixture, TestEulerZYXvsEmulatedContacts)
{
    emuQ[0] = 1.1;
    emuQ[1] = 1.2;
    emuQ[2] = 1.3;
    emuQ[3] = 1.4;
    emuQ[4] = 1.5;

    emuQDot[0] = 1.;
    emuQDot[1] = 2.;
    emuQDot[2] = 3.;
    emuQDot[3] = 4.;
    emuQDot[4] = 5.;

    emuTau[0] = 5.;
    emuTau[1] = 4.;
    emuTau[2] = 7.;
    emuTau[3] = 3.;
    emuTau[4] = 2.;

    VectorNd QDDot_emu = VectorNd::Zero(emulated_model.qdot_size);
    VectorNd QDDot_eulerzyx = VectorNd::Zero(eulerzyx_model.qdot_size);

    ConstraintSet CS_euler;
    CS_euler.addConstraint(eulerzyx_child_id, Vector3d(1., 1., 1.), Vector3d(1., 0., 0.));
    CS_euler.addConstraint(eulerzyx_child_id, Vector3d(0., 0., 0.), Vector3d(0., 1., 0.));
    CS_euler.addConstraint(eulerzyx_child_id, Vector3d(0., 0., 0.), Vector3d(0., 0., 1.));
    CS_euler.bind(eulerzyx_model);

    ConstraintSet CS_emulated;
    CS_emulated.addConstraint(emu_child_id, Vector3d(1., 1., 1.), Vector3d(1., 0., 0.));
    CS_emulated.addConstraint(emu_child_id, Vector3d(0., 0., 0.), Vector3d(0., 1., 0.));
    CS_emulated.addConstraint(emu_child_id, Vector3d(0., 0., 0.), Vector3d(0., 0., 1.));
    CS_emulated.bind(emulated_model);

    forwardDynamicsContactsDirect(emulated_model, emuQ, emuQDot, emuTau, CS_emulated, QDDot_emu);
    forwardDynamicsContactsDirect(eulerzyx_model, emuQ, emuQDot, emuTau, CS_euler, QDDot_eulerzyx);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_emu, QDDot_eulerzyx, unit_test_utils::E_MINUS_12));

    forwardDynamicsContactsKokkevis(emulated_model, emuQ, emuQDot, emuTau, CS_emulated, QDDot_emu);
    forwardDynamicsContactsKokkevis(eulerzyx_model, emuQ, emuQDot, emuTau, CS_euler, QDDot_eulerzyx);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_emu, QDDot_eulerzyx, unit_test_utils::E_MINUS_12 * QDDot_emu.norm()));

    forwardDynamicsContactsKokkevis(emulated_model, emuQ, emuQDot, emuTau, CS_emulated, QDDot_emu);
    forwardDynamicsContactsDirect(eulerzyx_model, emuQ, emuQDot, emuTau, CS_euler, QDDot_eulerzyx);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_emu, QDDot_eulerzyx, unit_test_utils::E_MINUS_12 * QDDot_emu.norm()));
}

TEST_F(SphericalJointFixture, TestEulerZYXvsEmulatedCRBA)
{
    emuQ[0] = 1.1;
    emuQ[1] = 1.2;
    emuQ[2] = 1.3;
    emuQ[3] = 1.4;
    emuQ[4] = 1.5;

    MatrixNd H_emulated(MatrixNd::Zero(emulated_model.q_size, emulated_model.q_size));
    MatrixNd H_eulerzyx(MatrixNd::Zero(eulerzyx_model.q_size, eulerzyx_model.q_size));

    compositeRigidBodyAlgorithm(emulated_model, emuQ, H_emulated);
    compositeRigidBodyAlgorithm(eulerzyx_model, emuQ, H_eulerzyx);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(H_emulated, H_eulerzyx, unit_test_utils::E_MINUS_12));
}

TEST_F(SphericalJointFixture, TestJointTypeTranslationZYX)
{
    Model model_emulated;
    Model model_3dof;

    Body body(1., Vector3d(1., 2., 1.), Matrix3d(1., 0., 0, 0., 1., 0., 0., 0., 1.));
    Joint joint_emulated(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.));
    Joint joint_3dof(JointTypeTranslationXYZ);

    model_emulated.appendBody(SpatialTransform(), joint_emulated, body);
    model_3dof.appendBody(SpatialTransform(), joint_3dof, body);

    VectorNd q(VectorNd::Zero(model_emulated.q_size));
    VectorNd qdot(VectorNd::Zero(model_emulated.qdot_size));
    VectorNd qddot_emulated(VectorNd::Zero(model_emulated.qdot_size));
    VectorNd qddot_3dof(VectorNd::Zero(model_emulated.qdot_size));
    VectorNd tau(VectorNd::Zero(model_emulated.qdot_size));

    for (int i = 0; i < q.size(); i++)
    {
        q[i] = 1.1 * (static_cast<double>(i + 1));
        qdot[i] = 0.1 * (static_cast<double>(i + 1));
        qddot_3dof[i] = 0.21 * (static_cast<double>(i + 1));
        tau[i] = 2.1 * (static_cast<double>(i + 1));
    }

    qddot_emulated = qddot_3dof;
    VectorNd id_emulated(tau);
    VectorNd id_3dof(tau);
    inverseDynamics(model_emulated, q, qdot, qddot_emulated, id_emulated);
    inverseDynamics(model_3dof, q, qdot, qddot_emulated, id_3dof);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(id_emulated, id_3dof, unit_test_utils::E_MINUS_12 * id_emulated.norm()));

    RobotDynamics::Math::MatrixNd H = RobotDynamics::Math::MatrixNd::Zero(model_emulated.qdot_size, model_emulated.qdot_size);
    RobotDynamics::Math::VectorNd C = RobotDynamics::Math::VectorNd::Zero(model_emulated.qdot_size);
    forwardDynamicsLagrangian(model_emulated, q, qdot, tau, qddot_emulated, H, C);

    H = RobotDynamics::Math::MatrixNd::Zero(model_3dof.qdot_size, model_3dof.qdot_size);
    C = RobotDynamics::Math::VectorNd::Zero(model_3dof.qdot_size);
    forwardDynamicsLagrangian(model_3dof, q, qdot, tau, qddot_3dof, H, C);

    EXPECT_TRUE(unit_test_utils::checkVector3dEq(qddot_emulated, qddot_3dof));

    MatrixNd H_emulated(MatrixNd::Zero(q.size(), q.size()));
    MatrixNd H_3dof(MatrixNd::Zero(q.size(), q.size()));

    compositeRigidBodyAlgorithm(model_emulated, q, H_emulated);
    compositeRigidBodyAlgorithm(model_3dof, q, H_3dof);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(H_emulated, H_3dof, unit_test_utils::E_MINUS_12));
}

TEST_F(SphericalJointFixture, TestJointTypeEulerXYZ)
{
    Model model_emulated;
    Model model_3dof;

    Body body(1., Vector3d(1., 2., 1.), Matrix3d(1., 0., 0, 0., 1., 0., 0., 0., 1.));
    Joint joint_emulated(SpatialVector(1., 0., 0., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(0., 0., 1., 0., 0., 0.));
    Joint joint_3dof(JointTypeEulerXYZ);

    model_emulated.appendBody(SpatialTransform(), joint_emulated, body);
    model_3dof.appendBody(SpatialTransform(), joint_3dof, body);

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

    updateKinematicsCustom(model_emulated, &q, &qdot, &qddot_emulated);
    updateKinematicsCustom(model_3dof, &q, &qdot, &qddot_emulated);

    EXPECT_TRUE(unit_test_utils::checkMatrix3dEq(model_emulated.bodyFrames[3]->getTransformToRoot().E, model_3dof.bodyFrames[1]->getTransformToRoot().E));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEq(model_emulated.v[3], model_3dof.v[1]));

    RobotDynamics::Math::MatrixNd H = RobotDynamics::Math::MatrixNd::Zero(model_emulated.qdot_size, model_emulated.qdot_size);
    RobotDynamics::Math::VectorNd C = RobotDynamics::Math::VectorNd::Zero(model_emulated.qdot_size);
    forwardDynamicsLagrangian(model_emulated, q, qdot, tau, qddot_emulated, H, C);

    H = RobotDynamics::Math::MatrixNd::Zero(model_3dof.qdot_size, model_3dof.qdot_size);
    C = RobotDynamics::Math::VectorNd::Zero(model_3dof.qdot_size);
    forwardDynamicsLagrangian(model_3dof, q, qdot, tau, qddot_3dof, H, C);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_emulated, qddot_3dof, unit_test_utils::E_MINUS_12));

    MatrixNd H_emulated(MatrixNd::Zero(q.size(), q.size()));
    MatrixNd H_3dof(MatrixNd::Zero(q.size(), q.size()));

    compositeRigidBodyAlgorithm(model_emulated, q, H_emulated);
    compositeRigidBodyAlgorithm(model_3dof, q, H_3dof);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(H_emulated, H_3dof, unit_test_utils::E_MINUS_12));
}

TEST_F(SphericalJointFixture, TestJointTypeEulerYXZ)
{
    Model model_emulated;
    Model model_3dof;

    Body body(1., Vector3d(1., 2., 1.), Matrix3d(1., 0., 0, 0., 1., 0., 0., 0., 1.));
    Joint joint_emulated(SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.), SpatialVector(0., 0., 1., 0., 0., 0.));
    Joint joint_3dof(JointTypeEulerYXZ);

    model_emulated.appendBody(SpatialTransform(), joint_emulated, body);
    model_3dof.appendBody(SpatialTransform(), joint_3dof, body);

    VectorNd q(VectorNd::Zero(model_emulated.q_size));
    VectorNd qdot(VectorNd::Zero(model_emulated.qdot_size));
    VectorNd qddot_emulated(VectorNd::Zero(model_emulated.qdot_size));
    VectorNd qddot_3dof(VectorNd::Zero(model_emulated.qdot_size));
    VectorNd tau(VectorNd::Zero(model_emulated.qdot_size));

    for (int i = 0; i < q.size(); i++)
    {
        q[i] = 0.4 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        qdot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        tau[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        qddot_3dof[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    }

    qddot_emulated = qddot_3dof;

    updateKinematicsCustom(model_emulated, &q, &qdot, &qddot_emulated);
    updateKinematicsCustom(model_3dof, &q, &qdot, &qddot_emulated);

    EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(model_emulated.bodyFrames[3]->getTransformToRoot().E, model_3dof.bodyFrames[1]->getTransformToRoot().E,
                                                           unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model_emulated.v[3], model_3dof.v[1], unit_test_utils::E_MINUS_12));

    VectorNd id_emulated(tau);
    VectorNd id_3dof(tau);
    inverseDynamics(model_emulated, q, qdot, qddot_emulated, id_emulated);
    inverseDynamics(model_3dof, q, qdot, qddot_emulated, id_3dof);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(id_emulated, id_3dof, unit_test_utils::E_MINUS_12));

    RobotDynamics::Math::MatrixNd H = RobotDynamics::Math::MatrixNd::Zero(model_emulated.qdot_size, model_emulated.qdot_size);
    RobotDynamics::Math::VectorNd C = RobotDynamics::Math::VectorNd::Zero(model_emulated.qdot_size);
    forwardDynamicsLagrangian(model_emulated, q, qdot, tau, qddot_emulated, H, C);

    H = RobotDynamics::Math::MatrixNd::Zero(model_3dof.qdot_size, model_3dof.qdot_size);
    C = RobotDynamics::Math::VectorNd::Zero(model_3dof.qdot_size);
    forwardDynamicsLagrangian(model_3dof, q, qdot, tau, qddot_3dof, H, C);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_emulated, qddot_3dof, unit_test_utils::E_MINUS_12));

    MatrixNd H_emulated(MatrixNd::Zero(q.size(), q.size()));
    MatrixNd H_3dof(MatrixNd::Zero(q.size(), q.size()));

    compositeRigidBodyAlgorithm(model_emulated, q, H_emulated);
    compositeRigidBodyAlgorithm(model_3dof, q, H_3dof);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(H_emulated, H_3dof, unit_test_utils::E_MINUS_12));
}

TEST_F(Human36, TestUpdateKinematics)
{
    randomizeStates();

    VectorNd id_emulated(tau);
    VectorNd id_3dof(tau);

    updateKinematics(*model_emulated, q, qdot, qddot);
    updateKinematics(*model_3dof, q, qdot, qddot);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model_emulated->v[body_id_emulated[BodyPelvis]], model_3dof->v[body_id_3dof[BodyPelvis]],
                                                                 unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model_emulated->v[body_id_emulated[BodyThighRight]], model_3dof->v[body_id_3dof[BodyThighRight]],
                                                                 unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model_emulated->v[body_id_emulated[BodyShankRight]], model_3dof->v[body_id_3dof[BodyShankRight]],
                                                                 unit_test_utils::E_MINUS_12));
}

TEST_F(Human36, TestInverseDynamics)
{
    randomizeStates();

    VectorNd id_emulated(tau);
    VectorNd id_3dof(tau);

    inverseDynamics(*model_emulated, q, qdot, qddot, id_emulated);
    inverseDynamics(*model_3dof, q, qdot, qddot, id_3dof);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(id_emulated, id_3dof, unit_test_utils::E_MINUS_12 * id_emulated.norm()));
}

TEST_F(Human36, TestNonlinearEffects)
{
    randomizeStates();

    VectorNd nle_emulated(tau);
    VectorNd nle_3dof(tau);

    nonlinearEffects(*model_emulated, q, qdot, nle_emulated);
    nonlinearEffects(*model_3dof, q, qdot, nle_3dof);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(nle_emulated, nle_3dof, unit_test_utils::E_MINUS_12 * nle_emulated.norm()));
}

TEST_F(Human36, TestContactsEmulatedLagrangianKokkevis)
{
    randomizeStates();

    VectorNd qddot_lagrangian(qddot_emulated);
    VectorNd qddot_kokkevis(qddot_emulated);

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_1B1C_emulated, qddot_lagrangian);
    forwardDynamicsContactsKokkevis(*model_emulated, q, qdot, tau, constraints_1B1C_emulated, qddot_kokkevis);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_kokkevis, unit_test_utils::E_MINUS_12 * qddot_lagrangian.norm()));

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_1B4C_emulated, qddot_lagrangian);
    forwardDynamicsContactsKokkevis(*model_emulated, q, qdot, tau, constraints_1B4C_emulated, qddot_kokkevis);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_kokkevis, unit_test_utils::E_MINUS_12 * qddot_lagrangian.norm()));

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_4B4C_emulated, qddot_lagrangian);
    forwardDynamicsContactsKokkevis(*model_emulated, q, qdot, tau, constraints_4B4C_emulated, qddot_kokkevis);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_kokkevis, 1000 * unit_test_utils::E_MINUS_12 * qddot_lagrangian.norm()));
}

TEST_F(Human36, TestContactsEmulatedLagrangianSparse)
{
    randomizeStates();

    VectorNd qddot_lagrangian(qddot_emulated);
    VectorNd qddot_sparse(qddot_emulated);

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_1B1C_emulated, qddot_lagrangian);
    forwardDynamicsContactsRangeSpaceSparse(*model_emulated, q, qdot, tau, constraints_1B1C_emulated, qddot_sparse);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_sparse, unit_test_utils::E_MINUS_12 * qddot_lagrangian.norm()));

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_1B4C_emulated, qddot_lagrangian);
    forwardDynamicsContactsRangeSpaceSparse(*model_emulated, q, qdot, tau, constraints_1B4C_emulated, qddot_sparse);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_sparse, unit_test_utils::E_MINUS_12 * qddot_lagrangian.norm()));

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_4B4C_emulated, qddot_lagrangian);
    forwardDynamicsContactsRangeSpaceSparse(*model_emulated, q, qdot, tau, constraints_4B4C_emulated, qddot_sparse);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_sparse, unit_test_utils::E_MINUS_12 * qddot_lagrangian.norm()));
}

TEST_F(Human36, TestContactsEmulatedLagrangianNullSpaceLinearSolverPartialPivLU)
{
    randomizeStates();

    VectorNd qddot_lagrangian(qddot_emulated);
    VectorNd qddot_nullspace(qddot_emulated);

    constraints_1B1C_emulated.linear_solver = RobotDynamics::Math::LinearSolver::LinearSolverPartialPivLU;

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_1B1C_emulated, qddot_lagrangian);
    forwardDynamicsContactsNullSpace(*model_emulated, q, qdot, tau, constraints_1B1C_emulated, qddot_nullspace);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_nullspace, unit_test_utils::E_MINUS_12 * qddot_lagrangian.norm()));

    constraints_1B4C_emulated.linear_solver = RobotDynamics::Math::LinearSolver::LinearSolverPartialPivLU;

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_1B4C_emulated, qddot_lagrangian);
    forwardDynamicsContactsNullSpace(*model_emulated, q, qdot, tau, constraints_1B4C_emulated, qddot_nullspace);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_nullspace, unit_test_utils::E_MINUS_12 * qddot_lagrangian.norm()));

    constraints_4B4C_emulated.linear_solver = RobotDynamics::Math::LinearSolver::LinearSolverPartialPivLU;

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_4B4C_emulated, qddot_lagrangian);
    forwardDynamicsContactsNullSpace(*model_emulated, q, qdot, tau, constraints_4B4C_emulated, qddot_nullspace);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_nullspace, unit_test_utils::E_MINUS_12 * qddot_lagrangian.norm()));
}

TEST_F(Human36, TestContactsEmulatedLagrangianNullSpaceLinearSolverHouseholderQR)
{
    randomizeStates();

    VectorNd qddot_lagrangian(qddot_emulated);
    VectorNd qddot_nullspace(qddot_emulated);

    constraints_1B1C_emulated.linear_solver = RobotDynamics::Math::LinearSolver::LinearSolverHouseholderQR;

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_1B1C_emulated, qddot_lagrangian);
    forwardDynamicsContactsNullSpace(*model_emulated, q, qdot, tau, constraints_1B1C_emulated, qddot_nullspace);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_nullspace, unit_test_utils::E_MINUS_12 * qddot_lagrangian.norm()));

    constraints_1B4C_emulated.linear_solver = RobotDynamics::Math::LinearSolver::LinearSolverHouseholderQR;

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_1B4C_emulated, qddot_lagrangian);
    forwardDynamicsContactsNullSpace(*model_emulated, q, qdot, tau, constraints_1B4C_emulated, qddot_nullspace);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_nullspace, unit_test_utils::E_MINUS_12 * qddot_lagrangian.norm()));

    constraints_4B4C_emulated.linear_solver = RobotDynamics::Math::LinearSolver::LinearSolverHouseholderQR;

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_4B4C_emulated, qddot_lagrangian);
    forwardDynamicsContactsNullSpace(*model_emulated, q, qdot, tau, constraints_4B4C_emulated, qddot_nullspace);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_nullspace, unit_test_utils::E_MINUS_12 * qddot_lagrangian.norm()));
}

TEST_F(Human36, TestContactsEmulatedLagrangianNullSpace)
{
    randomizeStates();

    VectorNd qddot_lagrangian(qddot_emulated);
    VectorNd qddot_nullspace(qddot_emulated);

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_1B1C_emulated, qddot_lagrangian);
    forwardDynamicsContactsNullSpace(*model_emulated, q, qdot, tau, constraints_1B1C_emulated, qddot_nullspace);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_nullspace, unit_test_utils::E_MINUS_12 * qddot_lagrangian.norm()));

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_1B4C_emulated, qddot_lagrangian);
    forwardDynamicsContactsNullSpace(*model_emulated, q, qdot, tau, constraints_1B4C_emulated, qddot_nullspace);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_nullspace, unit_test_utils::E_MINUS_12 * qddot_lagrangian.norm()));

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_4B4C_emulated, qddot_lagrangian);
    forwardDynamicsContactsNullSpace(*model_emulated, q, qdot, tau, constraints_4B4C_emulated, qddot_nullspace);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_lagrangian, qddot_nullspace, unit_test_utils::E_MINUS_12 * qddot_lagrangian.norm()));
}

TEST_F(Human36, TestContactsEmulatedMultdofLagrangian)
{
    randomizeStates();

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_1B1C_emulated, qddot_emulated);
    forwardDynamicsContactsDirect(*model_3dof, q, qdot, tau, constraints_1B1C_3dof, qddot_3dof);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_emulated, qddot_3dof, unit_test_utils::E_MINUS_12 * qddot_emulated.norm()));

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_1B4C_emulated, qddot_emulated);
    forwardDynamicsContactsDirect(*model_3dof, q, qdot, tau, constraints_1B4C_3dof, qddot_3dof);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_emulated, qddot_3dof, unit_test_utils::E_MINUS_12 * qddot_emulated.norm()));

    forwardDynamicsContactsDirect(*model_emulated, q, qdot, tau, constraints_4B4C_emulated, qddot_emulated);
    forwardDynamicsContactsDirect(*model_3dof, q, qdot, tau, constraints_4B4C_3dof, qddot_3dof);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_emulated, qddot_3dof, unit_test_utils::E_MINUS_12 * qddot_emulated.norm()));
}

TEST_F(Human36, TestContactsEmulatedMultdofSparse)
{
    for (unsigned int i = 0; i < q.size(); i++)
    {
        q[i] = 0.4 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        qdot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        tau[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    }

    forwardDynamicsContactsRangeSpaceSparse(*model_emulated, q, qdot, tau, constraints_1B1C_emulated, qddot_emulated);

    for (unsigned int i = 0; i < q.size(); i++)
    {
        EXPECT_EQ(model_emulated->lambda_q[i], model_3dof->lambda_q[i]);
    }

    forwardDynamicsContactsRangeSpaceSparse(*model_3dof, q, qdot, tau, constraints_1B1C_3dof, qddot_3dof);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_emulated, qddot_3dof, unit_test_utils::E_MINUS_12 * qddot_emulated.norm()));

    forwardDynamicsContactsRangeSpaceSparse(*model_emulated, q, qdot, tau, constraints_1B4C_emulated, qddot_emulated);
    forwardDynamicsContactsRangeSpaceSparse(*model_3dof, q, qdot, tau, constraints_1B4C_3dof, qddot_3dof);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_emulated, qddot_3dof, unit_test_utils::E_MINUS_12 * qddot_emulated.norm()));

    forwardDynamicsContactsRangeSpaceSparse(*model_emulated, q, qdot, tau, constraints_4B4C_emulated, qddot_emulated);
    forwardDynamicsContactsRangeSpaceSparse(*model_3dof, q, qdot, tau, constraints_4B4C_3dof, qddot_3dof);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_emulated, qddot_3dof, unit_test_utils::E_MINUS_12 * qddot_emulated.norm()));
}

TEST_F(Human36, TestContactsEmulatedMultdofKokkevisSparse)
{
    randomizeStates();

    forwardDynamicsContactsRangeSpaceSparse(*model_emulated, q, qdot, tau, constraints_1B1C_emulated, qddot_emulated);

    for (unsigned int i = 0; i < q.size(); i++)
    {
        EXPECT_EQ(model_emulated->lambda_q[i], model_3dof->lambda_q[i]);
    }

    VectorNd qddot_sparse(qddot_emulated);
    VectorNd qddot_kokkevis(qddot_emulated);

    forwardDynamicsContactsRangeSpaceSparse(*model_3dof, q, qdot, tau, constraints_1B1C_3dof, qddot_sparse);
    forwardDynamicsContactsKokkevis(*model_3dof, q, qdot, tau, constraints_1B1C_3dof, qddot_kokkevis);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_sparse, qddot_kokkevis, unit_test_utils::E_MINUS_12 * qddot_sparse.norm()));

    forwardDynamicsContactsRangeSpaceSparse(*model_3dof, q, qdot, tau, constraints_1B4C_3dof, qddot_sparse);
    forwardDynamicsContactsKokkevis(*model_3dof, q, qdot, tau, constraints_1B4C_3dof, qddot_kokkevis);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_sparse, qddot_kokkevis, unit_test_utils::E_MINUS_12 * qddot_sparse.norm()));

    forwardDynamicsContactsRangeSpaceSparse(*model_3dof, q, qdot, tau, constraints_4B4C_3dof, qddot_sparse);
    forwardDynamicsContactsKokkevis(*model_3dof, q, qdot, tau, constraints_4B4C_3dof, qddot_kokkevis);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_sparse, qddot_kokkevis, 1000 * unit_test_utils::E_MINUS_12 * qddot_sparse.norm()));
}

TEST_F(Human36, TestContactsEmulatedMultdofKokkevisMultiple)
{
    randomizeStates();

    VectorNd qddot_kokkevis(qddot_emulated);
    VectorNd qddot_kokkevis_2(qddot_emulated);

    forwardDynamicsContactsKokkevis(*model_3dof, q, qdot, tau, constraints_1B1C_3dof, qddot_kokkevis);
    forwardDynamicsContactsKokkevis(*model_3dof, q, qdot, tau, constraints_1B1C_3dof, qddot_kokkevis_2);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_kokkevis, qddot_kokkevis_2, unit_test_utils::E_MINUS_12 * qddot_kokkevis.norm()));

    forwardDynamicsContactsKokkevis(*model_3dof, q, qdot, tau, constraints_1B4C_3dof, qddot_kokkevis);
    forwardDynamicsContactsKokkevis(*model_3dof, q, qdot, tau, constraints_1B4C_3dof, qddot_kokkevis_2);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_kokkevis, qddot_kokkevis_2, unit_test_utils::E_MINUS_12 * qddot_kokkevis.norm()));

    forwardDynamicsContactsKokkevis(*model_3dof, q, qdot, tau, constraints_4B4C_3dof, qddot_kokkevis);
    forwardDynamicsContactsKokkevis(*model_3dof, q, qdot, tau, constraints_4B4C_3dof, qddot_kokkevis_2);
    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_kokkevis, qddot_kokkevis_2, unit_test_utils::E_MINUS_12 * qddot_kokkevis.norm()));
}

TEST_F(SphericalJointFixture, TestEulerZYXvsEmulated)
{
    emuQ[0] = 1.1;
    emuQ[1] = 1.2;
    emuQ[2] = 1.3;
    emuQ[3] = 1.4;
    emuQ[4] = 1.5;

    emuQDot[0] = 1.;
    emuQDot[1] = 2.;
    emuQDot[2] = 3.;
    emuQDot[3] = 4.;
    emuQDot[4] = 5.;

    emuTau[0] = 5.;
    emuTau[1] = 4.;
    emuTau[2] = 7.;
    emuTau[3] = 3.;
    emuTau[4] = 2.;

    VectorNd QDDot_emu = VectorNd::Zero(emulated_model.qdot_size);
    VectorNd QDDot_eulerzyx = VectorNd::Zero(eulerzyx_model.qdot_size);

    RobotDynamics::Math::MatrixNd H = RobotDynamics::Math::MatrixNd::Zero(emulated_model.qdot_size, emulated_model.qdot_size);
    RobotDynamics::Math::VectorNd C = RobotDynamics::Math::VectorNd::Zero(emulated_model.qdot_size);
    forwardDynamicsLagrangian(emulated_model, emuQ, emuQDot, emuTau, QDDot_emu, H, C);

    H = RobotDynamics::Math::MatrixNd::Zero(eulerzyx_model.qdot_size, eulerzyx_model.qdot_size);
    C = RobotDynamics::Math::VectorNd::Zero(eulerzyx_model.qdot_size);
    forwardDynamicsLagrangian(eulerzyx_model, emuQ, emuQDot, emuTau, QDDot_eulerzyx, H, C);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_emu, QDDot_eulerzyx, unit_test_utils::E_MINUS_12));
}

TEST_F(Human36, SolveMInvTimesTau)
{
    for (unsigned int i = 0; i < model->dof_count; i++)
    {
        q[i] = rand() / static_cast<double>(RAND_MAX);
        tau[i] = rand() / static_cast<double>(RAND_MAX);
    }

    MatrixNd M(MatrixNd::Zero(model->dof_count, model->dof_count));
    compositeRigidBodyAlgorithm(*model, q, M);

    VectorNd qddot_solve_llt = M.llt().solve(tau);

    VectorNd qddot_minv(q);
    calcMInvTimesTau(*model, q, tau, qddot_minv);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_solve_llt, qddot_minv, unit_test_utils::E_MINUS_12 * qddot_minv.norm()));
}

TEST_F(Human36, SolveMInvTimesTauReuse)
{
    for (unsigned int i = 0; i < model->dof_count; i++)
    {
        q[i] = rand() / static_cast<double>(RAND_MAX);
        tau[i] = rand() / static_cast<double>(RAND_MAX);
    }

    MatrixNd M(MatrixNd::Zero(model->dof_count, model->dof_count));
    compositeRigidBodyAlgorithm(*model, q, M);

    VectorNd qddot_solve_llt = M.llt().solve(tau);

    VectorNd qddot_minv(q);
    calcMInvTimesTau(*model, q, tau, qddot_minv);

    for (unsigned int j = 0; j < 1; j++)
    {
        for (unsigned int i = 0; i < model->dof_count; i++)
        {
            tau[i] = rand() / static_cast<double>(RAND_MAX);
        }

        compositeRigidBodyAlgorithm(*model, q, M);
        qddot_solve_llt = M.llt().solve(tau);

        calcMInvTimesTau(*model, q, tau, qddot_minv);

        EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(qddot_solve_llt, qddot_minv, unit_test_utils::E_MINUS_12 * qddot_solve_llt.norm()));
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
