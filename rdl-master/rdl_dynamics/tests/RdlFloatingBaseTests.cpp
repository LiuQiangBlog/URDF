// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <iostream>

#include <gtest/gtest.h>

#include "rdl_dynamics/Dynamics.hpp"
#include "rdl_dynamics/FramePoint.hpp"
#include "rdl_dynamics/Kinematics.hpp"
#include "rdl_dynamics/Model.hpp"
#include "UnitTestUtils.hpp"

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

struct FloatingBaseTestFixture : public testing::Test
{
    FloatingBaseTestFixture()
    {
    }

    void SetUp()
    {
        model = new Model;
        model->gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

        base = Body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
    }

    void TearDown()
    {
        delete model;
    }

    Model* model;
    Body base;
    unsigned int base_body_id;

    VectorNd q, qdot, qddot, tau;
};

TEST_F(FloatingBaseTestFixture, TestCalcPointTransformation)
{
    base_body_id = model->addBody(0, SpatialTransform(),
                                  Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                                        SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                                  base);

    q = VectorNd::Constant(model->dof_count, 0.);
    qdot = VectorNd::Constant(model->dof_count, 0.);
    qddot = VectorNd::Constant(model->dof_count, 0.);
    tau = VectorNd::Constant(model->dof_count, 0.);

    q[1] = 1.;
    forwardDynamics(*model, q, qdot, tau, qddot);

    FramePoint p(model->worldFrame, Vector3d::Zero());
    p.changeFrame(model->bodyFrames[base_body_id]);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., -1., 0.), p.vec(), unit_test_utils::E_MINUS_14));
}

TEST_F(FloatingBaseTestFixture, TestCalcDynamicFloatingBaseDoubleImplicit)
{
    // floating base
    base_body_id = model->addBody(0, SpatialTransform(),
                                  Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                                        SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                                  base);

    // body_a
    Body body_a(1., Vector3d(1., 0., 0), Vector3d(1., 1., 1.));
    Joint joint_a(SpatialVector(0., 0., 1., 0., 0., 0.));

    model->addBody(base_body_id, Xtrans(Vector3d(2., 0., 0.)), joint_a, body_a);

    // Initialization of the input vectors
    VectorNd Q = VectorNd::Zero((size_t)model->dof_count);
    VectorNd QDot = VectorNd::Zero((size_t)model->dof_count);
    VectorNd QDDot = VectorNd::Zero((size_t)model->dof_count);
    VectorNd Tau = VectorNd::Zero((size_t)model->dof_count);

    forwardDynamics(*model, Q, QDot, Tau, QDDot);

    EXPECT_NEAR(0.0000, QDDot[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(-9.8100, QDDot[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0.0000, QDDot[2], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0.0000, QDDot[3], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0.0000, QDDot[4], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0.0000, QDDot[5], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0.0000, QDDot[6], unit_test_utils::E_MINUS_14);

    // We rotate the base... let's see what happens...
    Q[3] = 0.8;
    forwardDynamics(*model, Q, QDot, Tau, QDDot);

    EXPECT_NEAR(0.0000, QDDot[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(-9.8100, QDDot[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0.0000, QDDot[2], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0.0000, QDDot[3], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0.0000, QDDot[4], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0.0000, QDDot[5], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0.0000, QDDot[6], unit_test_utils::E_MINUS_14);

    // We apply a torqe let's see what happens...
    Q[3] = 0.;
    Tau[6] = 1.;

    forwardDynamics(*model, Q, QDot, Tau, QDDot);

    EXPECT_NEAR(0.0000, QDDot[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(-8.8100, QDDot[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0.0000, QDDot[2], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(-1.0000, QDDot[3], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0.0000, QDDot[4], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0.0000, QDDot[5], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(2.0000, QDDot[6], unit_test_utils::E_MINUS_14);
}

TEST_F(FloatingBaseTestFixture, TestCalcPointVelocityFloatingBaseSimple)
{
    // floating base
    base_body_id = model->addBody(0, SpatialTransform(),
                                  Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                                        SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                                  base);

    VectorNd Q = VectorNd::Zero(model->dof_count);
    VectorNd QDot = VectorNd::Zero(model->dof_count);
    VectorNd QDDot = VectorNd::Zero(model->dof_count);
    VectorNd Tau = VectorNd::Zero(model->dof_count);

    unsigned int ref_body_id = base_body_id;

    // first we calculate the velocity when moving along the X axis
    QDot[0] = 1.;
    Vector3d point_position(1., 0., 0.);
    Vector3d point_velocity;

    point_velocity = calcPointVelocity(*model, Q, QDot, ref_body_id, point_position);

    EXPECT_NEAR(1., point_velocity[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_velocity[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_velocity[2], unit_test_utils::E_MINUS_14);

    // Now we calculate the velocity when rotating around the Z axis
    QDot[0] = 0.;
    QDot[3] = 1.;

    point_velocity = calcPointVelocity(*model, Q, QDot, ref_body_id, point_position);

    EXPECT_NEAR(0., point_velocity[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(1., point_velocity[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_velocity[2], unit_test_utils::E_MINUS_14);

    // Now we calculate the velocity when rotating around the Z axis and the
    // base is rotated around the z axis by 90 degrees
    Q[3] = M_PI * 0.5;
    QDot[3] = 1.;

    point_velocity = calcPointVelocity(*model, Q, QDot, ref_body_id, point_position);

    EXPECT_NEAR(-1., point_velocity[0], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_velocity[1], unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(0., point_velocity[2], unit_test_utils::E_MINUS_14);
}

TEST_F(FloatingBaseTestFixture, TestCalcPointVelocityCustom)
{
    // floating base
    base = Body(1., Vector3d(0., 1., 0.), Vector3d(1., 1., 1.));
    base_body_id = model->addBody(0, SpatialTransform(),
                                  Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                                        SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                                  base);

    VectorNd q = VectorNd::Zero(model->dof_count);
    VectorNd qdot = VectorNd::Zero(model->dof_count);
    VectorNd qddot = VectorNd::Zero(model->dof_count);
    VectorNd tau = VectorNd::Zero(model->dof_count);

    unsigned int ref_body_id = base_body_id;

    q[0] = 0.1;
    q[1] = 1.1;
    q[2] = 1.2;
    q[3] = 1.3;
    q[4] = 1.5;
    q[5] = 1.7;

    qdot[0] = 0.1;
    qdot[1] = 1.1;
    qdot[2] = 1.2;
    qdot[3] = 1.3;
    qdot[4] = 1.5;
    qdot[5] = 1.7;

    // first we calculate the velocity when rotating around the Z axis
    Vector3d point_body_position(1., 0., 0.);
    Vector3d point_base_position;
    Vector3d point_base_velocity;
    Vector3d point_base_velocity_reference;

    forwardDynamics(*model, q, qdot, tau, qddot);

    point_base_velocity = calcPointVelocity(*model, q, qdot, ref_body_id, point_body_position);

    point_base_velocity_reference = Vector3d(-3.888503432977729e-01, -3.171179347202455e-01, 1.093894197498446e+00);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(point_base_velocity_reference, point_base_velocity, unit_test_utils::E_MINUS_14));
}

/** \brief Compares computation of acceleration values for zero qddot
 *
 * Ensures that computation of position, velocity, and acceleration of a
 * point produce the same values as in an equivalent model that was
 * created with the HuMAnS toolbox
 *    http://www.inrialpes.fr/bipop/software/humans/ .
 * Here we omit the term of the generalized acceleration by setting qddot
 * to zero.
 */
TEST_F(FloatingBaseTestFixture, TestCalcPointAccelerationNoQDDot)
{
    // floating base
    base = Body(1., Vector3d(0., 1., 0.), Vector3d(1., 1., 1.));
    base_body_id = model->addBody(0, SpatialTransform(),
                                  Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                                        SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                                  base);

    VectorNd q = VectorNd::Zero(model->dof_count);
    VectorNd qdot = VectorNd::Zero(model->dof_count);
    VectorNd qddot = VectorNd::Zero(model->dof_count);
    VectorNd tau = VectorNd::Zero(model->dof_count);

    unsigned int ref_body_id = base_body_id;

    q[0] = 0.1;
    q[1] = 1.1;
    q[2] = 1.2;
    q[3] = 1.3;
    q[4] = 1.5;
    q[5] = 1.7;

    qdot[0] = 0.1;
    qdot[1] = 1.1;
    qdot[2] = 1.2;
    qdot[3] = 1.3;
    qdot[4] = 1.5;
    qdot[5] = 1.7;

    // first we calculate the velocity when rotating around the Z axis
    Vector3d point_body_position(-1.9, -1.8, 0.);
    Vector3d point_world_velocity;
    Vector3d point_world_acceleration;

    // call ForwardDynamics to update the model
    forwardDynamics(*model, q, qdot, tau, qddot);
    qddot = VectorNd::Zero(qddot.size());

    qdot = qdot;

    FramePoint point_world_position(model->bodyFrames[ref_body_id], point_body_position);
    point_world_position.changeFrame(model->worldFrame);
    point_world_velocity = calcPointVelocity(*model, q, qdot, ref_body_id, point_body_position);

    point_world_acceleration = calcPointAcceleration(*model, q, qdot, qddot, ref_body_id, point_body_position);

    Vector3d humans_point_position(-6.357089363622626e-01, -6.831041744630977e-01, 2.968974805916970e+00);
    Vector3d humans_point_velocity(3.091226260907569e-01, 3.891012095550828e+00, 4.100277995030419e+00);
    Vector3d humans_point_acceleration(-5.302760158847160e+00, 6.541369639625232e+00, -4.795115077652286e+00);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(humans_point_position, point_world_position.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(humans_point_velocity, point_world_velocity, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(humans_point_acceleration, point_world_acceleration, unit_test_utils::E_MINUS_14));
}

/** \brief Compares computation of acceleration values for zero q and qdot
 *
 * Ensures that computation of position, velocity, and acceleration of a
 * point produce the same values as in an equivalent model that was
 * created with the HuMAnS toolbox
 *    http://www.inrialpes.fr/bipop/software/humans/ .
 *
 * Here we set q and qdot to zero and only take into account values that
 * are dependent on qddot.
 */
TEST_F(FloatingBaseTestFixture, TestCalcPointAccelerationOnlyQDDot)
{
    // floating base
    base = Body(1., Vector3d(0., 1., 0.), Vector3d(1., 1., 1.));
    base_body_id = model->addBody(0, SpatialTransform(),
                                  Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                                        SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                                  base);

    VectorNd q = VectorNd::Zero(model->dof_count);
    VectorNd qdot = VectorNd::Zero(model->dof_count);
    VectorNd qddot = VectorNd::Zero(model->dof_count);
    VectorNd tau = VectorNd::Zero(model->dof_count);

    unsigned int ref_body_id = base_body_id;

    // first we calculate the velocity when rotating around the Z axis
    Vector3d point_body_position(-1.9, -1.8, 0.);
    Vector3d point_world_velocity;
    Vector3d point_world_acceleration;

    forwardDynamics(*model, q, qdot, tau, qddot);

    qddot = VectorNd::Zero(qddot.size());

    qddot[0] = 0.1;
    qddot[1] = 1.1;
    qddot[2] = 1.2;
    qddot[3] = 1.3;
    qddot[4] = 1.5;
    qddot[5] = 1.7;

    FramePoint point_world_position(model->bodyFrames[ref_body_id], point_body_position);
    point_world_position.changeFrame(model->worldFrame);
    point_world_velocity = calcPointVelocity(*model, q, qdot, ref_body_id, point_body_position);

    point_world_acceleration = calcPointAcceleration(*model, q, qdot, qddot, ref_body_id, point_body_position);

    Vector3d humans_point_position(-1.900000000000000e+00, -1.800000000000000e+00, 0.000000000000000e+00);
    Vector3d humans_point_velocity(0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00);
    Vector3d humans_point_acceleration(2.440000000000000e+00, -1.370000000000000e+00, 9.899999999999999e-01);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(humans_point_position, point_world_position.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(humans_point_velocity, point_world_velocity, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(humans_point_acceleration, point_world_acceleration, unit_test_utils::E_MINUS_14));
}

/** \brief Compares computation of acceleration values for zero q and qdot
 *
 * Ensures that computation of position, velocity, and acceleration of a
 * point produce the same values as in an equivalent model that was
 * created with the HuMAnS toolbox
 *    http://www.inrialpes.fr/bipop/software/humans/ .
 *
 * Here we set q and qdot to zero and only take into account values that
 * are dependent on qddot.
 */
TEST_F(FloatingBaseTestFixture, TestCalcPointAccelerationFull)
{
    // floating base
    base = Body(1., Vector3d(0., 1., 0.), Vector3d(1., 1., 1.));
    base_body_id = model->addBody(0, SpatialTransform(),
                                  Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                                        SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                                  base);

    VectorNd q = VectorNd::Zero(model->dof_count);
    VectorNd qdot = VectorNd::Zero(model->dof_count);
    VectorNd qddot = VectorNd::Zero(model->dof_count);
    VectorNd tau = VectorNd::Zero(model->dof_count);

    unsigned int ref_body_id = base_body_id;

    // first we calculate the velocity when rotating around the Z axis
    Vector3d point_body_position(-1.9, -1.8, 0.);
    Vector3d point_world_velocity;
    Vector3d point_world_acceleration;

    q[0] = 0.1;
    q[1] = 1.1;
    q[2] = 1.2;
    q[3] = 1.3;
    q[4] = 1.5;
    q[5] = 1.7;

    qdot[0] = 0.1;
    qdot[1] = 1.1;
    qdot[2] = 1.2;
    qdot[3] = 1.3;
    qdot[4] = 1.5;
    qdot[5] = 1.7;

    forwardDynamics(*model, q, qdot, tau, qddot);

    qddot[0] = 0.1;
    qddot[1] = 1.1;
    qddot[2] = 1.2;
    qddot[3] = 1.3;
    qddot[4] = 1.5;
    qddot[5] = 1.7;

    //	cout << "ref_body_id = " << ref_body_id << endl;
    //	cout << "point_body_position = " << point_body_position << endl;
    FramePoint point_world_position(model->bodyFrames[ref_body_id], point_body_position);
    point_world_position.changeFrame(model->worldFrame);
    point_world_velocity = calcPointVelocity(*model, q, qdot, ref_body_id, point_body_position);

    point_world_acceleration = calcPointAcceleration(*model, q, qdot, qddot, ref_body_id, point_body_position);

    Vector3d humans_point_position(-6.357089363622626e-01, -6.831041744630977e-01, 2.968974805916970e+00);
    Vector3d humans_point_velocity(3.091226260907569e-01, 3.891012095550828e+00, 4.100277995030419e+00);
    Vector3d humans_point_acceleration(-4.993637532756404e+00, 1.043238173517606e+01, -6.948370826218673e-01);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(humans_point_position, point_world_position.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(humans_point_velocity, point_world_velocity, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(humans_point_acceleration, point_world_acceleration, unit_test_utils::E_MINUS_14));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
