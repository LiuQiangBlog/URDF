// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>
#include <iostream>

#include "Fixtures.hpp"
#include "rdl_dynamics/FramePoint.hpp"
#include "rdl_dynamics/Kinematics.hpp"
#include "rdl_dynamics/Dynamics.hpp"
#include "rdl_dynamics/Model.hpp"
#include "rdl_dynamics/rdl_mathutils.hpp"
#include "UnitTestUtils.hpp"

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

struct RdlModelFixture : public testing::Test
{
    RdlModelFixture()
    {
    }

    void SetUp()
    {
        model = new Model;
        model->gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);
    }

    void TearDown()
    {
        delete model;
    }

    Model* model;
};

TEST_F(RdlModelFixture, testCenterOfBuoyancyFrames)
{
    model->gravity = SpatialVector(0., 0., 0., 0., 0., -9.81);
    Body b(0.1, Vector3d(0.1, 0., 0.), Vector3d(1., 1., 1.), Vector3d(0.12, 0., 0.), 0.00005, Math::SpatialMatrixZero, DragData());
    Joint j(JointTypeRevoluteY);
    unsigned int b_id = model->addBody(0, SpatialTransform(), j, b, "body1");

    b = Body(0.1, Vector3d(0., 0., -0.1), Vector3d(1., 1., 1.), Vector3d(0., 0., -0.12), 0.00005, Math::SpatialMatrixZero, DragData());
    j = Joint(JointTypeRevoluteY);
    model->addBody(b_id, Xtrans(Vector3d(0.2, 0., 0.)), j, b, "body2");

    VectorNd q(model->qdot_size);
    VectorNd qdot(model->qdot_size);

    q.setZero();
    qdot.setZero();

    updateKinematics(*model, q, qdot, qdot);

    EXPECT_TRUE(
        Xtrans(Vector3d(0.12, 0., 0.)).toMatrix().isApprox(model->bodyCenterOfBuoyancyFrames[1]->getTransformFromParent().toMatrix(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(
        Xtrans(Vector3d(0., 0., -0.12)).toMatrix().isApprox(model->bodyCenterOfBuoyancyFrames[2]->getTransformFromParent().toMatrix(), unit_test_utils::E_MINUS_14));

    EXPECT_TRUE(
        Xtrans(Vector3d(0.12, 0., 0.)).toMatrix().isApprox(model->bodyCenterOfBuoyancyFrames[1]->getInverseTransformToRoot().toMatrix(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(
        Xtrans(Vector3d(0.2, 0., -0.12)).toMatrix().isApprox(model->bodyCenterOfBuoyancyFrames[2]->getInverseTransformToRoot().toMatrix(), unit_test_utils::E_MINUS_14));

    q[1] = -M_PI_2;

    updateKinematics(*model, q, qdot, qdot);

    EXPECT_TRUE(
        Xtrans(Vector3d(0.12, 0., 0.)).toMatrix().isApprox(model->bodyCenterOfBuoyancyFrames[1]->getTransformFromParent().toMatrix(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(Xtrans(Vector3d(0.32, 0., 0.)).r.isApprox(model->bodyCenterOfBuoyancyFrames[2]->getInverseTransformToRoot().r, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlModelFixture, testCommonParentId)
{
    Body b(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));
    Joint j(JointTypeRevoluteX);

    unsigned int id1 = model->appendBody(SpatialTransform(), j, b, "b1");
    unsigned int id2 = model->appendBody(SpatialTransform(), j, b, "b2");
    unsigned int id3 = model->addBody(id1, SpatialTransform(), j, b, "b3");
    (void)id3;
    unsigned int id4 = model->addBody(id2, SpatialTransform(), j, b, "b4");
    unsigned int id5 = model->addBody(id2, SpatialTransform(), j, b, "b5");
    (void)id5;
    unsigned int id6 = model->addBody(id2, SpatialTransform(), j, b, "b6");

    unsigned int id7 = model->addBody(id6, SpatialTransform(), j, b, "b7");
    unsigned int id8 = model->addBody(id6, SpatialTransform(), j, b, "b8");

    EXPECT_EQ(model->getCommonMovableParentId(id8, id7), id6);
    EXPECT_EQ(model->getCommonMovableParentId(id1, id4), id1);
    EXPECT_EQ(model->getCommonMovableParentId(id7, id8), id6);
    EXPECT_EQ(model->getCommonMovableParentId(id4, id1), id1);
    EXPECT_EQ(model->getCommonMovableParentId(0, id8), (unsigned int)0);
    EXPECT_EQ(model->getCommonMovableParentId(id4, id4), id4);
    EXPECT_EQ(model->mass, 8.0);

    unsigned int id9 = model->addBody(0, SpatialTransform(), j, b, "b9");
    unsigned int id10 = model->addBody(id9, SpatialTransform(), j, b, "b10");

    EXPECT_EQ(model->getCommonMovableParentId(id10, id8), (unsigned int)0);
    EXPECT_EQ(model->getCommonMovableParentId(id8, id10), (unsigned int)0);

    EXPECT_EQ(model->mass, 10.0);

    Joint jf(JointTypeFixed);
    unsigned int fid1 = model->addBody(id10, SpatialTransform(), jf, b, "fb1");
    unsigned int fid2 = model->addBody(fid1, SpatialTransform(), jf, b, "fb2");
    unsigned int fid3 = model->addBody(id2, SpatialTransform(), jf, b, "fb3");
    unsigned int fid4 = model->addBody(id10, SpatialTransform(), jf, b, "fb4");

    EXPECT_EQ(model->mass, 14.0);

    EXPECT_EQ(model->getCommonMovableParentId(fid1, fid2), id10);
    EXPECT_EQ(model->getCommonMovableParentId(fid2, fid1), id10);
    EXPECT_EQ(model->getCommonMovableParentId(fid1, fid4), id10);
    EXPECT_EQ(model->getCommonMovableParentId(fid4, fid1), id10);
    EXPECT_EQ(model->getCommonMovableParentId(fid1, fid3), (unsigned int)0);
    EXPECT_EQ(model->getCommonMovableParentId(fid1, id9), id9);
}

TEST_F(RdlModelFixture, TestInit)
{
    EXPECT_EQ(1u, model->lambda.size());
    EXPECT_EQ(1u, model->mu.size());
    EXPECT_EQ(0u, model->dof_count);

    EXPECT_EQ(0u, model->q_size);
    EXPECT_EQ(0u, model->qdot_size);

    EXPECT_EQ(1u, model->v.size());
    EXPECT_EQ(1u, model->a.size());

    EXPECT_EQ(1u, model->mJoints.size());

    EXPECT_EQ(1u, model->S.size());

    EXPECT_EQ(1u, model->c.size());
    EXPECT_EQ(1u, model->IA.size());
    EXPECT_EQ(1u, model->pA.size());
    EXPECT_EQ(1u, model->U.size());
    EXPECT_EQ(1u, model->d.size());
    EXPECT_EQ(1u, model->u.size());
    EXPECT_EQ(1u, model->Ic.size());
    EXPECT_EQ(1u, model->I.size());

    EXPECT_EQ(1u, model->X_lambda.size());
    EXPECT_EQ(1u, model->bodyFrames.size());
    EXPECT_EQ(1u, model->bodyCenteredFrames.size());
    EXPECT_EQ(0u, model->fixedBodyFrames.size());
    EXPECT_EQ(1u, model->mBodies.size());
}

TEST_F(RdlModelFixture, TestaddBodyDimensions)
{
    Body body(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));
    Joint joint(SpatialVector(0., 0., 1., 0., 0., 0.));

    unsigned int body_id = 0;
    // Adding null body.
    body_id = model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body);

    EXPECT_EQ(1u, body_id);
    EXPECT_EQ(2u, model->lambda.size());
    EXPECT_EQ(2u, model->mu.size());
    EXPECT_EQ(1u, model->dof_count);

    EXPECT_EQ(2u, model->v.size());
    EXPECT_EQ(2u, model->a.size());

    EXPECT_EQ(2u, model->mJoints.size());
    EXPECT_EQ(2u, model->S.size());

    EXPECT_EQ(2u, model->c.size());
    EXPECT_EQ(2u, model->IA.size());
    EXPECT_EQ(2u, model->pA.size());
    EXPECT_EQ(2u, model->U.size());
    EXPECT_EQ(2u, model->d.size());
    EXPECT_EQ(2u, model->u.size());
    EXPECT_EQ(2u, model->Ic.size());
    EXPECT_EQ(2u, model->I.size());

    SpatialVector spatial_zero;
    spatial_zero.setZero();

    EXPECT_EQ(2u, model->X_lambda.size());
    EXPECT_EQ(2u, model->bodyFrames.size());
    EXPECT_EQ(2u, model->bodyCenteredFrames.size());
    EXPECT_EQ(0u, model->fixedBodyFrames.size());
    EXPECT_EQ(2u, model->mBodies.size());
}

TEST_F(RdlModelFixture, TestFloatingBodyDimensions)
{
    Body body(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));
    Joint float_base_joint(JointTypeFloatingBase);

    model->appendBody(SpatialTransform(), float_base_joint, body, "rootBody");

    EXPECT_EQ(3u, model->lambda.size());
    EXPECT_EQ(3u, model->mu.size());
    EXPECT_EQ(6u, model->dof_count);
    EXPECT_EQ(7u, model->q_size);
    EXPECT_EQ(6u, model->qdot_size);

    EXPECT_EQ(3u, model->v.size());
    EXPECT_EQ(3u, model->a.size());

    EXPECT_EQ(3u, model->mJoints.size());
    EXPECT_EQ(3u, model->S.size());

    EXPECT_EQ(3u, model->c.size());
    EXPECT_EQ(3u, model->IA.size());
    EXPECT_EQ(3u, model->pA.size());
    EXPECT_EQ(3u, model->U.size());
    EXPECT_EQ(3u, model->d.size());
    EXPECT_EQ(3u, model->u.size());

    SpatialVector spatial_zero;
    spatial_zero.setZero();

    EXPECT_EQ(3u, model->X_lambda.size());
    EXPECT_EQ(3u, model->bodyFrames.size());
    EXPECT_EQ(3u, model->bodyCenteredFrames.size());
    EXPECT_EQ(0u, model->fixedBodyFrames.size());
    EXPECT_EQ(3u, model->mBodies.size());
}

/** \brief Tests whether the joint and body information stored in the Model are computed correctly
 */
TEST_F(RdlModelFixture, TestaddBodySpatialValues)
{
    Body body(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));
    Joint joint(SpatialVector(0., 0., 1., 0., 0., 0.));

    model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body);

    SpatialVector spatial_joint_axis(0., 0., 1., 0., 0., 0.);
    EXPECT_EQ(spatial_joint_axis, joint.mJointAxes[0]);
}

TEST_F(RdlModelFixture, testIsBodyId)
{
    Body body(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));
    Joint joint(SpatialVector(0., 0., 1., 0., 0., 0.));

    Joint fixed(JointTypeFixed);

    unsigned int bodyId1 = model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body, "mybody");

    unsigned int fb1 = model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), fixed, body, "fixed");

    EXPECT_TRUE(model->IsBodyId(bodyId1));
    EXPECT_TRUE(model->IsBodyId(fb1));

    EXPECT_FALSE(model->IsBodyId(bodyId1 + 1));
    EXPECT_FALSE(model->IsBodyId(fb1 + 1));
}

TEST_F(RdlModelFixture, TestaddBodyTestBodyName)
{
    Body body(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));
    Joint joint(SpatialVector(0., 0., 1., 0., 0., 0.));

    model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body, "mybody");

    unsigned int body_id = model->GetBodyId("mybody");

    EXPECT_EQ(1u, body_id);
    EXPECT_EQ(std::numeric_limits<unsigned int>::max(), model->GetBodyId("unknownbody"));
}

TEST_F(RdlModelFixture, TestjcalcSimple)
{
    Body body(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));
    Joint joint(SpatialVector(0., 0., 1., 0., 0., 0.));

    model->addBody(0, Xtrans(Vector3d(1., 0., 0.)), joint, body);

    VectorNd Q = VectorNd::Zero(model->q_size);
    VectorNd QDot = VectorNd::Zero(model->q_size);

    QDot[0] = 1.;
    jcalc(*model, 1, Q, QDot);

    SpatialMatrix test_matrix(1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.,
                              1.);
    SpatialVector test_vector(0., 0., 1., 0., 0., 0.);
    SpatialVector test_joint_axis(0., 0., 1., 0., 0., 0.);

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(test_matrix, model->X_J[1].toMatrix(), unit_test_utils::E_MINUS_16));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(test_vector, model->v_J[1], unit_test_utils::E_MINUS_16));
    EXPECT_EQ(test_joint_axis, model->S[1]);

    Q[0] = M_PI * 0.5;
    QDot[0] = 1.;

    jcalc(*model, 1, Q, QDot);

    test_matrix.set(0., 1., 0., 0., 0., 0., -1., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., -1., 0., 0., 0., 0., 0., 0., 0., 1.);

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(test_matrix, model->X_J[1].toMatrix(), unit_test_utils::E_MINUS_16));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(test_vector, model->v_J[1], unit_test_utils::E_MINUS_16));
    EXPECT_EQ(test_joint_axis, model->S[1]);
}

TEST_F(RdlModelFixture, TestTransformBaseToLocal)
{
    Body body(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));

    unsigned int body_id = model->addBody(0, SpatialTransform(),
                                          Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                                                SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                                          body);

    VectorNd q = VectorNd::Zero(model->dof_count);
    VectorNd qdot = VectorNd::Zero(model->dof_count);
    VectorNd qddot = VectorNd::Zero(model->dof_count);
    VectorNd tau = VectorNd::Zero(model->dof_count);

    Vector3d base_coords(0., 0., 0.);
    Vector3d body_coords;
    Vector3d base_coords_back;

    updateKinematics(*model, q, qdot, qddot);
    FramePoint p_base_coords(model->bodyFrames[body_id], base_coords);
    p_base_coords.changeFrame(model->worldFrame);

    p_base_coords.changeFrame(model->bodyFrames[body_id]);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(base_coords, p_base_coords.vec(), unit_test_utils::E_MINUS_14));

    q[0] = 1.;
    q[1] = 0.2;
    q[2] = -2.3;
    q[3] = -2.3;
    q[4] = 0.03;
    q[5] = -0.23;

    updateKinematics(*model, q, qdot, qddot);

    p_base_coords.setIncludingFrame(base_coords, model->worldFrame);
    p_base_coords.changeFrame(model->bodyFrames[body_id]);
    p_base_coords.changeFrame(model->worldFrame);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(base_coords, p_base_coords.vec(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlModelFixture, Model1DoFJoint)
{
    // the standard modeling using a null body
    Body null_body;
    null_body.mIsVirtual = true;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    Joint joint_rot_z(JointType1DoF);
    joint_rot_z.mJointAxes[0] = SpatialVector(0., 0., 1., 0., 0., 0.);

    Model model_std;
    model_std.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

    model_std.addBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_rot_z, null_body);

    Model model_2;
    model_2.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

    Joint joint_rot_z_2(JointTypeRevoluteZ);
    model_2.addBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_rot_z_2, body, "body1");

    VectorNd Q = VectorNd::Zero(model_std.dof_count);
    VectorNd QDot = VectorNd::Zero(model_std.dof_count);
    VectorNd Tau = VectorNd::Zero(model_std.dof_count);

    VectorNd QDDot_2 = VectorNd::Zero(model_std.dof_count);
    VectorNd QDDot_std = VectorNd::Zero(model_std.dof_count);

    forwardDynamics(model_std, Q, QDot, Tau, QDDot_std);
    forwardDynamics(model_2, Q, QDot, Tau, QDDot_2);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_std, QDDot_2, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlModelFixture, Model2DoFJoint)
{
    // the standard modeling using a null body
    Body null_body;
    null_body.mIsVirtual = true;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));
    Joint joint_rot_x(SpatialVector(1., 0., 0., 0., 0., 0.));

    Model model_std;
    model_std.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

    model_std.addBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_rot_z, null_body);
    model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_x, body, "body1");

    // using a model with a 2 DoF joint
    Joint joint_rot_zx(SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.));

    Model model_2;
    model_2.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

    model_2.addBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_rot_zx, body, "body1");

    VectorNd Q = VectorNd::Zero(model_std.dof_count);
    VectorNd QDot = VectorNd::Zero(model_std.dof_count);
    VectorNd Tau = VectorNd::Zero(model_std.dof_count);

    VectorNd QDDot_2 = VectorNd::Zero(model_std.dof_count);
    VectorNd QDDot_std = VectorNd::Zero(model_std.dof_count);

    forwardDynamics(model_std, Q, QDot, Tau, QDDot_std);
    forwardDynamics(model_2, Q, QDot, Tau, QDDot_2);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_std, QDDot_2, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlModelFixture, Model3DoFJoint)
{
    // the standard modeling using a null body
    Body null_body;
    null_body.mIsVirtual = true;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));
    Joint joint_rot_y(SpatialVector(0., 1., 0., 0., 0., 0.));
    Joint joint_rot_x(SpatialVector(1., 0., 0., 0., 0., 0.));

    Model model_std;
    model_std.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

    unsigned int body_id;

    // in total we add two bodies to make sure that the transformations are
    // correct.
    model_std.addBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_rot_z, null_body);
    model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_y, null_body);
    body_id = model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_x, body, "body1");

    model_std.addBody(body_id, Xtrans(Vector3d(1., 0., 0.)), joint_rot_z, null_body);
    model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_y, null_body);
    body_id = model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_x, body, "body2");
    EXPECT_EQ(model_std.mass, 2.0);
    // using a model with a 3 DoF joint
    Joint joint_rot_zyx(SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.));

    Model model_2;
    model_2.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

    // in total we add two bodies to make sure that the transformations are
    // correct.
    body_id = model_2.addBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_rot_zyx, body);
    body_id = model_2.addBody(body_id, Xtrans(Vector3d(1., 0., 0.)), joint_rot_zyx, body);
    EXPECT_EQ(model_2.mass, 2.0);

    VectorNd Q = VectorNd::Zero(model_std.dof_count);
    VectorNd QDot = VectorNd::Zero(model_std.dof_count);
    VectorNd Tau = VectorNd::Zero(model_std.dof_count);

    VectorNd QDDot_2 = VectorNd::Zero(model_std.dof_count);
    VectorNd QDDot_std = VectorNd::Zero(model_std.dof_count);

    forwardDynamics(model_std, Q, QDot, Tau, QDDot_std);
    forwardDynamics(model_2, Q, QDot, Tau, QDDot_2);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_std, QDDot_2, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlModelFixture, Model4DoFJoint)
{
    // the standard modeling using a null body
    Body null_body;
    null_body.mIsVirtual = true;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));
    Joint joint_rot_y(SpatialVector(0., 1., 0., 0., 0., 0.));
    Joint joint_rot_x(SpatialVector(1., 0., 0., 0., 0., 0.));
    Joint joint_trans_x(SpatialVector(0., 0., 0., 1., 0., 0.));

    Model model_std;
    model_std.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

    unsigned int body_id;

    // in total we add two bodies to make sure that the transformations are
    // correct.
    model_std.addBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_rot_z, null_body);
    model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_y, null_body);
    model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_x, null_body);
    body_id = model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_trans_x, body, "body1");

    model_std.addBody(body_id, Xtrans(Vector3d(1., 0., 0.)), joint_rot_z, null_body);
    model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_y, null_body);
    model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_x, null_body);
    body_id = model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_trans_x, body, "body2");

    // using a model with a 2 DoF joint
    Joint joint_rot_zyx_tr_x(SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.),
                             SpatialVector(0., 0., 0., 1., 0., 0.));

    Model model_2;
    model_2.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

    // in total we add two bodies to make sure that the transformations are
    // correct.
    body_id = model_2.addBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_rot_zyx_tr_x, body);
    body_id = model_2.addBody(body_id, Xtrans(Vector3d(1., 0., 0.)), joint_rot_zyx_tr_x, body);

    VectorNd Q = VectorNd::Zero(model_std.dof_count);
    VectorNd QDot = VectorNd::Zero(model_std.dof_count);
    VectorNd Tau = VectorNd::Zero(model_std.dof_count);

    VectorNd QDDot_2 = VectorNd::Zero(model_std.dof_count);
    VectorNd QDDot_std = VectorNd::Zero(model_std.dof_count);

    forwardDynamics(model_std, Q, QDot, Tau, QDDot_std);
    forwardDynamics(model_2, Q, QDot, Tau, QDDot_2);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_std, QDDot_2, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlModelFixture, Model5DoFJoint)
{
    // the standard modeling using a null body
    Body null_body;
    null_body.mIsVirtual = true;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));
    Joint joint_rot_y(SpatialVector(0., 1., 0., 0., 0., 0.));
    Joint joint_rot_x(SpatialVector(1., 0., 0., 0., 0., 0.));
    Joint joint_trans_x(SpatialVector(0., 0., 0., 1., 0., 0.));
    Joint joint_trans_y(SpatialVector(0., 0., 0., 0., 1., 0.));

    Model model_std;
    model_std.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

    unsigned int body_id;

    // in total we add two bodies to make sure that the transformations are
    // correct.
    model_std.addBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_rot_z, null_body);
    model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_y, null_body);
    model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_x, null_body);
    model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_trans_x, null_body);
    body_id = model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_trans_y, body, "body1");

    model_std.addBody(body_id, Xtrans(Vector3d(1., 0., 0.)), joint_rot_z, null_body);
    model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_y, null_body);
    model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_x, null_body);
    model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_trans_x, null_body);
    body_id = model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_trans_y, body, "body2");

    // using a model with a 2 DoF joint
    Joint joint_rot_zyx_tr_xy(SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.),
                              SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.));

    Model model_2;
    model_2.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

    // in total we add two bodies to make sure that the transformations are
    // correct.
    body_id = model_2.addBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_rot_zyx_tr_xy, body);
    body_id = model_2.addBody(body_id, Xtrans(Vector3d(1., 0., 0.)), joint_rot_zyx_tr_xy, body);

    VectorNd Q = VectorNd::Zero(model_std.dof_count);
    VectorNd QDot = VectorNd::Zero(model_std.dof_count);
    VectorNd Tau = VectorNd::Zero(model_std.dof_count);

    VectorNd QDDot_2 = VectorNd::Zero(model_std.dof_count);
    VectorNd QDDot_std = VectorNd::Zero(model_std.dof_count);

    forwardDynamics(model_std, Q, QDot, Tau, QDDot_std);
    forwardDynamics(model_2, Q, QDot, Tau, QDDot_2);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_std, QDDot_2, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlModelFixture, Model6DoFJoint)
{
    // the standard modeling using a null body
    Body null_body;
    null_body.mIsVirtual = true;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));
    Joint joint_rot_y(SpatialVector(0., 1., 0., 0., 0., 0.));
    Joint joint_rot_x(SpatialVector(1., 0., 0., 0., 0., 0.));

    Model model_std;
    model_std.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

    unsigned int body_id;

    Joint joint_floating_base(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                              SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.));
    body_id = model_std.addBody(0, SpatialTransform(), joint_floating_base, body);

    model_std.addBody(body_id, Xtrans(Vector3d(1., 0., 0.)), joint_rot_z, null_body);
    model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_y, null_body);
    body_id = model_std.appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_x, body);

    // using a model with a 2 DoF joint
    Joint joint_rot_zyx(SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.));

    Model model_2;
    model_2.gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

    // in total we add two bodies to make sure that the transformations are
    // correct.
    body_id = model_2.addBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_floating_base, body);
    body_id = model_2.addBody(body_id, Xtrans(Vector3d(1., 0., 0.)), joint_rot_zyx, body);

    VectorNd Q = VectorNd::Zero(model_std.dof_count);
    VectorNd QDot = VectorNd::Zero(model_std.dof_count);
    VectorNd Tau = VectorNd::Zero(model_std.dof_count);

    VectorNd QDDot_2 = VectorNd::Zero(model_std.dof_count);
    VectorNd QDDot_std = VectorNd::Zero(model_std.dof_count);

    assert(model_std.q_size == model_2.q_size);

    forwardDynamics(model_std, Q, QDot, Tau, QDDot_std);
    forwardDynamics(model_2, Q, QDot, Tau, QDDot_2);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(QDDot_std, QDDot_2, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlModelFixture, ModelFixedJointQueryBodyId)
{
    // the standard modeling using a null body
    Body null_body;
    null_body.mIsVirtual = true;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Model model;

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));

    model.addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
    unsigned int fixed_body_id = model.appendBody(Xtrans(Vector3d(0., 1., 0.)), Joint(JointTypeFixed), fixed_body, "fixed_body");

    EXPECT_EQ(fixed_body_id, model.GetBodyId("fixed_body"));
}

/*
 * Makes sure that when appending a body to a fixed joint the parent of the
 * newly added parent is actually the moving body that the fixed body is
 * attached to.
 */
TEST_F(RdlModelFixture, ModelAppendToFixedBody)
{
    Body null_body;
    null_body.mIsVirtual = true;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Model model;

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));

    unsigned int movable_body = model.addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
    //	unsigned int fixed_body_id = model.appendBody (Xtrans(Vector3d(0., 1., 0.)), Joint(JointTypeFixed), fixed_body, "fixed_body");
    unsigned int appended_body_id = model.appendBody(Xtrans(Vector3d(0., 1., 0.)), joint_rot_z, body, "appended_body");

    EXPECT_EQ(movable_body + 1, appended_body_id);
    EXPECT_EQ(movable_body, model.lambda[appended_body_id]);
}

// Adds a fixed body to another fixed body.
TEST_F(RdlModelFixture, ModelAppendFixedToFixedBody)
{
    Body null_body;
    null_body.mIsVirtual = true;

    double movable_mass = 1.1;
    Vector3d movable_com(1., 0.4, 0.4);

    double fixed_mass = 1.2;
    Vector3d fixed_com(1.1, 0.5, 0.5);

    Vector3d fixed_displacement(0., 1., 0.);

    Body body(movable_mass, movable_com, Vector3d(1., 1., 1.));
    Body fixed_body(fixed_mass, fixed_com, Vector3d(1., 1., 1.));

    Model model;

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));

    unsigned int movable_body = model.addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
    unsigned int fixed_body_id = model.appendBody(Xtrans(fixed_displacement), Joint(JointTypeFixed), fixed_body, "fixed_body");
    unsigned int fixed_body_2_id = model.appendBody(Xtrans(fixed_displacement), Joint(JointTypeFixed), fixed_body, "fixed_body_2");
    unsigned int appended_body_id = model.appendBody(Xtrans(Vector3d(0., 1., 0.)), joint_rot_z, body, "appended_body");

    EXPECT_EQ(movable_body + 1, appended_body_id);
    EXPECT_EQ(movable_body, model.lambda[appended_body_id]);
    EXPECT_EQ(movable_mass + fixed_mass * 2., model.mBodies[movable_body].mMass);

    EXPECT_EQ(movable_body, model.mFixedBodies[fixed_body_id - model.fixed_body_discriminator].mMovableParent);
    EXPECT_EQ(movable_body, model.mFixedBodies[fixed_body_2_id - model.fixed_body_discriminator].mMovableParent);

    double new_mass = 3.5;
    Vector3d new_com =
        (1. / new_mass) * (movable_mass * movable_com + fixed_mass * (fixed_com + fixed_displacement) + fixed_mass * (fixed_com + fixed_displacement * 2.));

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(new_com, model.mBodies[movable_body].mCenterOfMass, unit_test_utils::E_MINUS_14));
}

// Ensures that the transformations of the movable parent and fixed joint
// frame is in proper order
TEST_F(RdlModelFixture, ModelFixedJointRotationOrderTranslationRotation)
{
    // the standard modeling using a null body
    Body null_body;
    null_body.mIsVirtual = true;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Model model;

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));

    SpatialTransform trans_x = Xtrans(Vector3d(1., 0., 0.));
    SpatialTransform rot_z = Xrotz(45. * M_PI / 180.);

    model.addBody(0, trans_x, joint_rot_z, body);
    model.appendBody(rot_z, Joint(JointTypeFixed), fixed_body, "fixed_body");
    unsigned int body_after_fixed = model.appendBody(trans_x, joint_rot_z, body);

    VectorNd Q(VectorNd::Zero(model.dof_count));
    Q[0] = 45 * M_PI / 180.;
    updateKinematicsCustom(model, &Q, nullptr, nullptr);
    VectorNd QDot = VectorNd::Zero(model.dof_count);
    VectorNd QDDot = VectorNd::Zero(model.dof_count);

    updateKinematics(model, Q, QDot, QDDot);
    FramePoint p(model.bodyFrames[body_after_fixed], Vector3d(0., 1., 0.));
    p.changeFrame(model.worldFrame);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 1., 0.), p.vec(), unit_test_utils::E_MINUS_14));
}

// Ensures that the transformations of the movable parent and fixed joint
// frame is in proper order
TEST_F(RdlModelFixture, ModelFixedJointRotationOrderRotationTranslation)
{
    // the standard modeling using a null body
    Body null_body;
    null_body.mIsVirtual = true;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Model model;

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));

    SpatialTransform rot_z = Xrotz(45. * M_PI / 180.);
    SpatialTransform trans_x = Xtrans(Vector3d(1., 0., 0.));

    model.addBody(0, rot_z, joint_rot_z, body);
    model.appendBody(trans_x, Joint(JointTypeFixed), fixed_body, "fixed_body");
    unsigned int body_after_fixed = model.appendBody(trans_x, joint_rot_z, body);

    VectorNd Q(VectorNd::Zero(model.dof_count));
    Q[0] = 45 * M_PI / 180.;
    updateKinematicsCustom(model, &Q, nullptr, nullptr);
    FramePoint p(model.bodyFrames[body_after_fixed], Vector3d(0., 1., 0.));
    p.changeFrame(model.worldFrame);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(-1., 2., 0.), p.vec(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlModelFixture, ModelGetBodyName)
{
    Body null_body;
    null_body.mIsVirtual = true;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Model model;

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));

    model.addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
    unsigned int fixed_body_id = model.appendBody(Xtrans(Vector3d(0., 1., 0.)), Joint(JointTypeFixed), fixed_body, "fixed_body");
    unsigned int appended_body_id = model.appendBody(Xtrans(Vector3d(0., 1., 0.)), joint_rot_z, body, "appended_body");

    EXPECT_EQ(string("fixed_body"), model.GetBodyName(fixed_body_id));
    EXPECT_EQ(string("appended_body"), model.GetBodyName(appended_body_id));
    EXPECT_EQ(string(""), model.GetBodyName(123));
}

TEST_F(RotZRotZYXFixed, ModelGetParentBodyId)
{
    EXPECT_EQ(0u, model->GetParentBodyId(0));
    EXPECT_EQ(0u, model->GetParentBodyId(body_a_id));
    EXPECT_EQ(body_a_id, model->GetParentBodyId(body_b_id));
}

TEST_F(RotZRotZYXFixed, ModelGetParentIdFixed)
{
    EXPECT_EQ(body_b_id, model->GetParentBodyId(body_fixed_id));
}

TEST_F(RotZRotZYXFixed, ModelGetJointFrame)
{
    SpatialTransform transform_a = model->GetJointFrame(body_a_id);
    SpatialTransform transform_b = model->GetJointFrame(body_b_id);
    SpatialTransform transform_root = model->GetJointFrame(0);

    EXPECT_TRUE(unit_test_utils::checkVector3dEq(fixture_transform_a.r, transform_a.r));
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(fixture_transform_b.r, transform_b.r));
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(Vector3d(0., 0., 0.), transform_root.r));
}

TEST_F(RotZRotZYXFixed, ModelGetJointFrameFixed)
{
    SpatialTransform transform_fixed = model->GetJointFrame(body_fixed_id);

    EXPECT_TRUE(unit_test_utils::checkVector3dEq(fixture_transform_fixed.r, transform_fixed.r));
}

TEST_F(RotZRotZYXFixed, ModelSetJointFrame)
{
    SpatialTransform new_transform_a = Xtrans(Vector3d(-1., -2., -3.));
    SpatialTransform new_transform_b = Xtrans(Vector3d(-4., -5., -6.));
    SpatialTransform new_transform_root = Xtrans(Vector3d(-99, -99., -99.));

    model->SetJointFrame(body_a_id, new_transform_a);
    model->SetJointFrame(body_b_id, new_transform_b);
    model->SetJointFrame(0, new_transform_root);

    SpatialTransform transform_a = model->GetJointFrame(body_a_id);
    SpatialTransform transform_b = model->GetJointFrame(body_b_id);
    SpatialTransform transform_root = model->GetJointFrame(0);

    EXPECT_TRUE(unit_test_utils::checkVector3dEq(new_transform_a.r, transform_a.r));
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(new_transform_b.r, transform_b.r));
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(Vector3d(0., 0., 0.), transform_root.r));

    try
    {
        model->SetJointFrame(model->fixed_body_discriminator + 1, new_transform_root);
        FAIL();
    }
    catch (RobotDynamics::RdlException& e)
    {
        EXPECT_STREQ(e.what(), "Error: setting of parent transform not supported for fixed bodies!");
    }
}

TEST_F(RdlModelFixture, BodyOrientationFixedJoint)
{
    Model model_fixed;
    Model model_movable;

    Body body(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));
    Joint joint_fixed(JointTypeFixed);
    Joint joint_rot_x(SpatialVector(1., 0., 0., 0., 0., 0.));

    model_fixed.appendBody(Xrotx(45 * M_PI / 180), joint_rot_x, body);
    unsigned int body_id_fixed = model_fixed.appendBody(Xroty(45 * M_PI / 180), joint_fixed, body);

    model_movable.appendBody(Xrotx(45 * M_PI / 180), joint_rot_x, body);
    unsigned int body_id_movable = model_movable.appendBody(Xroty(45 * M_PI / 180), joint_rot_x, body);

    VectorNd q_fixed(VectorNd::Zero(model_fixed.q_size));
    VectorNd q_movable(VectorNd::Zero(model_movable.q_size));

    updateKinematicsCustom(model_fixed, &q_fixed, nullptr, nullptr);
    updateKinematicsCustom(model_movable, &q_movable, nullptr, nullptr);

    EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(model_fixed.fixedBodyFrames[body_id_fixed - model->fixed_body_discriminator]->getInverseTransformToRoot().E,
                                                           model_movable.bodyFrames[body_id_movable]->getInverseTransformToRoot().E, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(model_fixed.fixedBodyFrames[body_id_fixed - model->fixed_body_discriminator]->getTransformToRoot().E,
                                                           model_movable.bodyFrames[body_id_movable]->getTransformToRoot().E, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlModelFixture, testAddingTwoBodiesWithSameNameThrows)
{
    Model model;

    Body body(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));
    Joint joint(JointTypeRevoluteX);

    unsigned int id = model.addBody(0, Xrotx(0.1), joint, body, "body_1");

    try
    {
        model.addBody(id, Xrotx(0.1), joint, body, "body_1");
        FAIL();
    }
    catch (RobotDynamics::RdlException& e)
    {
        EXPECT_STREQ("Error: Body with name 'body_1' already exists!", e.what());
    }
}

TEST_F(RdlModelFixture, testAddingTwoFixedBodiesWithSameNameThrows)
{
    Model model;

    Body body(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));
    Joint joint_fixed(JointTypeFixed);

    unsigned int fbid = model.addBody(0, Xrotx(0.1), joint_fixed, body, "body_1");

    try
    {
        model.addBody(fbid, Xrotx(0.1), joint_fixed, body, "body_1");
        FAIL();
    }
    catch (RobotDynamics::RdlException& e)
    {
        EXPECT_STREQ("Error: Fixed body with name 'body_1' already exists!", e.what());
    }
}

TEST_F(RdlModelFixture, transformsOnConstruction)
{
    Model model;

    Body body(1., Vector3d(1., 1., 1.), Vector3d(1., 1., 1.));
    Joint j(JointTypeRevoluteX);

    unsigned int id_1 = model.addBody(0, RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0., 0., 1.)), j, body, "b1");
    unsigned int id_2 = model.addBody(id_1, RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0., 1., 1.)), j, body, "b2");
    unsigned int id_3 = model.addBody(id_2, RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(1., 0., 1.)), j, body, "b3");

    EXPECT_TRUE(model.bodyFrames[id_1]->getTransformFromParent().toMatrix().isApprox(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0., 0., 1.)).toMatrix(),
                                                                                     unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(model.bodyFrames[id_2]->getTransformFromParent().toMatrix().isApprox(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0., 1., 1.)).toMatrix(),
                                                                                     unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(model.bodyFrames[id_3]->getTransformFromParent().toMatrix().isApprox(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(1., 0., 1.)).toMatrix(),
                                                                                     unit_test_utils::E_MINUS_14));

    EXPECT_TRUE(model.bodyFrames[id_3]->getTransformToRoot().toMatrix().isApprox(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(-1., -1., -3.)).toMatrix(),
                                                                                 unit_test_utils::E_MINUS_14));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
