// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>

#include "Fixtures.hpp"
#include "Human36Fixture.hpp"
#include "rdl_dynamics/Dynamics.hpp"
#include "rdl_dynamics/FramePoint.hpp"
#include "rdl_dynamics/Kinematics.hpp"
#include "rdl_dynamics/Model.hpp"
#include "UnitTestUtils.hpp"

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

struct RdlKinematicsFixture : public testing::Test
{
    void SetUp()
    {
        srand(time(NULL));
    }

    void TearDown()
    {
    }

    void randomizeStates()
    {
        for (int i = 0; i < q.rows(); i++)
        {
            q[i] = 0.4 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        }

        for (int i = 0; i < qdot.rows(); i++)
        {
            qdot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            tau[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            qddot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        }
    }

    RobotDynamics::Math::VectorNd q, qdot, qddot, tau;
};

struct RdlKinematicsSingleChainFixture : public testing::Test
{
    RdlKinematicsSingleChainFixture()
    {
        srand(time(NULL));
    }

    void SetUp()
    {
        model = new Model;

        /* Basically a model like this, where X are the Center of Masses
         * and the CoM of the last (3rd) body comes out of the Y=X=0 plane.
         *
         *                X
         *                *
         *              _/
         *            _/  (-Z)
         *      Z    /
         *      *---*
         *      |
         *      |
         *  Z   |
         *  O---*
         *      Y
         */

        body_a = Body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
        joint_a = Joint(SpatialVector(0., 0., 1., 0., 0., 0.));

        body_a_id = model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a, "body_a");

        body_b = Body(1., Vector3d(0., 1., 0.), Vector3d(1., 1., 1.));
        joint_b = Joint(SpatialVector(0., 1., 0., 0., 0., 0.));

        body_b_id = model->addBody(body_a_id, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b, "body_b");

        body_c = Body(1., Vector3d(0., 0., 1.), Vector3d(1., 1., 1.));
        joint_c = Joint(SpatialVector(0., 0., 1., 0., 0., 0.));

        body_c_id = model->addBody(body_b_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c, "body_c");

        body_d = Body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
        joint_d = Joint(SpatialVector(1., 0., 0., 0., 0., 0.));

        body_d_id = model->addBody(body_c_id, Xtrans(Vector3d(0., 0., -1.)), joint_d, body_d, "body_d");

        Q = VectorNd::Constant((size_t)model->dof_count, 0.);
        QDot = VectorNd::Constant((size_t)model->dof_count, 0.);
        QDDot = VectorNd::Constant((size_t)model->dof_count, 0.);
        Tau = VectorNd::Constant((size_t)model->dof_count, 0.);
    }

    void randomizeStates()
    {
        for (int i = 0; i < Q.size(); i++)
        {
            Q[i] = 0.4 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            QDot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            Tau[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            QDDot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        }
    }

    void TearDown()
    {
        EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(*model));
        delete model;
    }

    Model* model;

    unsigned int body_a_id, body_b_id, body_c_id, body_d_id;
    Body body_a, body_b, body_c, body_d;
    Joint joint_a, joint_b, joint_c, joint_d;

    VectorNd Q;
    VectorNd QDot;
    VectorNd QDDot;
    VectorNd Tau;
};

struct RdlKinematicsSingleChainFixture6DoF : public testing::Test
{
    RdlKinematicsSingleChainFixture6DoF()
    {
    }

    void SetUp()
    {
        model = new Model;

        model->gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

        /*
         *
         *          X Contact point (ref child)
         *          |
         *    Base  |
         *   / body |
         *  O-------*
         *           \
         *             Child body
         */

        // base body (3 DoF)
        base = Body(1., Vector3d(0.5, 0., 0.), Vector3d(1., 1., 1.));
        joint_rotzyx = Joint(SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.));
        base_id = model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rotzyx, base);

        // child body (3 DoF)
        child = Body(1., Vector3d(0., 0.5, 0.), Vector3d(1., 1., 1.));
        child_id = model->addBody(base_id, Xtrans(Vector3d(1., 0., 0.)), joint_rotzyx, child);

        Q = VectorNd::Constant(model->mBodies.size() - 1, 0.);
        QDot = VectorNd::Constant(model->mBodies.size() - 1, 0.);
        QDDot = VectorNd::Constant(model->mBodies.size() - 1, 0.);
        Tau = VectorNd::Constant(model->mBodies.size() - 1, 0.);
    }

    void TearDown()
    {
        EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(*model));
        delete model;
    }

    Model* model;

    unsigned int base_id, child_id;
    Body base, child;
    Joint joint_rotzyx;

    VectorNd Q;
    VectorNd QDot;
    VectorNd QDDot;
    VectorNd Tau;
};

TEST_F(RdlKinematicsFixture, parallelKinematics)
{
    RobotDynamics::ModelPtr model_parallel(new RobotDynamics::Model());
    RobotDynamics::ModelPtr model(new RobotDynamics::Model());
    RobotDynamics::Body b(1., Vector3d(0.5, 0., 0.), Vector3d(1., 1., 1.));
    RobotDynamics::Joint j(RobotDynamics::JointTypeRevoluteX);

    model_parallel->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b1");
    model_parallel->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b2");
    model_parallel->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b3");
    model_parallel->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b4");

    unsigned int b5_id_p = model_parallel->addBody(0, RobotDynamics::Math::SpatialTransform(), j, b, "b5");
    unsigned int b6_id_p = model_parallel->addBody(b5_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b6");
    unsigned int b7_id_p = model_parallel->addBody(b6_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b7");
    model_parallel->addBody(b7_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b8");

    // // Mixing these up intentionally
    unsigned int b9_id_p = model_parallel->addBody(b5_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b9");
    unsigned int b13_id_p = model_parallel->addBody(b5_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b13");

    unsigned int b10_id_p = model_parallel->addBody(b9_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b10");
    unsigned int b14_id_p = model_parallel->addBody(b13_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b14");

    unsigned int b11_id_p = model_parallel->addBody(b10_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b11");
    unsigned int b15_id_p = model_parallel->addBody(b14_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b15");

    model_parallel->addBody(b11_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b12");
    model_parallel->addBody(b15_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b16");

    model->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b1");
    model->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b2");
    model->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b3");
    model->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b4");

    unsigned int b5_id = model->addBody(0, RobotDynamics::Math::SpatialTransform(), j, b, "b5");
    unsigned int b6_id = model->addBody(b5_id, RobotDynamics::Math::SpatialTransform(), j, b, "b6");
    unsigned int b7_id = model->addBody(b6_id, RobotDynamics::Math::SpatialTransform(), j, b, "b7");
    model->addBody(b7_id, RobotDynamics::Math::SpatialTransform(), j, b, "b8");

    // // Mixing these up intentionally
    unsigned int b9_id = model->addBody(b5_id, RobotDynamics::Math::SpatialTransform(), j, b, "b9");
    unsigned int b13_id = model->addBody(b5_id, RobotDynamics::Math::SpatialTransform(), j, b, "b13");

    unsigned int b10_id = model->addBody(b9_id, RobotDynamics::Math::SpatialTransform(), j, b, "b10");
    unsigned int b14_id = model->addBody(b13_id, RobotDynamics::Math::SpatialTransform(), j, b, "b14");

    unsigned int b11_id = model->addBody(b10_id, RobotDynamics::Math::SpatialTransform(), j, b, "b11");
    unsigned int b15_id = model->addBody(b14_id, RobotDynamics::Math::SpatialTransform(), j, b, "b15");

    model->addBody(b11_id, RobotDynamics::Math::SpatialTransform(), j, b, "b12");
    model->addBody(b15_id, RobotDynamics::Math::SpatialTransform(), j, b, "b16");
    q = RobotDynamics::Math::VectorNd::Zero(model->q_size);
    qdot = RobotDynamics::Math::VectorNd::Zero(model->qdot_size);
    qddot = RobotDynamics::Math::VectorNd::Zero(model->qdot_size);
    tau = RobotDynamics::Math::VectorNd::Zero(model->qdot_size);
    randomizeStates();

    updateKinematics(*model, q, qdot, qddot);
    updateKinematicsParallel(*model_parallel, q, qdot, qddot);

    for (std::pair<std::string, RobotDynamics::ReferenceFramePtr> pair : model->referenceFrameMap)
    {
        EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(pair.second->getTransformToRoot().E, model_parallel->referenceFrameMap[pair.first]->getTransformToRoot().E,
                                                               1.e-12));
        EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(pair.second->getTransformToRoot().r, model_parallel->referenceFrameMap[pair.first]->getTransformToRoot().r,
                                                               1.e-12));
    }

    for (unsigned int i = 0; i < model->v.size(); ++i)
    {
        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model->v[i], model_parallel->v[i], 1.e-12));
        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model->a[i], model_parallel->a[i], 1.e-12));
    }
}

TEST_F(RdlKinematicsFixture, parallelKinematicsCustom)
{
    RobotDynamics::ModelPtr model_parallel(new RobotDynamics::Model());
    RobotDynamics::ModelPtr model(new RobotDynamics::Model());
    RobotDynamics::Body b(1., Vector3d(0.5, 0., 0.), Vector3d(1., 1., 1.));
    RobotDynamics::Joint j(RobotDynamics::JointTypeRevoluteX);

    model_parallel->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b1");
    model_parallel->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b2");
    model_parallel->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b3");
    model_parallel->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b4");

    unsigned int b5_id_p = model_parallel->addBody(0, RobotDynamics::Math::SpatialTransform(), j, b, "b5");
    unsigned int b6_id_p = model_parallel->addBody(b5_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b6");
    unsigned int b7_id_p = model_parallel->addBody(b6_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b7");
    model_parallel->addBody(b7_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b8");

    // // Mixing these up intentionally
    unsigned int b9_id_p = model_parallel->addBody(b5_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b9");
    unsigned int b13_id_p = model_parallel->addBody(b5_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b13");

    unsigned int b10_id_p = model_parallel->addBody(b9_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b10");
    unsigned int b14_id_p = model_parallel->addBody(b13_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b14");

    unsigned int b11_id_p = model_parallel->addBody(b10_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b11");
    unsigned int b15_id_p = model_parallel->addBody(b14_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b15");

    model_parallel->addBody(b11_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b12");
    model_parallel->addBody(b15_id_p, RobotDynamics::Math::SpatialTransform(), j, b, "b16");

    model->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b1");
    model->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b2");
    model->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b3");
    model->appendBody(RobotDynamics::Math::SpatialTransform(), j, b, "b4");

    unsigned int b5_id = model->addBody(0, RobotDynamics::Math::SpatialTransform(), j, b, "b5");
    unsigned int b6_id = model->addBody(b5_id, RobotDynamics::Math::SpatialTransform(), j, b, "b6");
    unsigned int b7_id = model->addBody(b6_id, RobotDynamics::Math::SpatialTransform(), j, b, "b7");
    model->addBody(b7_id, RobotDynamics::Math::SpatialTransform(), j, b, "b8");

    // // Mixing these up intentionally
    unsigned int b9_id = model->addBody(b5_id, RobotDynamics::Math::SpatialTransform(), j, b, "b9");
    unsigned int b13_id = model->addBody(b5_id, RobotDynamics::Math::SpatialTransform(), j, b, "b13");

    unsigned int b10_id = model->addBody(b9_id, RobotDynamics::Math::SpatialTransform(), j, b, "b10");
    unsigned int b14_id = model->addBody(b13_id, RobotDynamics::Math::SpatialTransform(), j, b, "b14");

    unsigned int b11_id = model->addBody(b10_id, RobotDynamics::Math::SpatialTransform(), j, b, "b11");
    unsigned int b15_id = model->addBody(b14_id, RobotDynamics::Math::SpatialTransform(), j, b, "b15");

    model->addBody(b11_id, RobotDynamics::Math::SpatialTransform(), j, b, "b12");
    model->addBody(b15_id, RobotDynamics::Math::SpatialTransform(), j, b, "b16");
    q = RobotDynamics::Math::VectorNd::Zero(model->q_size);
    qdot = RobotDynamics::Math::VectorNd::Zero(model->qdot_size);
    qddot = RobotDynamics::Math::VectorNd::Zero(model->qdot_size);
    tau = RobotDynamics::Math::VectorNd::Zero(model->qdot_size);
    randomizeStates();

    updateKinematicsCustom(*model, &q, nullptr, nullptr);
    updateKinematicsCustomParallel(*model_parallel, &q, nullptr, nullptr);

    for (std::pair<std::string, RobotDynamics::ReferenceFramePtr> pair : model->referenceFrameMap)
    {
        EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(pair.second->getTransformToRoot().E, model_parallel->referenceFrameMap[pair.first]->getTransformToRoot().E,
                                                               1.e-12));
        EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(pair.second->getTransformToRoot().r, model_parallel->referenceFrameMap[pair.first]->getTransformToRoot().r,
                                                               1.e-12));
    }

    for (unsigned int i = 0; i < model->v.size(); ++i)
    {
        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model->v[i], model_parallel->v[i], 1.e-12));
        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model->a[i], model_parallel->a[i], 1.e-12));
    }

    updateKinematicsCustom(*model, &q, &qdot, nullptr);
    updateKinematicsCustomParallel(*model_parallel, &q, &qdot, nullptr);

    for (std::pair<std::string, RobotDynamics::ReferenceFramePtr> pair : model->referenceFrameMap)
    {
        EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(pair.second->getTransformToRoot().E, model_parallel->referenceFrameMap[pair.first]->getTransformToRoot().E,
                                                               1.e-12));
        EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(pair.second->getTransformToRoot().r, model_parallel->referenceFrameMap[pair.first]->getTransformToRoot().r,
                                                               1.e-12));
    }

    for (unsigned int i = 0; i < model->v.size(); ++i)
    {
        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model->v[i], model_parallel->v[i], 1.e-12));
        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model->a[i], model_parallel->a[i], 1.e-12));
    }

    updateKinematicsCustom(*model, &q, &qdot, &qddot);
    updateKinematicsCustomParallel(*model_parallel, &q, &qdot, &qddot);

    for (std::pair<std::string, RobotDynamics::ReferenceFramePtr> pair : model->referenceFrameMap)
    {
        EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(pair.second->getTransformToRoot().E, model_parallel->referenceFrameMap[pair.first]->getTransformToRoot().E,
                                                               1.e-12));
        EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(pair.second->getTransformToRoot().r, model_parallel->referenceFrameMap[pair.first]->getTransformToRoot().r,
                                                               1.e-12));
    }

    for (unsigned int i = 0; i < model->v.size(); ++i)
    {
        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model->v[i], model_parallel->v[i], 1.e-12));
        EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model->a[i], model_parallel->a[i], 1.e-12));
    }
}

TEST_F(RdlKinematicsSingleChainFixture, TestPositionNeutral)
{
    updateKinematics(*model, Q, QDot, QDDot);
    FramePoint p_a(model->bodyFrames[1], 0., 0., 0.);
    FramePoint p_b(model->bodyFrames[2], 0., 0., 0.);
    FramePoint p_c(model->bodyFrames[3], 0., 0., 0.);
    FramePoint p_d(model->bodyFrames[4], 0., 0., 0.);
    p_a.changeFrame(ReferenceFrame::getWorldFrame());
    p_b.changeFrame(ReferenceFrame::getWorldFrame());
    p_c.changeFrame(ReferenceFrame::getWorldFrame());
    p_d.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3dZero, p_a.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 0., 0.), p_b.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 1., 0.), p_c.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 1., -1.), p_d.vec(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlKinematicsSingleChainFixture, TestPositionBaseRotated90Deg)
{
    Q[0] = M_PI_2;

    updateKinematics(*model, Q, QDot, QDDot);
    FramePoint p_a(model->bodyFrames[1], 0., 0., 0.);
    FramePoint p_b(model->bodyFrames[2], 0., 0., 0.);
    FramePoint p_c(model->bodyFrames[3], 0., 0., 0.);
    FramePoint p_d(model->bodyFrames[4], 0., 0., 0.);
    p_a.changeFrame(ReferenceFrame::getWorldFrame());
    p_b.changeFrame(ReferenceFrame::getWorldFrame());
    p_c.changeFrame(ReferenceFrame::getWorldFrame());
    p_d.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3dZero, p_a.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 1., 0.), p_b.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(-1., 1., 0.), p_c.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(-1., 1., -1.), p_d.vec(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlKinematicsSingleChainFixture, TestPositionBaseRotatedNeg45Deg)
{
    Q[0] = -0.25 * M_PI;

    updateKinematics(*model, Q, QDot, QDDot);
    FramePoint p_a(model->bodyFrames[1], 0., 0., 0.);
    FramePoint p_b(model->bodyFrames[2], 0., 0., 0.);
    FramePoint p_c(model->bodyFrames[3], 0., 0., 0.);
    FramePoint p_d(model->bodyFrames[4], 0., 0., 0.);
    p_a.changeFrame(ReferenceFrame::getWorldFrame());
    p_b.changeFrame(ReferenceFrame::getWorldFrame());
    p_c.changeFrame(ReferenceFrame::getWorldFrame());
    p_d.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3dZero, p_a.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0.707106781186547, -0.707106781186547, 0.), p_b.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(sqrt(2.0), 0., 0.), p_c.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(sqrt(2.0), 0., -1.), p_d.vec(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlKinematicsSingleChainFixture, TestPositionBodyBRotated90Deg)
{
    Q[1] = 0.5 * M_PI;

    updateKinematics(*model, Q, QDot, QDDot);
    FramePoint p_a(model->bodyFrames[1], 0., 0., 0.);
    FramePoint p_b(model->bodyFrames[2], 0., 0., 0.);
    FramePoint p_c(model->bodyFrames[3], 0., 0., 0.);
    FramePoint p_d(model->bodyFrames[4], 0., 0., 0.);
    p_a.changeFrame(ReferenceFrame::getWorldFrame());
    p_b.changeFrame(ReferenceFrame::getWorldFrame());
    p_c.changeFrame(ReferenceFrame::getWorldFrame());
    p_d.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3dZero, p_a.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 0., 0.), p_b.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 1., 0.), p_c.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 1., 0.), p_d.vec(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlKinematicsSingleChainFixture, TestPositionBodyBRotatedNeg45Deg)
{
    // We call ForwardDynamics() as it updates the spatial transformation
    // matrices
    Q[1] = -0.25 * M_PI;

    updateKinematics(*model, Q, QDot, QDDot);
    FramePoint p_a(model->bodyFrames[1], 0., 0., 0.);
    FramePoint p_b(model->bodyFrames[2], 0., 0., 0.);
    FramePoint p_c(model->bodyFrames[3], 0., 0., 0.);
    FramePoint p_d(model->bodyFrames[4], 0., 0., 0.);
    p_a.changeFrame(ReferenceFrame::getWorldFrame());
    p_b.changeFrame(ReferenceFrame::getWorldFrame());
    p_c.changeFrame(ReferenceFrame::getWorldFrame());
    p_d.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3dZero, p_a.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 0., 0.), p_b.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 1., 0.), p_c.vec(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1 + 0.707106781186547, 1., -0.707106781186547), p_d.vec(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlKinematicsSingleChainFixture, TestCalcBodyToBaseCoordinates)
{
    updateKinematics(*model, Q, QDot, QDDot);
    FramePoint p_c(model->bodyFrames[3], 0., 1., 0.);
    p_c.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 2., 0.), p_c.vec(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlKinematicsSingleChainFixture, TestCalcBodyToBaseCoordinatesRotated)
{
    Q[2] = 0.5 * M_PI;

    updateKinematics(*model, Q, QDot, QDDot);
    FramePoint p_c(model->bodyFrames[3], 0., 0., 0.);
    p_c.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 1., 0.), p_c.vec(), unit_test_utils::E_MINUS_14));

    p_c.setIncludingFrame(0., 1., 0., model->bodyFrames[3]);
    p_c.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 1., 0.), p_c.vec(), unit_test_utils::E_MINUS_14));

    // Rotate the other way round
    Q[2] = -0.5 * M_PI;

    updateKinematics(*model, Q, QDot, QDDot);
    p_c.setIncludingFrame(0., 0., 0., model->bodyFrames[3]);
    p_c.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 1., 0.), p_c.vec(), unit_test_utils::E_MINUS_14));

    p_c.setIncludingFrame(0., 1., 0., model->bodyFrames[3]);
    p_c.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(2., 1., 0.), p_c.vec(), unit_test_utils::E_MINUS_14));

    // Rotate around the base
    Q[0] = 0.5 * M_PI;
    Q[2] = 0.;

    updateKinematics(*model, Q, QDot, QDDot);
    p_c.setIncludingFrame(0., 0., 0., model->bodyFrames[3]);
    p_c.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(-1., 1., 0.), p_c.vec(), unit_test_utils::E_MINUS_14));

    p_c.setIncludingFrame(0., 1., 0., model->bodyFrames[3]);
    p_c.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(-2., 1., 0.), p_c.vec(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlKinematicsSingleChainFixture, calcRelativeBodySpatialJacobian)
{
    randomizeStates();
    ReferenceFramePtr body_a_frame = model->bodyFrames[body_a_id];
    ReferenceFramePtr body_b_frame = model->bodyFrames[body_b_id];
    ReferenceFramePtr body_c_frame = model->bodyFrames[body_c_id];

    SpatialMotion v = calcSpatialVelocity(*model, Q, QDot, body_a_id, body_c_id);
    v.changeFrame(body_b_frame);

    MatrixNd G(6, model->qdot_size);
    G.setZero();

    calcRelativeBodySpatialJacobian(*model, Q, G, body_a_frame, body_c_frame, body_b_frame);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(v, G * QDot, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlKinematicsSingleChainFixture, calcPointJacobianFrames)
{
    randomizeStates();
    ReferenceFramePtr body_a_frame = model->bodyFrames[body_a_id];
    ReferenceFramePtr body_b_frame = model->bodyFrames[body_b_id];
    ReferenceFramePtr body_c_frame = model->bodyFrames[body_c_id];

    MatrixNd G3d(3, model->qdot_size), G3dCheck(3, model->qdot_size);
    G3d.setZero();
    G3dCheck.setZero();
    calcPointJacobian(*model, Q, G3d, body_c_frame);
    calcPointJacobian(*model, Q, body_b_id, body_c_frame->getTransformFromParent().r, G3dCheck);

    EXPECT_TRUE(G3d.isApprox(G3dCheck, unit_test_utils::E_MINUS_14));

    randomizeStates();

    unsigned int fb_id = model->addBody(body_c_id, RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.1, -0.2, 0.3)),
                                        RobotDynamics::Joint(RobotDynamics::JointType::JointTypeFixed), body_c, "fb1");

    RobotDynamics::ReferenceFramePtr fb_frame = model->getReferenceFrame("fb1");

    G3d.setZero();
    G3dCheck.setZero();
    calcPointJacobian(*model, Q, G3d, fb_frame);
    calcPointJacobian(*model, Q, body_c_id, RobotDynamics::Math::Vector3d(0.1, -0.2, 0.3), G3dCheck);

    EXPECT_TRUE(G3d.isApprox(G3dCheck, unit_test_utils::E_MINUS_14));

    model->addBody(fb_id, RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.2, -0.3, 0.4)), RobotDynamics::Joint(RobotDynamics::JointType::JointTypeFixed),
                   body_c, "fb2");

    fb_frame = model->getReferenceFrame("fb2");

    G3d.setZero();
    G3dCheck.setZero();
    calcPointJacobian(*model, Q, G3d, fb_frame);
    calcPointJacobian(*model, Q, fb_id, RobotDynamics::Math::Vector3d(0.2, -0.3, 0.4), G3dCheck);

    EXPECT_TRUE(G3d.isApprox(G3dCheck, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlKinematicsSingleChainFixture, calcPointJacobian6DFrames)
{
    randomizeStates();
    updateKinematicsCustom(*model, &Q, &QDot, nullptr);
    ReferenceFramePtr body_a_frame = model->bodyFrames[body_a_id];
    ReferenceFramePtr body_b_frame = model->bodyFrames[body_b_id];
    ReferenceFramePtr body_c_frame = model->bodyFrames[body_c_id];

    MatrixNd G(6, model->qdot_size), GCheck(6, model->qdot_size);
    G.setZero();
    GCheck.setZero();
    calcPointJacobian6D(*model, Q, G, body_c_frame, false);
    calcPointJacobian6D(*model, Q, body_c_id, RobotDynamics::Math::Vector3dZero, GCheck, false);

    EXPECT_TRUE(G.isApprox(GCheck, unit_test_utils::E_MINUS_14));
    RobotDynamics::Math::SpatialVector v = G * QDot;
    RobotDynamics::Math::FrameVectorPair p = calcPointVelocity6D(*model, Q, QDot, body_c_frame, false);
    EXPECT_TRUE(v.getLinearPart().isApprox(p.linear(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(v.getAngularPart().isApprox(p.angular(), unit_test_utils::E_MINUS_12));

    randomizeStates();
    updateKinematicsCustom(*model, &Q, &QDot, nullptr);

    unsigned int fb_id = model->addBody(body_c_id, RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.1, -0.2, 0.3)),
                                        RobotDynamics::Joint(RobotDynamics::JointType::JointTypeFixed), body_c, "fb1");

    RobotDynamics::ReferenceFramePtr fb_frame = model->getReferenceFrame("fb1");

    G.setZero();
    GCheck.setZero();
    calcPointJacobian6D(*model, Q, G, fb_frame, false);
    calcPointJacobian6D(*model, Q, fb_id, RobotDynamics::Math::Vector3dZero, GCheck, false);

    EXPECT_TRUE(G.isApprox(GCheck, unit_test_utils::E_MINUS_14));
    p = calcPointVelocity6D(*model, Q, QDot, fb_frame, false);
    v = G * QDot;
    EXPECT_TRUE(v.getLinearPart().isApprox(p.linear(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(v.getAngularPart().isApprox(p.angular(), unit_test_utils::E_MINUS_12));

    fb_id = model->addBody(fb_id, RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.2, -0.3, 0.4)),
                           RobotDynamics::Joint(RobotDynamics::JointType::JointTypeFixed), body_c, "fb2");

    fb_frame = model->getReferenceFrame("fb2");

    G.setZero();
    GCheck.setZero();
    calcPointJacobian6D(*model, Q, G, fb_frame, false);
    calcPointJacobian6D(*model, Q, fb_id, RobotDynamics::Math::Vector3dZero, GCheck, false);

    EXPECT_TRUE(G.isApprox(GCheck, unit_test_utils::E_MINUS_14));
    p = calcPointVelocity6D(*model, Q, QDot, fb_frame, false);
    v = G * QDot;
    EXPECT_TRUE(v.getLinearPart().isApprox(p.linear(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(v.getAngularPart().isApprox(p.angular(), unit_test_utils::E_MINUS_12));
}

TEST_F(RdlKinematicsSingleChainFixture, calcRelativePointJacobian6DFrames)
{
    randomizeStates();
    updateKinematicsCustom(*model, &Q, &QDot, nullptr);
    ReferenceFramePtr body_a_frame = model->bodyFrames[body_a_id];
    ReferenceFramePtr body_b_frame = model->bodyFrames[body_b_id];
    ReferenceFramePtr body_c_frame = model->bodyFrames[body_c_id];

    MatrixNd G(6, model->qdot_size);
    G.setZero();
    calcRelativePointJacobian6D(*model, Q, G, body_c_frame, body_b_frame, model->worldFrame, false);

    calcPointVelocity6D(*model, Q, QDot, body_c_frame, false);
    calcPointVelocity6D(*model, Q, QDot, body_b_frame, false);

    FrameVectorPair p = calcPointVelocity6D(*model, Q, QDot, body_c_frame, false) - calcPointVelocity6D(*model, Q, QDot, body_b_frame, false);
    SpatialVector v = G * QDot;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v.getLinearPart(), p.linear(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v.getAngularPart(), p.angular(), unit_test_utils::E_MINUS_12));

    G.setZero();

    calcRelativePointJacobian6D(*model, Q, G, body_c_frame, body_b_frame, body_a_frame, false);
    p = calcPointVelocity6D(*model, Q, QDot, body_c_frame, false) - calcPointVelocity6D(*model, Q, QDot, body_b_frame, false);
    p.changeFrame(body_a_frame);
    v = G * QDot;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v.getLinearPart(), p.linear(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v.getAngularPart(), p.angular(), unit_test_utils::E_MINUS_12));

    G.setZero();
    calcRelativePointJacobian6D(*model, Q, G, body_b_frame, body_c_frame, model->worldFrame, false);
    p = calcPointVelocity6D(*model, Q, QDot, body_b_frame, false) - calcPointVelocity6D(*model, Q, QDot, body_c_frame, false);
    v = G * QDot;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v.getLinearPart(), p.linear(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v.getAngularPart(), p.angular(), unit_test_utils::E_MINUS_12));

    unsigned int fb_id = model->addBody(body_c_id, RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.1, -0.2, 0.3)),
                                        RobotDynamics::Joint(RobotDynamics::JointType::JointTypeFixed), body_c, "fb1");

    RobotDynamics::ReferenceFramePtr fb_frame = model->getReferenceFrame("fb1");

    G.setZero();
    calcRelativePointJacobian6D(*model, Q, G, body_b_frame, fb_frame, model->worldFrame, false);
    p = calcPointVelocity6D(*model, Q, QDot, body_b_frame, false) - calcPointVelocity6D(*model, Q, QDot, fb_frame, false);
    v = G * QDot;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v.getLinearPart(), p.linear(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v.getAngularPart(), p.angular(), unit_test_utils::E_MINUS_12));

    fb_id = model->addBody(fb_id, RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.2, -0.3, 0.4)),
                           RobotDynamics::Joint(RobotDynamics::JointType::JointTypeFixed), body_c, "fb2");

    RobotDynamics::ReferenceFramePtr fb_frame2 = model->getReferenceFrame("fb2");

    G.setZero();
    calcRelativePointJacobian6D(*model, Q, G, fb_frame, fb_frame2, model->worldFrame, false);
    p = calcPointVelocity6D(*model, Q, QDot, fb_frame, false) - calcPointVelocity6D(*model, Q, QDot, fb_frame2, false);
    v = G * QDot;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v.getLinearPart(), p.linear(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v.getAngularPart(), p.angular(), unit_test_utils::E_MINUS_12));
}

TEST_F(RdlKinematicsSingleChainFixture, calcPointJacobianDotFrames)
{
    randomizeStates();
    ReferenceFramePtr body_a_frame = model->bodyFrames[body_a_id];
    ReferenceFramePtr body_b_frame = model->bodyFrames[body_b_id];
    ReferenceFramePtr body_c_frame = model->bodyFrames[body_c_id];

    MatrixNd G3d(3, model->qdot_size), G3dCheck(3, model->qdot_size);
    G3d.setZero();
    G3dCheck.setZero();
    calcPointJacobianDot(*model, Q, QDot, body_c_frame, G3d);
    calcPointJacobianDot(*model, Q, QDot, body_c_id, RobotDynamics::Math::Vector3dZero, G3dCheck);

    EXPECT_TRUE(G3d.isApprox(G3dCheck, unit_test_utils::E_MINUS_14));

    randomizeStates();

    unsigned int fb_id = model->addBody(body_c_id, RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.1, -0.2, 0.3)),
                                        RobotDynamics::Joint(RobotDynamics::JointType::JointTypeFixed), body_c, "fb1");

    RobotDynamics::ReferenceFramePtr fb_frame = model->getReferenceFrame("fb1");

    G3d.setZero();
    G3dCheck.setZero();
    calcPointJacobianDot(*model, Q, QDot, fb_frame, G3d);
    calcPointJacobianDot(*model, Q, QDot, fb_id, RobotDynamics::Math::Vector3dZero, G3dCheck);

    EXPECT_TRUE(G3d.isApprox(G3dCheck, unit_test_utils::E_MINUS_14));

    unsigned int fb_id2 = model->addBody(fb_id, RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.2, -0.3, 0.4)),
                                         RobotDynamics::Joint(RobotDynamics::JointType::JointTypeFixed), body_c, "fb2");

    fb_frame = model->getReferenceFrame("fb2");

    G3d.setZero();
    G3dCheck.setZero();
    calcPointJacobianDot(*model, Q, QDot, fb_frame, G3d);
    calcPointJacobianDot(*model, Q, QDot, fb_id2, RobotDynamics::Math::Vector3dZero, G3dCheck);

    EXPECT_TRUE(G3d.isApprox(G3dCheck, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlKinematicsSingleChainFixture, calcRelativeBodySpatialJacobianDot)
{
    randomizeStates();
    ReferenceFramePtr body_a_frame = model->bodyFrames[body_a_id];
    ReferenceFramePtr body_b_frame = model->bodyFrames[body_b_id];
    ReferenceFramePtr body_c_frame = model->bodyFrames[body_c_id];

    MatrixNd GDot(6, model->qdot_size);
    GDot.setZero();
    MatrixNd G(6, model->qdot_size);
    G.setZero();

    updateKinematics(*model, Q, QDot, QDDot);

    ReferenceFramePtr f1 = body_a_frame;
    ReferenceFramePtr f2 = body_b_frame;
    ReferenceFramePtr f3 = body_c_frame;
    calcRelativeBodySpatialJacobianDot(*model, Q, QDot, GDot, f3, f1, f2);
    calcRelativeBodySpatialJacobian(*model, Q, G, f3, f1, f2);

    SpatialAcceleration a_calc = calcSpatialAcceleration(*model, Q, QDot, QDDot, f3, f1, f2);
    EXPECT_TRUE(a_calc.isApprox(GDot * QDot + G * QDDot, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlKinematicsSingleChainFixture, calcRelativeBodySpatialJacobianAndJacobianDot)
{
    randomizeStates();
    ReferenceFramePtr body_a_frame = model->bodyFrames[body_a_id];
    ReferenceFramePtr body_b_frame = model->bodyFrames[body_b_id];
    ReferenceFramePtr body_c_frame = model->bodyFrames[body_c_id];

    MatrixNd GDot(6, model->qdot_size), G(6, model->qdot_size), GDot2(6, model->qdot_size), G2(6, model->qdot_size);
    G.setZero();
    G2.setZero();
    GDot.setZero();
    GDot2.setZero();

    updateKinematics(*model, Q, QDot, QDDot);

    ReferenceFramePtr f1 = body_a_frame;
    ReferenceFramePtr f2 = body_b_frame;
    ReferenceFramePtr f3 = body_c_frame;
    calcRelativeBodySpatialJacobianDot(*model, Q, QDot, GDot, f3, f1, f2);
    calcRelativeBodySpatialJacobian(*model, Q, G, f3, f1, f2);

    calcRelativeBodySpatialJacobianAndJacobianDot(*model, Q, QDot, G2, GDot2, f3, f1, f2);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(G, G2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDot, GDot2, unit_test_utils::E_MINUS_14));
}

TEST_F(SimpleFixture, calcRelativeBodySpatialJacobianDot)
{
    model->gravity.setZero();

    Body b1(1., Vector3d(0., 0., -0.1), Vector3d(1., 1., 1.));
    Joint jx(JointTypeRevoluteX);

    model->addBody(0, SpatialTransform(), jx, b1, "b1");
    model->appendBody(Xtrans(Vector3d(0., 0., -1.)), jx, b1, "b2");
    model->appendBody(Xtrans(Vector3d(0., 0., -2.)), jx, b1, "b3");

    VectorNd Q(model->q_size);
    VectorNd QDot(model->qdot_size);
    VectorNd QDDot(model->qdot_size);
    VectorNd Tau(model->qdot_size);
    Q.setZero();
    Q[0] = 1.1;
    Q[1] = -0.3;
    QDot.setZero();
    QDot[0] = 0.1;
    QDot[1] = 0.1;
    QDot[2] = 0.1;
    QDDot[0] = 0.2;
    QDDot[1] = 0.3;
    QDDot[2] = 0.5;

    MatrixNd GDot(6, model->qdot_size), G(6, model->qdot_size);
    GDot.setZero();
    G.setZero();

    updateKinematics(*model, Q, QDot, QDDot);

    ReferenceFramePtr f1 = model->bodyFrames[model->GetBodyId("b3")];
    ReferenceFramePtr f2 = model->bodyFrames[model->GetBodyId("b1")];
    ReferenceFramePtr f3 = model->worldFrame;
    ReferenceFramePtr f4 = model->bodyFrames[model->GetBodyId("b2")];

    calcRelativeBodySpatialJacobianDot(*model, Q, QDot, GDot, f2, f1, f2);
    calcRelativeBodySpatialJacobian(*model, Q, G, f2, f1, f2);

    SpatialAcceleration a_calc = calcSpatialAcceleration(*model, Q, QDot, QDDot, f2, f1, f2);
    EXPECT_TRUE(a_calc.isApprox(GDot * QDot + G * QDDot, unit_test_utils::E_MINUS_14));
}

TEST_F(SimpleFixture, calcRelativeBodySpatialJacobianAndJacobianDot)
{
    model->gravity.setZero();

    Body b1(1., Vector3d(0., 0., -0.1), Vector3d(1., 1., 1.));
    Joint jx(JointTypeRevoluteX);

    model->addBody(0, SpatialTransform(), jx, b1, "b1");
    model->appendBody(Xtrans(Vector3d(0., 0., -1.)), jx, b1, "b2");
    model->appendBody(Xtrans(Vector3d(0., 0., -2.)), jx, b1, "b3");

    VectorNd Q(model->q_size);
    VectorNd QDot(model->qdot_size);
    VectorNd QDDot(model->qdot_size);
    VectorNd Tau(model->qdot_size);
    Q.setZero();
    Q[0] = 1.1;
    Q[1] = -0.3;
    QDot.setZero();
    QDot[0] = 0.1;
    QDot[1] = 0.1;
    QDot[2] = 0.1;
    QDDot[0] = 0.2;
    QDDot[1] = -0.1;
    QDDot[2] = 0.5;

    MatrixNd GDot(6, model->qdot_size), G(6, model->qdot_size), GDot2(6, model->qdot_size), G2(6, model->qdot_size);
    G.setZero();
    G2.setZero();
    GDot.setZero();
    GDot2.setZero();

    updateKinematics(*model, Q, QDot, QDDot);

    ReferenceFramePtr f1 = model->bodyFrames[model->GetBodyId("b3")];
    ReferenceFramePtr f2 = model->bodyFrames[model->GetBodyId("b1")];
    ReferenceFramePtr f3 = model->worldFrame;
    ReferenceFramePtr f4 = model->bodyFrames[model->GetBodyId("b2")];

    calcRelativeBodySpatialJacobianDot(*model, Q, QDot, GDot, f2, f1, f2);
    calcRelativeBodySpatialJacobian(*model, Q, G, f2, f1, f2);
    calcRelativeBodySpatialJacobianAndJacobianDot(*model, Q, QDot, G2, GDot2, f2, f1, f2);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(G, G2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDot, GDot2, unit_test_utils::E_MINUS_14));
}

TEST_F(FloatingBaseWith2SingleDofJoints, calcRelativeBodySpatialJacobian)
{
    randomizeStates();
    ReferenceFramePtr body_root_frame = model->bodyFrames[root_body_id];
    ReferenceFramePtr body_1_frame = model->bodyFrames[body_1_id];
    ReferenceFramePtr body_2_frame = model->bodyFrames[body_2_id];

    SpatialMotion v = calcSpatialVelocity(*model, Q, QDot, body_1_id, body_2_id);
    v.changeFrame(body_root_frame);

    MatrixNd G(6, model->qdot_size);
    G.setZero();

    calcRelativeBodySpatialJacobian(*model, Q, G, body_1_frame, body_2_frame, body_root_frame);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(v, G * QDot, unit_test_utils::E_MINUS_14));
}

TEST_F(FloatingBaseWith2SingleDofJoints, calcRelativeBodySpatialJacobianDot)
{
    randomizeStates();
    QDDot.setZero();  // Needs to have zero accel until a method is written for calculating general accelerations
    ReferenceFramePtr body_a_frame = model->bodyFrames[root_body_id];
    ReferenceFramePtr body_b_frame = model->bodyFrames[body_1_id];
    ReferenceFramePtr body_c_frame = model->bodyFrames[body_2_id];

    MatrixNd G(6, model->qdot_size), GDot(6, model->qdot_size);
    G.setZero();
    GDot.setZero();

    updateKinematics(*model, Q, QDot, QDDot);

    ReferenceFramePtr f1 = body_b_frame;
    ReferenceFramePtr f2 = body_c_frame;
    ReferenceFramePtr f3 = body_a_frame;

    calcRelativeBodySpatialJacobianDot(*model, Q, QDot, GDot, f3, f1, f2, false);
    SpatialAcceleration a_calc = calcSpatialAcceleration(*model, Q, QDot, QDDot, f3, f1, f2, false);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(GDot * QDot + G * QDDot, a_calc, unit_test_utils::E_MINUS_14));
}

TEST_F(FloatingBaseWith2SingleDofJoints, calcRelativeBodySpatialJacobianAndJacobianDot)
{
    randomizeStates();
    QDDot.setZero();  // Needs to have zero accel until a method is written for calculating general accelerations
    ReferenceFramePtr body_a_frame = model->bodyFrames[root_body_id];
    ReferenceFramePtr body_b_frame = model->bodyFrames[body_1_id];
    ReferenceFramePtr body_c_frame = model->bodyFrames[body_2_id];

    MatrixNd G(6, model->qdot_size), GDot(6, model->qdot_size), G2(6, model->qdot_size), GDot2(6, model->qdot_size);
    G.setZero();
    GDot.setZero();
    G2.setZero();
    GDot2.setZero();

    updateKinematics(*model, Q, QDot, QDDot);

    ReferenceFramePtr f1 = body_b_frame;
    ReferenceFramePtr f2 = body_c_frame;
    ReferenceFramePtr f3 = body_a_frame;

    calcRelativeBodySpatialJacobianDot(*model, Q, QDot, GDot, f3, f1, f2, false);
    calcRelativeBodySpatialJacobian(*model, Q, G, f3, f1, f2, false);
    calcRelativeBodySpatialJacobianAndJacobianDot(*model, Q, QDot, G2, GDot2, f3, f1, f2, false);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(G, G2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDot, GDot2, unit_test_utils::E_MINUS_14));
}

TEST_F(RotZRotZYXFixed, calcRelativeBodySpatialJacobian)
{
    randomizeStates();
    ReferenceFramePtr body_fixed_frame = model->fixedBodyFrames[body_fixed_id - model->fixed_body_discriminator];

    ReferenceFramePtr body_a_frame = model->bodyFrames[body_a_id];
    ReferenceFramePtr body_b_frame = model->bodyFrames[body_b_id];

    SpatialMotion v = calcSpatialVelocity(*model, Q, QDot, body_a_id, body_b_id);
    v.changeFrame(body_fixed_frame);

    MatrixNd G(6, model->qdot_size);
    G.setZero();

    calcRelativeBodySpatialJacobian(*model, Q, G, body_a_frame, body_b_frame, body_fixed_frame);
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(v, G * QDot, unit_test_utils::E_MINUS_13));
    v = calcSpatialVelocity(*model, Q, QDot, body_fixed_id, body_b_id);
    v.changeFrame(body_a_frame);

    G.setZero();

    calcRelativeBodySpatialJacobian(*model, Q, G, body_fixed_frame, body_b_frame, body_a_frame);
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(v, G * QDot, unit_test_utils::E_MINUS_14));

    v = calcSpatialVelocity(*model, Q, QDot, body_b_id, body_fixed_id);
    v.changeFrame(body_a_frame);
    EXPECT_TRUE(v.isZero(unit_test_utils::E_MINUS_13));

    G.setZero();

    calcRelativeBodySpatialJacobian(*model, Q, G, body_b_frame, body_fixed_frame, body_a_frame);
    EXPECT_TRUE((G * QDot).isZero(unit_test_utils::E_MINUS_13));
}

TEST_F(Human36, calcRelativeBodySpatialJacobian)
{
    randomizeStates();
    unsigned int body_trunk_id = model_3dof->GetBodyId("middletrunk");
    unsigned int body_hand_r_id = model_3dof->GetBodyId("hand_r");
    unsigned int body_hand_l_id = model_3dof->GetBodyId("hand_l");
    ReferenceFramePtr body_trunk_frame = model_3dof->bodyFrames[body_trunk_id];
    ReferenceFramePtr body_hand_r_frame = model_3dof->bodyFrames[body_hand_r_id];
    ReferenceFramePtr body_hand_l_frame = model_3dof->bodyFrames[body_hand_l_id];

    SpatialMotion v = calcSpatialVelocity(*model_3dof, q, qdot, body_hand_r_id, body_hand_l_id);
    v.changeFrame(body_trunk_frame);

    MatrixNd G(6, model_3dof->qdot_size);
    G.setZero();

    calcRelativeBodySpatialJacobian(*model_3dof, q, G, body_hand_r_frame, body_hand_l_frame, body_trunk_frame);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(v, G * qdot, unit_test_utils::E_MINUS_14));
}

TEST_F(Human36, calcRelativeBodySpatialJacobianDotEmulated)
{
    randomizeStates();
    // qddot.setZero();
    unsigned int body_trunk_id = model_emulated->GetBodyId("middletrunk");
    unsigned int body_hand_r_id = model_emulated->GetBodyId("hand_r");
    unsigned int body_hand_l_id = model_emulated->GetBodyId("hand_l");
    ReferenceFramePtr body_trunk_frame = model_emulated->bodyFrames[body_trunk_id];
    ReferenceFramePtr body_hand_r_frame = model_emulated->bodyFrames[body_hand_r_id];
    ReferenceFramePtr body_hand_l_frame = model_emulated->bodyFrames[body_hand_l_id];

    updateKinematics(*model_emulated, q, qdot, qddot);

    MatrixNd GDot(6, model_emulated->qdot_size), G(6, model_emulated->qdot_size);
    GDot.setZero();

    ReferenceFramePtr f1 = body_trunk_frame;
    ReferenceFramePtr f2 = body_hand_r_frame;
    ReferenceFramePtr f3 = body_hand_l_frame;
    calcRelativeBodySpatialJacobianDot(*model_emulated, q, qdot, GDot, f1, f2, f3, false);
    calcRelativeBodySpatialJacobian(*model_emulated, q, G, f1, f2, f3, false);

    SpatialAcceleration a_calc = calcSpatialAcceleration(*model_emulated, q, qdot, qddot, f1, f2, f3, false);
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(a_calc, GDot * qdot + G * qddot, unit_test_utils::E_MINUS_13));
}

TEST_F(Human36, calcRelativeBodySpatialJacobianAndJacobianDotEmulated)
{
    randomizeStates();
    qddot.setZero();
    unsigned int body_trunk_id = model_emulated->GetBodyId("middletrunk");
    unsigned int body_hand_r_id = model_emulated->GetBodyId("hand_r");
    unsigned int body_hand_l_id = model_emulated->GetBodyId("hand_l");
    ReferenceFramePtr body_trunk_frame = model_emulated->bodyFrames[body_trunk_id];
    ReferenceFramePtr body_hand_r_frame = model_emulated->bodyFrames[body_hand_r_id];
    ReferenceFramePtr body_hand_l_frame = model_emulated->bodyFrames[body_hand_l_id];

    updateKinematics(*model_emulated, q, qdot, qddot);

    MatrixNd GDot(6, model_emulated->qdot_size), G(6, model_emulated->qdot_size), GDot2(6, model_emulated->qdot_size), G2(6, model_emulated->qdot_size);
    GDot.setZero();
    G.setZero();
    G2.setZero();
    GDot2.setZero();

    ReferenceFramePtr f1 = body_trunk_frame;
    ReferenceFramePtr f2 = body_hand_r_frame;
    ReferenceFramePtr f3 = body_hand_l_frame;
    calcRelativeBodySpatialJacobianDot(*model_emulated, q, qdot, GDot, f1, f2, f3, false);
    calcRelativeBodySpatialJacobian(*model_emulated, q, G, f1, f2, f3, false);
    calcRelativeBodySpatialJacobianAndJacobianDot(*model_emulated, q, qdot, G2, GDot2, f1, f2, f3, false);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(G, G2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDot, GDot2, unit_test_utils::E_MINUS_14));
}

TEST_F(Human36, calcRelativeBodySpatialJacobianDot3Dof)
{
    randomizeStates();
    unsigned int body_trunk_id = model_3dof->GetBodyId("middletrunk");
    unsigned int body_hand_r_id = model_3dof->GetBodyId("hand_r");
    unsigned int body_hand_l_id = model_3dof->GetBodyId("hand_l");
    ReferenceFramePtr body_trunk_frame = model_3dof->bodyFrames[body_trunk_id];
    ReferenceFramePtr body_hand_r_frame = model_3dof->bodyFrames[body_hand_r_id];
    ReferenceFramePtr body_hand_l_frame = model_3dof->bodyFrames[body_hand_l_id];

    updateKinematics(*model_3dof, q, qdot, qddot);

    MatrixNd GDot(6, model_3dof->qdot_size), G(6, model_3dof->qdot_size);
    GDot.setZero();
    G.setZero();

    ReferenceFramePtr f1 = body_trunk_frame;
    ReferenceFramePtr f2 = body_hand_r_frame;
    ReferenceFramePtr f3 = body_hand_l_frame;
    calcRelativeBodySpatialJacobianDot(*model_3dof, q, qdot, GDot, f1, f2, f3, false);
    calcRelativeBodySpatialJacobian(*model_3dof, q, G, f1, f2, f3, false);

    SpatialAcceleration a_calc = calcSpatialAcceleration(*model_3dof, q, qdot, qddot, f1, f2, f3, false);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(a_calc, GDot * qdot + G * qddot, unit_test_utils::E_MINUS_13));
}

TEST_F(Human36, calcRelativeBodySpatialJacobianAndJacobianDot3Dof)
{
    randomizeStates();
    unsigned int body_trunk_id = model_3dof->GetBodyId("middletrunk");
    unsigned int body_hand_r_id = model_3dof->GetBodyId("hand_r");
    unsigned int body_hand_l_id = model_3dof->GetBodyId("hand_l");
    ReferenceFramePtr body_trunk_frame = model_3dof->bodyFrames[body_trunk_id];
    ReferenceFramePtr body_hand_r_frame = model_3dof->bodyFrames[body_hand_r_id];
    ReferenceFramePtr body_hand_l_frame = model_3dof->bodyFrames[body_hand_l_id];

    updateKinematics(*model_3dof, q, qdot, qddot);

    MatrixNd GDot(6, model_3dof->qdot_size), G(6, model_3dof->qdot_size), GDot2(6, model_3dof->qdot_size), G2(6, model_3dof->qdot_size);
    GDot.setZero();
    G.setZero();
    GDot2.setZero();
    G2.setZero();

    ReferenceFramePtr f1 = body_trunk_frame;
    ReferenceFramePtr f2 = body_hand_r_frame;
    ReferenceFramePtr f3 = body_hand_l_frame;
    calcRelativeBodySpatialJacobianDot(*model_3dof, q, qdot, GDot, f1, f2, f3, false);
    calcRelativeBodySpatialJacobian(*model_3dof, q, G, f1, f2, f3, false);
    calcRelativeBodySpatialJacobianAndJacobianDot(*model_3dof, q, qdot, G2, GDot2, f1, f2, f3, false);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(G, G2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDot, GDot2, unit_test_utils::E_MINUS_14));
}

TEST_F(Human36, calcRelativeBodySpatialJacobianDot3DofFixedBody)
{
    randomizeStates();
    unsigned int body_trunk_id = model_3dof->GetBodyId("uppertrunk");
    unsigned int body_hand_r_id = model_3dof->GetBodyId("hand_r");
    unsigned int body_hand_l_id = model_3dof->GetBodyId("hand_l");
    ReferenceFramePtr body_trunk_frame = model_3dof->fixedBodyFrames[body_trunk_id - model_3dof->fixed_body_discriminator];
    ReferenceFramePtr body_hand_r_frame = model_3dof->bodyFrames[body_hand_r_id];
    ReferenceFramePtr body_hand_l_frame = model_3dof->bodyFrames[body_hand_l_id];

    updateKinematics(*model_3dof, q, qdot, qddot);

    MatrixNd GDot(6, model_3dof->qdot_size), G(6, model_3dof->qdot_size);
    GDot.setZero();
    G.setZero();

    ReferenceFramePtr f1 = body_trunk_frame;
    ReferenceFramePtr f2 = body_hand_r_frame;
    ReferenceFramePtr f3 = body_hand_l_frame;
    calcRelativeBodySpatialJacobianDot(*model_3dof, q, qdot, GDot, f1, f2, f3, false);
    calcRelativeBodySpatialJacobian(*model_3dof, q, G, f1, f2, f3, false);

    SpatialAcceleration a_calc = calcSpatialAcceleration(*model_3dof, q, qdot, qddot, f1, f2, f3, false);
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(a_calc, GDot * qdot + G * qddot, unit_test_utils::E_MINUS_13));

    G.setZero();
    GDot.setZero();
    calcRelativeBodySpatialJacobianDot(*model_3dof, q, qdot, GDot, f2, f1, f3, false);
    calcRelativeBodySpatialJacobian(*model_3dof, q, G, f2, f1, f3, false);

    a_calc = calcSpatialAcceleration(*model_3dof, q, qdot, qddot, f2, f1, f3, false);
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(a_calc, GDot * qdot + G * qddot, unit_test_utils::E_MINUS_13));

    G.setZero();
    GDot.setZero();
    calcRelativeBodySpatialJacobianDot(*model_3dof, q, qdot, GDot, f3, f2, f1, false);
    calcRelativeBodySpatialJacobian(*model_3dof, q, G, f3, f2, f1, false);

    a_calc = calcSpatialAcceleration(*model_3dof, q, qdot, qddot, f3, f2, f1, false);
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(a_calc, GDot * qdot + G * qddot, unit_test_utils::E_MINUS_13));
}

TEST_F(Human36, calcRelativeBodySpatialJacobianAndJacobianDot3DofFixedBody)
{
    randomizeStates();
    unsigned int body_trunk_id = model_3dof->GetBodyId("uppertrunk");
    unsigned int body_hand_r_id = model_3dof->GetBodyId("hand_r");
    unsigned int body_hand_l_id = model_3dof->GetBodyId("hand_l");
    ReferenceFramePtr body_trunk_frame = model_3dof->fixedBodyFrames[body_trunk_id - model_3dof->fixed_body_discriminator];
    ReferenceFramePtr body_hand_r_frame = model_3dof->bodyFrames[body_hand_r_id];
    ReferenceFramePtr body_hand_l_frame = model_3dof->bodyFrames[body_hand_l_id];

    updateKinematics(*model_3dof, q, qdot, qddot);

    MatrixNd GDot(6, model_3dof->qdot_size), G(6, model_3dof->qdot_size), GDot2(6, model_3dof->qdot_size), G2(6, model_3dof->qdot_size);
    GDot.setZero();
    G.setZero();
    GDot2.setZero();
    G2.setZero();

    ReferenceFramePtr f1 = body_trunk_frame;
    ReferenceFramePtr f2 = body_hand_r_frame;
    ReferenceFramePtr f3 = body_hand_l_frame;
    calcRelativeBodySpatialJacobianDot(*model_3dof, q, qdot, GDot, f1, f2, f3, false);
    calcRelativeBodySpatialJacobian(*model_3dof, q, G, f1, f2, f3, false);
    calcRelativeBodySpatialJacobianAndJacobianDot(*model_3dof, q, qdot, G2, GDot2, f1, f2, f3, false);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(G, G2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDot, GDot2, unit_test_utils::E_MINUS_14));

    G.setZero();
    GDot.setZero();
    G2.setZero();
    GDot2.setZero();
    calcRelativeBodySpatialJacobianDot(*model_3dof, q, qdot, GDot, f2, f1, f3, false);
    calcRelativeBodySpatialJacobian(*model_3dof, q, G, f2, f1, f3, false);
    calcRelativeBodySpatialJacobianAndJacobianDot(*model_3dof, q, qdot, G2, GDot2, f2, f1, f3, false);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(G, G2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDot, GDot2, unit_test_utils::E_MINUS_14));

    G.setZero();
    GDot.setZero();
    G2.setZero();
    GDot2.setZero();
    calcRelativeBodySpatialJacobianDot(*model_3dof, q, qdot, GDot, f3, f2, f1, false);
    calcRelativeBodySpatialJacobian(*model_3dof, q, G, f3, f2, f1, false);
    calcRelativeBodySpatialJacobianAndJacobianDot(*model_3dof, q, qdot, G2, GDot2, f3, f2, f1, false);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(G, G2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDot, GDot2, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlKinematicsSingleChainFixture, TestCalcPointJacobian)
{
    Model model;
    Body base_body(1., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.));

    unsigned int base_body_id = model.addBody(0, SpatialTransform(),
                                              Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                                                    SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                                              base_body, "b1");

    Body fixed_body(1., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.));
    Body fixed_body_2(1.1, Vector3d(1., 1., 1.), Vector3d(1.2, 1.3, 1.4));

    unsigned int fixed_body1_id = model.addBody(base_body_id, SpatialTransform(Xtrans(Vector3d(0.01, 0.02, 0.03))), Joint(JointTypeFixed), base_body, "fb1");
    unsigned int fixed_body2_id = model.addBody(base_body_id, SpatialTransform(Xtrans(Vector3d(-0.18, -0.05, 0.051))), Joint(JointTypeFixed), base_body, "fb2");

    VectorNd Q = VectorNd::Constant((size_t)model.dof_count, 0.);
    VectorNd QDot = VectorNd::Constant((size_t)model.dof_count, 0.);
    MatrixNd G = MatrixNd::Constant(3, model.dof_count, 0.);
    MatrixNd G2 = MatrixNd::Constant(3, model.dof_count, 0.);
    Vector3d point_position(1.1, 1.2, 2.1);
    FrameVector point_velocity_ref;
    Vector3d point_velocity;

    Q[0] = 1.1;
    Q[1] = 1.2;
    Q[2] = 1.3;
    Q[3] = 0.7;
    Q[4] = 0.8;
    Q[5] = 0.9;

    QDot[0] = -1.1;
    QDot[1] = 2.2;
    QDot[2] = 1.3;
    QDot[3] = -2.7;
    QDot[4] = 1.8;
    QDot[5] = -2.9;

    //     Compute the reference velocity
    point_velocity_ref = calcPointVelocity(model, Q, QDot, base_body_id, point_position);

    G.setZero();

    calcPointJacobian(model, Q, base_body_id, point_position, G);

    point_velocity = G * QDot;

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(point_velocity_ref, point_velocity, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));

    point_position = Vector3d(0.2, -0.1, 0.8);

    point_velocity_ref = calcPointVelocity(model, Q, QDot, fixed_body1_id, point_position);

    G.setZero();

    calcPointJacobian(model, Q, fixed_body1_id, point_position, G);

    point_velocity = G * QDot;

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(point_velocity_ref, point_velocity, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));

    point_position = Vector3d(1.2, -1.1, 1.8);

    point_velocity_ref = calcPointVelocity(model, Q, QDot, fixed_body2_id, point_position);

    G.setZero();

    calcPointJacobian(model, Q, fixed_body2_id, point_position, G);

    point_velocity = G * QDot;

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(point_velocity_ref, point_velocity, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));
}

TEST_F(RdlKinematicsSingleChainFixture, TestInverseKinematicSimple)
{
    std::vector<unsigned int> body_ids;
    std::vector<Vector3d> body_points;
    std::vector<Vector3d> target_pos;

    Q[0] = 0.2;
    Q[1] = 0.1;
    Q[2] = 0.1;

    VectorNd Qres = VectorNd::Zero((size_t)model->dof_count);

    unsigned int body_id = body_d_id;
    Vector3d body_point = Vector3d(1., 0., 0.);
    Vector3d target(1.3, 0., 0.);

    body_ids.push_back(body_d_id);
    body_points.push_back(body_point);
    target_pos.push_back(target);

    bool res = inverseKinematics(*model, Q, body_ids, body_points, target_pos, Qres);
    EXPECT_EQ(true, res);

    updateKinematicsCustom(*model, &Qres, NULL, NULL);

    FramePoint p(model->bodyFrames[body_id], body_point);
    p.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(target, p.vec(), unit_test_utils::E_MINUS_12));
}

TEST_F(RdlKinematicsSingleChainFixture6DoF, TestInverseKinematicUnreachable)
{
    std::vector<unsigned int> body_ids;
    std::vector<Vector3d> body_points;
    std::vector<Vector3d> target_pos;

    Q[0] = 0.2;
    Q[1] = 0.1;
    Q[2] = 0.1;

    VectorNd Qres = VectorNd::Zero((size_t)model->dof_count);

    unsigned int body_id = child_id;
    Vector3d body_point = Vector3d(1., 0., 0.);
    Vector3d target(2.2, 0., 0.);

    body_ids.push_back(body_id);
    body_points.push_back(body_point);
    target_pos.push_back(target);

    bool res = inverseKinematics(*model, Q, body_ids, body_points, target_pos, Qres, 1.0e-8, 0.9, 1000);

    EXPECT_EQ(true, res);

    updateKinematicsCustom(*model, &Qres, NULL, NULL);

    FramePoint p(model->bodyFrames[body_id], body_point);
    p.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(2.0, 0., 0.), p.vec(), 1.0e-7));
}

TEST_F(RdlKinematicsSingleChainFixture6DoF, TestInverseKinematicTwoPoints)
{
    std::vector<unsigned int> body_ids;
    std::vector<Vector3d> body_points;
    std::vector<Vector3d> target_pos;

    Q[0] = 0.2;
    Q[1] = 0.1;
    Q[2] = 0.1;

    VectorNd Qres = VectorNd::Zero((size_t)model->dof_count);

    unsigned int body_id = child_id;
    Vector3d body_point = Vector3d(1., 0., 0.);
    Vector3d target(2., 0., 0.);

    body_ids.push_back(body_id);
    body_points.push_back(body_point);
    target_pos.push_back(target);

    body_ids.push_back(base_id);
    body_points.push_back(Vector3d(0.6, 1.0, 0.));
    target_pos.push_back(Vector3d(0.5, 1.1, 0.));

    bool res = inverseKinematics(*model, Q, body_ids, body_points, target_pos, Qres, 1.0e-3, 0.9, 200);
    EXPECT_EQ(true, res);

    updateKinematicsCustom(*model, &Qres, NULL, NULL);

    FramePoint p(model->bodyFrames[body_ids[0]], body_points[0]);
    p.changeFrame(ReferenceFrame::getWorldFrame());
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(target_pos[0], p.vec(), 1.0e-1));

    p.setIncludingFrame(body_points[1], model->bodyFrames[body_ids[1]]);
    p.changeFrame(ReferenceFrame::getWorldFrame());
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(target_pos[1], p.vec(), 1.0e-1));
}

TEST_F(RdlKinematicsSingleChainFixture6DoF, FixedJointBodyCalcBodyToBase)
{
    // the standard modeling using a null body
    Body null_body;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Model model;

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));
    model.addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
    model.appendBody(Xtrans(Vector3d(0., 1., 0.)), Joint(JointTypeFixed), fixed_body);

    VectorNd Q_zero = VectorNd::Zero(model.dof_count);

    FramePoint p(model.fixedBodyFrames[0], 1., 1., 0.1);

    p.changeFrame(model.worldFrame);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 2., 0.1), p.vec(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));
}

TEST_F(RdlKinematicsSingleChainFixture6DoF, FixedJointBodyCalcBodyToBaseRotated)
{
    // the standard modeling using a null body
    Body null_body;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Model model;

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));
    model.addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
    model.appendBody(Xtrans(Vector3d(1., 0., 0.)), Joint(JointTypeFixed), fixed_body);

    VectorNd Q = VectorNd::Zero(model.dof_count);

    Q[0] = M_PI * 0.5;

    updateKinematicsCustom(model, &Q, nullptr, nullptr);
    FramePoint p(model.fixedBodyFrames[0], Vector3d(1., 0., 0.));

    p.changeFrame(ReferenceFrame::getWorldFrame());
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 2., 0.), p.vec(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));
}

TEST_F(RdlKinematicsSingleChainFixture6DoF, FixedJointBodyCalcBaseToBody)
{
    // the standard modeling using a null body
    Body null_body;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Model model;

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));
    model.addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
    model.appendBody(Xtrans(Vector3d(0., 1., 0.)), Joint(JointTypeFixed), fixed_body);

    VectorNd Q_zero = VectorNd::Zero(model.dof_count);

    updateKinematicsCustom(model, &Q_zero, nullptr, nullptr);
    FramePoint p(ReferenceFrame::getWorldFrame(), Vector3d(1., 2., 0.1));

    p.changeFrame(model.fixedBodyFrames[0]);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 1., 0.1), p.vec(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));
}

TEST_F(RdlKinematicsSingleChainFixture6DoF, FixedJointBodyCalcBaseToBodyRotated)
{
    // the standard modeling using a null body
    Body null_body;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Model model;

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));
    model.addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
    model.appendBody(Xtrans(Vector3d(1., 0., 0.)), Joint(JointTypeFixed), fixed_body);

    VectorNd Q = VectorNd::Zero(model.dof_count);

    Q[0] = M_PI * 0.5;

    updateKinematicsCustom(model, &Q, nullptr, nullptr);
    FramePoint p(ReferenceFrame::getWorldFrame(), Vector3d(0., 2., 0.));

    p.changeFrame(model.fixedBodyFrames[0]);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(1., 0., 0.), p.vec(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));
}

TEST_F(RdlKinematicsSingleChainFixture6DoF, FixedJointBodyWorldOrientation)
{
    // the standard modeling using a null body
    Body null_body;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Model model;

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));
    model.addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);

    SpatialTransform transform = Xrotz(0.25) * Xtrans(Vector3d(1., 2., 3.));
    unsigned int fixed_body_id = model.appendBody(transform, Joint(JointTypeFixed), fixed_body);

    updateKinematicsCustom(model, &Q, nullptr, nullptr);
    VectorNd Q_zero = VectorNd::Zero(model.dof_count);
    Matrix3d orientation = model.fixedBodyFrames[fixed_body_id - model.fixed_body_discriminator]->getInverseTransformToRoot().E;

    Matrix3d reference = transform.E;

    EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(reference, orientation, unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));
}

TEST_F(RdlKinematicsSingleChainFixture6DoF, FixedJointCalcPointJacobian)
{
    // the standard modeling using a null body
    Body null_body;
    Body body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));

    Model model;

    Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));
    model.addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);

    SpatialTransform transform = Xrotz(0.25) * Xtrans(Vector3d(1., 2., 3.));
    unsigned int fixed_body_id = model.appendBody(transform, Joint(JointTypeFixed), fixed_body);

    VectorNd Q = VectorNd::Zero(model.dof_count);
    VectorNd QDot = VectorNd::Zero(model.dof_count);

    Q[0] = 1.1;
    QDot[0] = 1.2;

    Vector3d point_position(1., 0., 0.);

    MatrixNd G = MatrixNd::Zero(3, model.dof_count);
    calcPointJacobian(model, Q, fixed_body_id, point_position, G);
    Vector3d point_velocity_jacobian = G * QDot;
    FrameVector point_velocity_reference = calcPointVelocity(model, Q, QDot, fixed_body_id, point_position);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(point_velocity_reference, point_velocity_jacobian, unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));
}

TEST_F(Human36, SpatialJacobianSimple)
{
    randomizeStates();

    unsigned int foot_r_id = model->GetBodyId("foot_r");
    MatrixNd G(MatrixNd::Zero(6, model->dof_count));

    calcBodySpatialJacobian(*model, q, foot_r_id, G);

    updateKinematicsCustom(*model, &q, &qdot, NULL);
    SpatialVector v_body = SpatialVector(G * qdot);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model->v[foot_r_id], v_body, unit_test_utils::E_MINUS_12));

    G.setZero();
    calcBodySpatialJacobian(*model_3dof, q, foot_r_id, G);

    updateKinematicsCustom(*model_3dof, &q, &qdot, NULL);
    v_body = SpatialVector(G * qdot);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(model_3dof->v[foot_r_id], v_body, unit_test_utils::E_MINUS_12));
}

TEST_F(Human36, SpatialJacobianFixedBody)
{
    randomizeStates();

    unsigned int uppertrunk_id = model->GetBodyId("uppertrunk");
    MatrixNd G(MatrixNd::Zero(6, model->dof_count));

    updateKinematicsCustom(*model, &q, &qdot, NULL);
    calcBodySpatialJacobian(*model, q, uppertrunk_id, G);

    unsigned int fixed_body_id = uppertrunk_id - model->fixed_body_discriminator;
    unsigned int movable_parent = model->mFixedBodies[fixed_body_id].mMovableParent;

    updateKinematicsCustom(*model, &q, &qdot, NULL);
    SpatialVector v_body = SpatialVector(G * qdot);

    SpatialVector v_fixed_body = model->mFixedBodies[fixed_body_id].mParentTransform.apply(model->v[movable_parent]);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(v_fixed_body, v_body, unit_test_utils::E_MINUS_12));
}

TEST_F(Human36, CalcPointJacobian6D)
{
    unsigned int foot_r_id = model->GetBodyId("foot_r");
    Body nullBody(0., Vector3d(0., 0., 0.), Vector3d(0., 0., 0.));
    Vector3d point_local(1.1, 2.2, 3.3);

    model->addBody(foot_r_id, Xtrans(point_local), Joint(JointTypeFixed), nullBody, "right_sole_frame");
    randomizeStates();

    // Compute the 6-D velocity using the 6-D Jacobian
    MatrixNd G(MatrixNd::Zero(6, model->dof_count));
    calcPointJacobian6D(*model, q, foot_r_id, point_local, G);
    SpatialVector v_foot_0_jac = SpatialVector(G * qdot);

    updateKinematicsCustom(*model, &q, &qdot, NULL);

    FrameVectorPair v_f = calcPointVelocity6D(*model, q, qdot, model->GetBodyId("right_sole_frame"), Vector3d(0., 0., 0.), false);

    EXPECT_TRUE(v_f.linear().isApprox(v_foot_0_jac.getLinearPart(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(v_f.angular().isApprox(v_foot_0_jac.getAngularPart(), unit_test_utils::E_MINUS_12));
}

TEST_F(Human36, CalcPointJacobian6DRelative)
{
    unsigned int foot_r_id = model->GetBodyId("foot_r");
    Body nullBody(0., Vector3d(0., 0., 0.), Vector3d(0., 0., 0.));
    Vector3d point_local(1.1, 2.2, 3.3);

    model->addBody(foot_r_id, Xtrans(point_local), Joint(JointTypeFixed), nullBody, "right_sole_frame");
    randomizeStates();

    updateKinematicsCustom(*model, &q, &qdot, nullptr);
    // Compute the 6-D velocity using the 6-D Jacobian
    MatrixNd G(MatrixNd::Zero(6, model->dof_count));
    ReferenceFramePtr baseFrame = model->getReferenceFrame("foot_r");
    ReferenceFramePtr relFrame = model->getReferenceFrame("pelvis");
    ReferenceFramePtr expressedInFrame = model->getReferenceFrame("right_sole_frame");

    calcRelativePointJacobian6D(*model, q, G, baseFrame, relFrame, expressedInFrame, false);
    SpatialVector v_foot_0_jac = SpatialVector(G * qdot);

    FrameVectorPair v_f = calcPointVelocity6D(*model, q, qdot, baseFrame, relFrame, expressedInFrame, false);
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v_foot_0_jac.getLinearPart(), v_f.linear(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v_foot_0_jac.getAngularPart(), v_f.angular(), unit_test_utils::E_MINUS_12));

    G.setZero();
    baseFrame = model->getReferenceFrame("pelvis");
    relFrame = model->getReferenceFrame("foot_r");
    expressedInFrame = model->getReferenceFrame("right_sole_frame");

    calcRelativePointJacobian6D(*model, q, G, baseFrame, relFrame, expressedInFrame, false);
    v_foot_0_jac = SpatialVector(G * qdot);

    v_f = calcPointVelocity6D(*model, q, qdot, baseFrame, relFrame, expressedInFrame, false);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v_foot_0_jac.getLinearPart(), v_f.linear(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v_foot_0_jac.getAngularPart(), v_f.angular(), unit_test_utils::E_MINUS_12));

    G.setZero();
    baseFrame = model->getReferenceFrame("right_sole_frame");
    relFrame = model->getReferenceFrame("foot_r");
    expressedInFrame = model->getReferenceFrame("pelvis");

    calcRelativePointJacobian6D(*model, q, G, baseFrame, relFrame, expressedInFrame, false);
    v_foot_0_jac = SpatialVector(G * qdot);

    v_f = calcPointVelocity6D(*model, q, qdot, baseFrame, relFrame, expressedInFrame, false);

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v_foot_0_jac.getLinearPart(), v_f.linear(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(v_foot_0_jac.getAngularPart(), v_f.angular(), unit_test_utils::E_MINUS_12));
}

TEST_F(Human36, CalcPointVelocity6D)
{
    randomizeStates();

    unsigned int foot_r_id = model->GetBodyId("foot_r");
    Vector3d point_local(1.1, 2.2, 3.3);

    // Compute the 6-D velocity
    updateKinematicsCustom(*model, &q, &qdot, nullptr);
    FrameVectorPair v_foot_0 = calcPointVelocity6D(*model, q, qdot, foot_r_id, point_local, false);

    // Compute the 6-D velocity by transforming the body velocity to the
    // reference point and aligning it with the base coordinate system
    FramePoint p(model->bodyFrames[foot_r_id], point_local);
    p.changeFrame(ReferenceFrame::getWorldFrame());

    SpatialTransform X_foot(Matrix3d::Identity(), p.vec());
    updateKinematicsCustom(*model, &q, &qdot, NULL);
    SpatialVector v_foot_0_ref = X_foot.apply(model->bodyFrames[foot_r_id]->getTransformToRoot().apply(model->v[foot_r_id]));

    EXPECT_TRUE(v_foot_0_ref.getLinearPart().isApprox(v_foot_0.linear(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(v_foot_0_ref.getAngularPart().isApprox(v_foot_0.angular(), unit_test_utils::E_MINUS_12));
}

TEST_F(Human36, CalcPointVelocity6DFrames)
{
    randomizeStates();

    unsigned int foot_r_id = model->GetBodyId("foot_r");

    // Compute the 6-D velocity
    updateKinematicsCustom(*model, &q, &qdot, nullptr);
    FrameVectorPair v_foot_0 = calcPointVelocity6D(*model, q, qdot, foot_r_id, RobotDynamics::Math::Vector3dZero, false);
    FrameVectorPair v_foot_0_frame = calcPointVelocity6D(*model, q, qdot, model->getReferenceFrame("foot_r"), false);

    EXPECT_TRUE(v_foot_0.linear().isApprox(v_foot_0_frame.linear(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(v_foot_0.angular().isApprox(v_foot_0_frame.angular(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlKinematicsSingleChainFixture, CalcPointVelocity6D)
{
    randomizeStates();
    ReferenceFramePtr body_a_frame = model->bodyFrames[body_a_id];
    ReferenceFramePtr body_b_frame = model->bodyFrames[body_b_id];
    ReferenceFramePtr body_c_frame = model->bodyFrames[body_c_id];

    FrameVectorPair v = calcPointVelocity6D(*model, Q, QDot, body_b_id, RobotDynamics::Math::Vector3dZero, false);
    FrameVectorPair v_frame = calcPointVelocity6D(*model, Q, QDot, body_b_frame, false);
    EXPECT_TRUE(v.linear().isApprox(v_frame.linear(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(v.angular().isApprox(v_frame.angular(), unit_test_utils::E_MINUS_14));

    randomizeStates();

    unsigned int fb_id = model->addBody(body_c_id, RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.1, -0.2, 0.3)),
                                        RobotDynamics::Joint(RobotDynamics::JointType::JointTypeFixed), body_c, "fb1");

    RobotDynamics::ReferenceFramePtr fb_frame = model->getReferenceFrame("fb1");

    v = calcPointVelocity6D(*model, Q, QDot, fb_id, RobotDynamics::Math::Vector3dZero, false);
    v_frame = calcPointVelocity6D(*model, Q, QDot, fb_frame, false);
    EXPECT_TRUE(v.linear().isApprox(v_frame.linear(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(v.angular().isApprox(v_frame.angular(), unit_test_utils::E_MINUS_14));

    unsigned int fb_id2 = model->addBody(fb_id, RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.2, -0.3, 0.4)),
                                         RobotDynamics::Joint(RobotDynamics::JointType::JointTypeFixed), body_c, "fb2");

    fb_frame = model->getReferenceFrame("fb2");

    v = calcPointVelocity6D(*model, Q, QDot, fb_id2, RobotDynamics::Math::Vector3dZero, false);
    v_frame = calcPointVelocity6D(*model, Q, QDot, fb_frame, false);
    EXPECT_TRUE(v.linear().isApprox(v_frame.linear(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(v.angular().isApprox(v_frame.angular(), unit_test_utils::E_MINUS_14));
}

TEST_F(Human36, CalcPointVelocity6DFixedBody)
{
    randomizeStates();

    unsigned int foot_r_id = model->GetBodyId("foot_r");
    Vector3d point_local(1.1, 2.2, 3.3);

    Body nullBody(0., Vector3d(0., 0., 0.), Vector3d(0., 0., 0.));

    unsigned int fixed_body_id = model->addBody(foot_r_id, Xtrans(point_local), Joint(JointTypeFixed), nullBody, "right_sole_frame");

    // Compute the 6-D velocity
    FrameVectorPair v_fixed = calcPointVelocity6D(*model, q, qdot, fixed_body_id, point_local);

    MatrixNd G(MatrixNd::Zero(6, model->dof_count));

    calcPointJacobian6D(*model, q, fixed_body_id, point_local, G);
    SpatialVector v_j_qdot = G * qdot;

    EXPECT_TRUE(v_j_qdot.getLinearPart().isApprox(v_fixed.linear(), v_j_qdot.norm() * unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(v_j_qdot.getAngularPart().isApprox(v_fixed.angular(), v_j_qdot.norm() * unit_test_utils::E_MINUS_14));
}

TEST_F(Human36, CalcPointJacobianDot6D)
{
    randomizeStates();
    unsigned int body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    Vector3d point_local(1.1, 2.2, 3.3);

    MatrixNd G(6, model->qdot_size), GDot(6, model->qdot_size);
    G.setZero();
    GDot.setZero();
    calcPointJacobianDot6D(*model, q, qdot, body_id, point_local, GDot);
    calcPointJacobian6D(*model, q, body_id, point_local, G);
    FrameVectorPair a_ref = calcPointAcceleration6D(*model, q, qdot, qddot, body_id, point_local);
    SpatialVector a_exp = G * qddot + GDot * qdot;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_ref.linear(), a_exp.getLinearPart(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_ref.angular(), a_exp.getAngularPart(), unit_test_utils::E_MINUS_12));

    body_id = (rand() % (model_3dof->mBodies.size() - 2)) + 1;
    G.setZero();
    GDot.setZero();
    a_ref = calcPointAcceleration6D(*model_3dof, q, qdot, qddot, body_id, point_local);
    calcPointJacobianDot6D(*model_3dof, q, qdot, body_id, point_local, GDot);
    calcPointJacobian6D(*model_3dof, q, body_id, point_local, G);
    a_exp = G * qddot + GDot * qdot;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_ref.linear(), a_exp.getLinearPart(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_ref.angular(), a_exp.getAngularPart(), unit_test_utils::E_MINUS_12));

    Joint fj(JointTypeFixed);
    Body fb(1., Vector3d(0.1, 0.1, 0.1), Vector3d(0.1, 0.1, 0.1));
    body_id = (rand() % (model_3dof->mBodies.size() - 2)) + 1;
    unsigned int fb_id = model_3dof->addBody(body_id, Xtrans(Vector3d(-0.01, 0.02, 0.01)) * Xrotx(0.5), fj, fb);
    G.setZero();
    GDot.setZero();
    a_ref = calcPointAcceleration6D(*model_3dof, q, qdot, qddot, fb_id, point_local);
    calcPointJacobianDot6D(*model_3dof, q, qdot, fb_id, point_local, GDot);
    calcPointJacobian6D(*model_3dof, q, fb_id, point_local, G);
    a_exp = G * qddot + GDot * qdot;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_ref.linear(), a_exp.getLinearPart(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_ref.angular(), a_exp.getAngularPart(), unit_test_utils::E_MINUS_12));
}

TEST_F(Human36, CalcPointJacobianDot6DFrame)
{
    randomizeStates();
    updateKinematics(*model, q, qdot, qddot);
    unsigned int body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    Vector3d point_local(0., 0., 0.);
    ReferenceFramePtr body_frame = model->bodyFrames[body_id];

    MatrixNd G(6, model->qdot_size), GDot(6, model->qdot_size);
    G.setZero();
    GDot.setZero();
    calcPointJacobianDot6D(*model, q, qdot, body_frame, GDot, false);
    calcPointJacobian6D(*model, q, body_id, point_local, G, false);
    FrameVectorPair a_ref = calcPointAcceleration6D(*model, q, qdot, qddot, body_id, point_local, false);
    SpatialVector a_exp = G * qddot + GDot * qdot;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_ref.angular(), a_exp.getAngularPart(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_ref.linear(), a_exp.getLinearPart(), unit_test_utils::E_MINUS_12));

    updateKinematics(*model_3dof, q, qdot, qddot);
    body_id = (rand() % (model_3dof->mBodies.size() - 2)) + 1;
    body_frame = model_3dof->bodyFrames[body_id];
    G.setZero();
    GDot.setZero();
    a_ref = calcPointAcceleration6D(*model_3dof, q, qdot, qddot, body_id, point_local, false);
    calcPointJacobianDot6D(*model_3dof, q, qdot, body_frame, GDot, false);
    calcPointJacobian6D(*model_3dof, q, body_id, point_local, G, false);
    a_exp = G * qddot + GDot * qdot;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_ref.angular(), a_exp.getAngularPart(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_ref.linear(), a_exp.getLinearPart(), unit_test_utils::E_MINUS_12));

    Joint fj(JointTypeFixed);
    Body fb(1., Vector3d(0.1, 0.1, 0.1), Vector3d(0.1, 0.1, 0.1));
    body_id = (rand() % (model_3dof->mBodies.size() - 2)) + 1;
    unsigned int fb_id = model_3dof->addBody(body_id, Xtrans(Vector3d(-0.01, 0.02, 0.01)) * Xrotx(0.5), fj, fb, "fixed_body");

    updateKinematics(*model_3dof, q, qdot, qddot);

    G.setZero();
    GDot.setZero();
    body_frame = model_3dof->referenceFrameMap["fixed_body"];
    a_ref = calcPointAcceleration6D(*model_3dof, q, qdot, qddot, fb_id, point_local, false);
    calcPointJacobianDot6D(*model_3dof, q, qdot, body_frame, GDot, false);
    calcPointJacobian6D(*model_3dof, q, fb_id, point_local, G, false);
    a_exp = G * qddot + GDot * qdot;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_ref.angular(), a_exp.getAngularPart(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_ref.linear(), a_exp.getLinearPart(), unit_test_utils::E_MINUS_12));
}

TEST_F(Human36, CalcRelativePointJacobianDot6D)
{
    randomizeStates();
    updateKinematics(*model, q, qdot, qddot);
    unsigned int body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    unsigned int relative_body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    unsigned int expressed_in_body_id = (rand() % (model->mBodies.size() - 2)) + 1;

    MatrixNd G(6, model->qdot_size), GDot(6, model->qdot_size);
    G.setZero();
    GDot.setZero();
    ReferenceFramePtr body_frame = model->bodyFrames[body_id];
    ReferenceFramePtr relative_frame = model->bodyFrames[relative_body_id];
    ReferenceFramePtr expressedInFrame = model->bodyFrames[expressed_in_body_id];

    FrameVectorPair a = calcPointAcceleration6D(*model, q, qdot, qddot, body_frame, relative_frame, expressedInFrame, false);
    FrameVectorPair a_body = calcPointAcceleration6D(*model, q, qdot, qddot, body_frame, model->worldFrame, model->worldFrame, false);
    FrameVectorPair a_rel = calcPointAcceleration6D(*model, q, qdot, qddot, relative_frame, model->worldFrame, model->worldFrame, false);

    a_body -= a_rel;
    a_body.changeFrame(expressedInFrame);

    EXPECT_TRUE(unit_test_utils::checkFrameVectorPairEpsilonClose(a_body, a, unit_test_utils::E_MINUS_12));

    calcRelativePointJacobian6D(*model, q, G, body_frame, relative_frame, expressedInFrame, false);
    calcRelativePointJacobianDot6D(*model, q, qdot, GDot, body_frame, relative_frame, expressedInFrame, false);

    SpatialVector a_calc = G * qddot + GDot * qdot;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a.linear(), a_calc.getLinearPart(), unit_test_utils::E_MINUS_13));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a.angular(), a_calc.getAngularPart(), unit_test_utils::E_MINUS_13));

    updateKinematics(*model_3dof, q, qdot, qddot);
    body_id = (rand() % (model_3dof->mBodies.size() - 2)) + 1;
    relative_body_id = (rand() % (model_3dof->mBodies.size() - 2)) + 1;
    expressed_in_body_id = (rand() % (model_3dof->mBodies.size() - 2)) + 1;

    body_frame = model_3dof->bodyFrames[body_id];
    relative_frame = model_3dof->bodyFrames[relative_body_id];
    expressedInFrame = model_3dof->bodyFrames[expressed_in_body_id];

    a = calcPointAcceleration6D(*model_3dof, q, qdot, qddot, body_frame, relative_frame, expressedInFrame, false);
    a_body = calcPointAcceleration6D(*model_3dof, q, qdot, qddot, body_frame, model_3dof->worldFrame, model_3dof->worldFrame, false);
    a_rel = calcPointAcceleration6D(*model_3dof, q, qdot, qddot, relative_frame, model_3dof->worldFrame, model_3dof->worldFrame, false);

    a_body -= a_rel;
    a_body.changeFrame(expressedInFrame);

    EXPECT_TRUE(unit_test_utils::checkFrameVectorPairEpsilonClose(a_body, a, unit_test_utils::E_MINUS_12));

    G = MatrixNd::Zero(6, model_3dof->qdot_size);
    GDot = MatrixNd::Zero(6, model_3dof->qdot_size);
    G.setZero();
    GDot.setZero();
    calcRelativePointJacobian6D(*model_3dof, q, G, body_frame, relative_frame, expressedInFrame, false);
    calcRelativePointJacobianDot6D(*model_3dof, q, qdot, GDot, body_frame, relative_frame, expressedInFrame, false);

    a_calc = G * qddot + GDot * qdot;

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a.linear(), a_calc.getLinearPart(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a.angular(), a_calc.getAngularPart(), unit_test_utils::E_MINUS_14));

    Joint fj(JointTypeFixed);
    Body fb(1., Vector3d(0.1, 0.1, 0.1), Vector3d(0.1, 0.1, 0.1));
    body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    relative_body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    expressed_in_body_id = (rand() % (model->mBodies.size() - 2)) + 1;

    model->addBody(body_id, Xtrans(Vector3d(-0.01, 0.02, 0.01)) * Xrotx(0.5), fj, fb, "fixed_body");
    G.setZero();
    GDot.setZero();

    qdot.setZero();
    updateKinematics(*model, q, qdot, qddot);

    body_frame = model->referenceFrameMap["fixed_body"];
    relative_frame = model->bodyFrames[relative_body_id];
    expressedInFrame = model->bodyFrames[expressed_in_body_id];

    a = calcPointAcceleration6D(*model, q, qdot, qddot, body_frame, relative_frame, expressedInFrame, false);

    calcRelativePointJacobianDot6D(*model, q, qdot, GDot, body_frame, relative_frame, expressedInFrame, false);
    calcRelativePointJacobian6D(*model, q, G, body_frame, relative_frame, expressedInFrame, false);

    a_calc.setZero();
    a_calc = G * qddot + GDot * qdot;

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a.linear(), a_calc.getLinearPart(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a.angular(), a_calc.getAngularPart(), unit_test_utils::E_MINUS_14));
}

TEST_F(Human36, CalcRelativePointJacobianAndJacobianDot6D)
{
    randomizeStates();
    updateKinematics(*model, q, qdot, qddot);
    unsigned int body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    unsigned int relative_body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    unsigned int expressed_in_body_id = (rand() % (model->mBodies.size() - 2)) + 1;

    MatrixNd G(6, model->qdot_size), GDot(6, model->qdot_size), G2(6, model->qdot_size), GDot2(6, model->qdot_size);
    G.setZero();
    GDot.setZero();
    G2.setZero();
    GDot2.setZero();
    ReferenceFramePtr body_frame = model->bodyFrames[body_id];
    ReferenceFramePtr relative_frame = model->bodyFrames[relative_body_id];
    ReferenceFramePtr expressedInFrame = model->bodyFrames[expressed_in_body_id];

    calcRelativePointJacobian6D(*model, q, G, body_frame, relative_frame, expressedInFrame, false);
    calcRelativePointJacobianDot6D(*model, q, qdot, GDot, body_frame, relative_frame, expressedInFrame, false);
    calcRelativePointJacobianAndJacobianDot6D(*model, q, qdot, G2, GDot2, body_frame, relative_frame, expressedInFrame, false);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(G, G2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDot, GDot2, unit_test_utils::E_MINUS_14));

    updateKinematics(*model_3dof, q, qdot, qddot);
    body_id = (rand() % (model_3dof->mBodies.size() - 2)) + 1;
    relative_body_id = (rand() % (model_3dof->mBodies.size() - 2)) + 1;
    expressed_in_body_id = (rand() % (model_3dof->mBodies.size() - 2)) + 1;

    body_frame = model_3dof->bodyFrames[body_id];
    relative_frame = model_3dof->bodyFrames[relative_body_id];
    expressedInFrame = model_3dof->bodyFrames[expressed_in_body_id];

    G = MatrixNd::Zero(6, model_3dof->qdot_size);
    GDot = MatrixNd::Zero(6, model_3dof->qdot_size);
    G2 = MatrixNd::Zero(6, model_3dof->qdot_size);
    GDot2 = MatrixNd::Zero(6, model_3dof->qdot_size);

    calcRelativePointJacobian6D(*model_3dof, q, G, body_frame, relative_frame, expressedInFrame, false);
    calcRelativePointJacobianDot6D(*model_3dof, q, qdot, GDot, body_frame, relative_frame, expressedInFrame, false);
    calcRelativePointJacobianAndJacobianDot6D(*model_3dof, q, qdot, G2, GDot2, body_frame, relative_frame, expressedInFrame, false);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(G, G2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDot, GDot2, unit_test_utils::E_MINUS_14));

    Joint fj(JointTypeFixed);
    Body fb(1., Vector3d(0.1, 0.1, 0.1), Vector3d(0.1, 0.1, 0.1));
    body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    relative_body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    expressed_in_body_id = (rand() % (model->mBodies.size() - 2)) + 1;

    model->addBody(body_id, Xtrans(Vector3d(-0.01, 0.02, 0.01)) * Xrotx(0.5), fj, fb, "fixed_body");
    G.setZero();
    GDot.setZero();
    G2.setZero();
    GDot2.setZero();

    updateKinematics(*model, q, qdot, qddot);

    body_frame = model->referenceFrameMap["fixed_body"];
    relative_frame = model->bodyFrames[relative_body_id];
    expressedInFrame = model->bodyFrames[expressed_in_body_id];

    calcRelativePointJacobianDot6D(*model, q, qdot, GDot, body_frame, relative_frame, expressedInFrame, false);
    calcRelativePointJacobian6D(*model, q, G, body_frame, relative_frame, expressedInFrame, false);
    calcRelativePointJacobianAndJacobianDot6D(*model, q, qdot, G2, GDot2, body_frame, relative_frame, expressedInFrame, false);

    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(G, G2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkMatrixNdEpsilonClose(GDot, GDot2, unit_test_utils::E_MINUS_14));
}

TEST_F(Human36, CalcPointAcceleration6DFixedFrames)
{
    randomizeStates();
    updateKinematics(*model, q, qdot, qddot);
    unsigned int body_id = (rand() % (model->mBodies.size() - 2)) + 1;

    MatrixNd G(6, model->qdot_size), GDot(6, model->qdot_size);
    G.setZero();
    GDot.setZero();

    Joint fj(JointTypeFixed);
    Body fb(1., Vector3d(0.1, 0.1, 0.1), Vector3d(0.1, 0.1, 0.1));
    body_id = (rand() % (model->mBodies.size() - 2)) + 1;

    unsigned int fb_id_1 = model->addBody(body_id, Xtrans(Vector3d(-0.01, 0.02, 0.01)) * Xrotx(0.5), fj, fb, "fixed_body_1");
    body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    unsigned int fb_id_2 = model->addBody(body_id, Xtrans(Vector3d(-0.01, 0.02, 0.01)) * Xrotx(0.5), fj, fb, "fixed_body_2");
    body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    model->addBody(body_id, Xtrans(Vector3d(-0.01, 0.02, 0.01)) * Xrotx(0.5), fj, fb, "fixed_body_3");

    G.setZero();
    GDot.setZero();

    qdot.setZero();
    updateKinematics(*model, q, qdot, qddot);

    ReferenceFramePtr fb_1_frame = model->referenceFrameMap["fixed_body_1"];
    ReferenceFramePtr fb_2_frame = model->referenceFrameMap["fixed_body_2"];
    ReferenceFramePtr fb_3_frame = model->referenceFrameMap["fixed_body_3"];

    FrameVectorPair a_fb_1 = calcPointAcceleration6D(*model, q, qdot, qddot, fb_id_1, Vector3dZero, false);
    FrameVectorPair a_fb_2 = calcPointAcceleration6D(*model, q, qdot, qddot, fb_id_2, Vector3dZero, false);

    FrameVectorPair a_fb_1_frames = calcPointAcceleration6D(*model, q, qdot, qddot, fb_1_frame, model->worldFrame, model->worldFrame, false);
    FrameVectorPair a_fb_2_frames = calcPointAcceleration6D(*model, q, qdot, qddot, fb_2_frame, model->worldFrame, model->worldFrame, false);

    EXPECT_TRUE(unit_test_utils::checkFrameVectorPairEpsilonClose(a_fb_1_frames, a_fb_1, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkFrameVectorPairEpsilonClose(a_fb_2_frames, a_fb_2, unit_test_utils::E_MINUS_14));

    FrameVectorPair a = calcPointAcceleration6D(*model, q, qdot, qddot, fb_1_frame, fb_2_frame, fb_3_frame, false);

    a_fb_1_frames -= a_fb_2_frames;
    a_fb_1_frames.changeFrame(fb_3_frame);

    SpatialTransform X = model->worldFrame->getTransformToDesiredFrame(fb_3_frame);
    X.r.setZero();

    a_fb_1 -= a_fb_2;
    a_fb_1.changeFrame(fb_3_frame);

    EXPECT_TRUE(unit_test_utils::checkFrameVectorPairEpsilonClose(a_fb_1_frames, a_fb_1, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkFrameVectorPairEpsilonClose(a_fb_1_frames, a, unit_test_utils::E_MINUS_14));
}

TEST_F(Human36, CalcPointAccelerationDot6DFrames)
{
    randomizeStates();
    updateKinematics(*model, q, qdot, qddot);
    unsigned int body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    ReferenceFramePtr body_frame = model->bodyFrames[body_id];
    FrameVectorPair a_ref = calcPointAcceleration6D(*model, q, qdot, qddot, body_id, Vector3dZero, false);

    body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    ReferenceFramePtr relative_frame = model->bodyFrames[body_id];
    FrameVectorPair a_ref2 = calcPointAcceleration6D(*model, q, qdot, qddot, body_id, Vector3dZero, false);

    FrameVectorPair a_beep = calcPointAcceleration6D(*model, q, qdot, qddot, body_frame, relative_frame, model->worldFrame, false);

    EXPECT_TRUE(unit_test_utils::checkFrameVectorPairEpsilonClose(a_ref - a_ref2, a_beep, unit_test_utils::E_MINUS_14));

    body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    ReferenceFramePtr expressedInFrame = model->bodyFrames[body_id];

    a_beep = calcPointAcceleration6D(*model, q, qdot, qddot, body_frame, relative_frame, expressedInFrame, false);
    SpatialTransform X_rot = model->worldFrame->getTransformToDesiredFrame(expressedInFrame);
    X_rot.r.setZero();

    a_ref -= a_ref2;

    a_ref.setLinearPart(a_ref.linear().transform_copy(X_rot));
    a_ref.setAngularPart(a_ref.angular().transform_copy(X_rot));
    a_ref.setReferenceFrame(expressedInFrame);

    EXPECT_TRUE(unit_test_utils::checkFrameVectorPairEpsilonClose(a_ref, a_beep, unit_test_utils::E_MINUS_12));
}

TEST_F(Human36, CalcPointAccelerationDot6DFramesModel3Dof)
{
    randomizeStates();
    updateKinematics(*model_3dof, q, qdot, qddot);
    unsigned int body_id = (rand() % (model_3dof->mBodies.size() - 2)) + 1;
    ReferenceFramePtr body_frame = model_3dof->bodyFrames[body_id];
    FrameVectorPair a_ref = calcPointAcceleration6D(*model_3dof, q, qdot, qddot, body_id, Vector3dZero, false);

    body_id = (rand() % (model_3dof->mBodies.size() - 2)) + 1;
    ReferenceFramePtr relative_frame = model_3dof->bodyFrames[body_id];
    FrameVectorPair a_ref2 = calcPointAcceleration6D(*model_3dof, q, qdot, qddot, body_id, Vector3dZero, false);

    FrameVectorPair a_beep = calcPointAcceleration6D(*model_3dof, q, qdot, qddot, body_frame, relative_frame, model_3dof->worldFrame, false);
    EXPECT_TRUE(unit_test_utils::checkFrameVectorPairEpsilonClose(a_ref - a_ref2, a_beep, unit_test_utils::E_MINUS_12));

    body_id = (rand() % (model_3dof->mBodies.size() - 2)) + 1;
    ReferenceFramePtr expressedInFrame = model_3dof->bodyFrames[body_id];

    a_beep = calcPointAcceleration6D(*model_3dof, q, qdot, qddot, body_frame, relative_frame, expressedInFrame, false);
    SpatialTransform X_rot = model_3dof->worldFrame->getTransformToDesiredFrame(expressedInFrame);
    X_rot.r.setZero();

    a_ref -= a_ref2;

    a_ref.setLinearPart(a_ref.linear().transform_copy(X_rot));
    a_ref.setAngularPart(a_ref.angular().transform_copy(X_rot));
    a_ref.setReferenceFrame(expressedInFrame);

    EXPECT_TRUE(unit_test_utils::checkFrameVectorPairEpsilonClose(a_ref, a_beep, unit_test_utils::E_MINUS_12));
}

TEST_F(Human36, CalcPointAccelerationDot)
{
    randomizeStates();
    unsigned int body_id = (rand() % (model->mBodies.size() - 2)) + 1;
    Vector3d point_local(1.1, 2.2, 3.3);

    MatrixNd G(3, model->qdot_size), GDot(3, model->qdot_size);
    G.setZero();
    GDot.setZero();
    calcPointJacobianDot(*model, q, qdot, body_id, point_local, GDot);
    calcPointJacobian(*model, q, body_id, point_local, G);
    FrameVector a_ref = calcPointAcceleration(*model, q, qdot, qddot, body_id, point_local);
    Vector3d a_exp = G * qddot + GDot * qdot;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_ref, a_exp, unit_test_utils::E_MINUS_12));

    body_id = (rand() % (model_3dof->mBodies.size() - 2)) + 1;
    G.setZero();
    GDot.setZero();
    a_ref = calcPointAcceleration(*model_3dof, q, qdot, qddot, body_id, point_local);
    calcPointJacobianDot(*model_3dof, q, qdot, body_id, point_local, GDot);
    calcPointJacobian(*model_3dof, q, body_id, point_local, G);
    a_exp = G * qddot + GDot * qdot;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_ref, a_exp, unit_test_utils::E_MINUS_12));

    Joint fj(JointTypeFixed);
    Body fb(1., Vector3d(0.1, 0.1, 0.1), Vector3d(0.1, 0.1, 0.1));
    body_id = (rand() % (model_3dof->mBodies.size() - 2)) + 1;
    unsigned int fb_id = model_3dof->addBody(body_id, Xtrans(Vector3d(-0.01, 0.02, 0.01)) * Xrotx(0.5), fj, fb);
    G.setZero();
    GDot.setZero();
    a_ref = calcPointAcceleration(*model_3dof, q, qdot, qddot, fb_id, point_local);
    calcPointJacobianDot(*model_3dof, q, qdot, fb_id, point_local, GDot);
    calcPointJacobian(*model_3dof, q, fb_id, point_local, G);
    a_exp = G * qddot + GDot * qdot;
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_ref, a_exp, unit_test_utils::E_MINUS_12));
}

TEST_F(Human36, CalcPointAcceleration)
{
    randomizeStates();
    unsigned int foot_r_id = model->GetBodyId("foot_r");
    Vector3d point_local(1.1, 2.2, 3.3);

    //     Compute the 6-D acceleration
    FrameVectorPair a_foot_0 = calcPointAcceleration6D(*model, q, qdot, qddot, foot_r_id, point_local);
    Vector3d a_v = calcPointAcceleration(*model, q, qdot, qddot, foot_r_id, point_local);

    EXPECT_NEAR(a_v(0), a_foot_0.linear().x(), unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(a_v(1), a_foot_0.linear().y(), unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(a_v(2), a_foot_0.linear().z(), unit_test_utils::E_MINUS_14);

    // Compute the 6-D acceleration by adding the coriolis term to the
    // acceleration of the body and transforming the result to the
    // point and align it with the base coordinate system.
    FramePoint p(model->bodyFrames[foot_r_id], point_local);
    p.changeFrame(ReferenceFrame::getWorldFrame());

    FrameVector v_foot_0 = calcPointVelocity(*model, q, qdot, foot_r_id, point_local);
    SpatialVector rdot(0., 0., 0., v_foot_0[0], v_foot_0[1], v_foot_0[2]);

    SpatialTransform X_foot(Matrix3d::Identity(), p.vec());
    SpatialVector a_foot_0_ref = X_foot.apply(model->bodyFrames[foot_r_id]->getTransformToRoot().apply(model->a[foot_r_id]) -
                                              rdot.crossm(model->bodyFrames[foot_r_id]->getTransformToRoot().apply(model->v[foot_r_id])));

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_foot_0_ref.getLinearPart(), a_foot_0.linear(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_foot_0_ref.getAngularPart(), a_foot_0.angular(), unit_test_utils::E_MINUS_12));
}

TEST_F(Human36, CalcPointAccelerationFixedBody)
{
    randomizeStates();
    unsigned int foot_r_id = model->GetBodyId("foot_r");
    Body fb(1., Math::Vector3d(1., 1., 1.), Math::Vector3d(1., 1., 1.));
    Joint fj(JointTypeFixed);
    unsigned int fb_id = model->addBody(foot_r_id, Math::SpatialTransform(), fj, fb);
    Vector3d point_local(1.1, 2.2, 3.3);

    // Compute the 6-D acceleration
    FrameVectorPair a_foot_0 = calcPointAcceleration6D(*model, q, qdot, qddot, fb_id, point_local);
    Vector3d a_v = calcPointAcceleration(*model, q, qdot, qddot, fb_id, point_local);

    EXPECT_NEAR(a_v(0), a_foot_0.linear().x(), unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(a_v(1), a_foot_0.linear().y(), unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(a_v(2), a_foot_0.linear().z(), unit_test_utils::E_MINUS_14);

    // Compute the 6-D acceleration by adding the coriolis term to the
    // acceleration of the body and transforming the result to the
    // point and align it with the base coordinate system.
    FramePoint p(model->fixedBodyFrames[fb_id - model->fixed_body_discriminator], point_local);
    p.changeFrame(ReferenceFrame::getWorldFrame());

    FrameVector v_foot_0 = calcPointVelocity(*model, q, qdot, fb_id, point_local);
    SpatialVector rdot(0., 0., 0., v_foot_0[0], v_foot_0[1], v_foot_0[2]);

    SpatialTransform X_foot(Matrix3d::Identity(), p.vec());
    SpatialVector a_foot_0_ref =
        X_foot.apply(model->fixedBodyFrames[fb_id - model->fixed_body_discriminator]->getTransformToRoot().apply(model->a[foot_r_id]) -
                     rdot.crossm(model->fixedBodyFrames[fb_id - model->fixed_body_discriminator]->getTransformToRoot().apply(model->v[foot_r_id])));

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_foot_0_ref.getLinearPart(), a_foot_0.linear(), unit_test_utils::E_MINUS_12));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(a_foot_0_ref.getAngularPart(), a_foot_0.angular(), unit_test_utils::E_MINUS_12));
}

TEST_F(RdlKinematicsSingleChainFixture, calcSpatialVelocity)
{
    Model model;
    Body b1(1., Math::Vector3d(0., 0., -0.1), Math::Vector3d(1., 1., 1.));
    Joint j1(JointTypeRevoluteX);

    unsigned int id = model.addBody(0, Math::SpatialTransform(), j1, b1, "b1");

    id = model.addBody(id, Xtrans(Math::Vector3d(0., 0., -1.)), j1, b1, "b2");

    Body fb1(0., Math::Vector3d(0., 0., 0.), Math::Vector3d(0., 0., 0.));
    Joint fj1(JointTypeFixed);

    unsigned int fb_id = model.addBody(id, Xtrans(Math::Vector3d(0., 0., -1.)), fj1, fb1, "fb1");

    Math::VectorNd Q(model.qdot_size), QDot(model.qdot_size);

    QDot[0] = 1.;
    updateKinematicsCustom(model, &Q, &QDot, nullptr);
    SpatialMotion m = calcSpatialVelocity(model, Q, QDot, 2, 0);
    SpatialMotion m_frame = calcSpatialVelocity(model, Q, QDot, model.bodyFrames[2], model.worldFrame);
    EXPECT_STREQ(m.getReferenceFrame()->getName().c_str(), "b2");
    EXPECT_STREQ(m.getBaseFrame()->getName().c_str(), "world");
    EXPECT_STREQ(m.getReferenceFrame()->getName().c_str(), "b2");

    EXPECT_STREQ(m_frame.getReferenceFrame()->getName().c_str(), "b2");
    EXPECT_STREQ(m_frame.getBaseFrame()->getName().c_str(), "world");
    EXPECT_STREQ(m_frame.getReferenceFrame()->getName().c_str(), "b2");

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m, SpatialVector(1., 0., 0., 0., 1., 0.), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m_frame, SpatialVector(1., 0., 0., 0., 1., 0.), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));

    m = calcSpatialVelocity(model, Q, QDot, fb_id, 0);
    m_frame = calcSpatialVelocity(model, Q, QDot, model.fixedBodyFrames[fb_id - model.fixed_body_discriminator], model.worldFrame);
    EXPECT_STREQ(m.getReferenceFrame()->getName().c_str(), "fb1");
    EXPECT_STREQ(m.getBaseFrame()->getName().c_str(), "world");
    EXPECT_STREQ(m.getReferenceFrame()->getName().c_str(), "fb1");

    EXPECT_STREQ(m_frame.getReferenceFrame()->getName().c_str(), "fb1");
    EXPECT_STREQ(m_frame.getBaseFrame()->getName().c_str(), "world");
    EXPECT_STREQ(m_frame.getReferenceFrame()->getName().c_str(), "fb1");

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m, SpatialVector(1., 0., 0., 0., 2., 0.), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m_frame, SpatialVector(1., 0., 0., 0., 2., 0.), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));

    m = calcSpatialVelocity(model, Q, QDot, 0, 2);
    m_frame = calcSpatialVelocity(model, Q, QDot, model.worldFrame, model.bodyFrames[2]);
    EXPECT_STREQ(m.getReferenceFrame()->getName().c_str(), "world");
    EXPECT_STREQ(m.getBaseFrame()->getName().c_str(), "b2");
    EXPECT_STREQ(m.getReferenceFrame()->getName().c_str(), "world");

    EXPECT_STREQ(m_frame.getReferenceFrame()->getName().c_str(), "world");
    EXPECT_STREQ(m_frame.getBaseFrame()->getName().c_str(), "b2");
    EXPECT_STREQ(m_frame.getReferenceFrame()->getName().c_str(), "world");

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(-m, MotionVector(1., 0., 0., 0., 1., 0.).transform_copy(model.bodyFrames[2]->getTransformToRoot()),
                                                                 unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(-m_frame, MotionVector(1., 0., 0., 0., 1., 0.).transform_copy(model.bodyFrames[2]->getTransformToRoot()),
                                                                 unit_test_utils::E_MINUS_14));

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m.transform_copy(model.worldFrame->getTransformToDesiredFrame(model.bodyFrames[2])),
                                                                 -MotionVector(1., 0., 0., 0., 1., 0.), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m_frame.transform_copy(model.worldFrame->getTransformToDesiredFrame(model.bodyFrames[2])),
                                                                 -MotionVector(1., 0., 0., 0., 1., 0.), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));

    for (unsigned int i = 0; i < QDot.size(); i++)
    {
        Q[i] = 0.4 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        QDot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    }

    updateKinematicsCustom(model, &Q, &QDot, nullptr);
    MotionVector m1 = calcSpatialVelocity(model, Q, QDot, 2, 0);
    MotionVector m2 = calcSpatialVelocity(model, Q, QDot, 0, 2);

    MotionVector m1_frame = calcSpatialVelocity(model, Q, QDot, model.bodyFrames[2], model.worldFrame);
    MotionVector m2_frame = calcSpatialVelocity(model, Q, QDot, model.worldFrame, model.bodyFrames[2]);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m1.transform_copy(model.bodyFrames[2]->getTransformToRoot()), -m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(
        unit_test_utils::checkSpatialVectorsEpsilonClose(m1_frame.transform_copy(model.bodyFrames[2]->getTransformToRoot()), -m2_frame, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));

    m1 = calcSpatialVelocity(model, Q, QDot, fb_id, 0);
    m2 = calcSpatialVelocity(model, Q, QDot, 0, fb_id);

    m1_frame = calcSpatialVelocity(model, Q, QDot, model.fixedBodyFrames[fb_id - model.fixed_body_discriminator], model.worldFrame);
    m2_frame = calcSpatialVelocity(model, Q, QDot, model.worldFrame, model.fixedBodyFrames[fb_id - model.fixed_body_discriminator]);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m1.transform_copy(model.fixedBodyFrames[0]->getTransformToRoot()), -m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m1_frame.transform_copy(model.fixedBodyFrames[0]->getTransformToRoot()), -m2_frame,
                                                                 unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));

    Body fb2(1., Math::Vector3d(0.1, -0.1, 0.2), Math::Vector3d(1., 1., 1.));
    Joint fj2(JointTypeFixed);

    unsigned int fb_id2 = model.addBody(1, Xtrans(Math::Vector3d(0.1, 0.2, 0.1)) * Xrotx(0.1), fj2, fb2, "fb2");

    m = calcSpatialVelocity(model, Q, QDot, fb_id, fb_id2);
    m_frame = calcSpatialVelocity(model, Q, QDot, model.fixedBodyFrames[fb_id - model.fixed_body_discriminator],
                                  model.fixedBodyFrames[fb_id2 - model.fixed_body_discriminator]);
    m1 = m;
    m1_frame = m_frame;

    EXPECT_STREQ(m.getReferenceFrame()->getName().c_str(), "fb1");
    EXPECT_STREQ(m.getBaseFrame()->getName().c_str(), "fb2");
    EXPECT_STREQ(m.getReferenceFrame()->getName().c_str(), "fb1");
    EXPECT_STREQ(m_frame.getReferenceFrame()->getName().c_str(), "fb1");
    EXPECT_STREQ(m_frame.getBaseFrame()->getName().c_str(), "fb2");
    EXPECT_STREQ(m_frame.getReferenceFrame()->getName().c_str(), "fb1");
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));

    m = calcSpatialVelocity(model, Q, QDot, fb_id2, fb_id);
    m_frame = calcSpatialVelocity(model, Q, QDot, model.fixedBodyFrames[fb_id2 - model.fixed_body_discriminator],
                                  model.fixedBodyFrames[fb_id - model.fixed_body_discriminator]);
    EXPECT_STREQ(m.getReferenceFrame()->getName().c_str(), "fb2");
    EXPECT_STREQ(m.getBaseFrame()->getName().c_str(), "fb1");
    EXPECT_STREQ(m.getReferenceFrame()->getName().c_str(), "fb2");
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));

    m2 = m;
    m2_frame = m_frame;

    EXPECT_TRUE(
        unit_test_utils::checkSpatialVectorsEpsilonClose(m1.transform_copy(model.fixedBodyFrames[fb_id - model.fixed_body_discriminator]->getTransformToDesiredFrame(
                                                             model.fixedBodyFrames[fb_id2 - model.fixed_body_discriminator])),
                                                         -m2, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(
        m1_frame.transform_copy(
            model.fixedBodyFrames[fb_id - model.fixed_body_discriminator]->getTransformToDesiredFrame(model.fixedBodyFrames[fb_id2 - model.fixed_body_discriminator])),
        -m2_frame, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlKinematicsSingleChainFixture, calcBodySpatialJacobianDot)
{
    Model model;
    Body b1(1., Math::Vector3d(0.1, -0.1, 0.1), Math::Vector3d(1., 1., 1.));
    Joint j_rev_x(JointTypeRevoluteX);
    Joint j_rev_y(JointTypeRevoluteY);
    Joint j_rev_z(JointTypeRevoluteZ);

    unsigned int parent_id = 0;

    parent_id = model.addBody(parent_id, Math::Xtrans(Math::Vector3d(0.1, -0.2, 0.15)), j_rev_x, b1, "b1");

    parent_id = model.addBody(parent_id, Math::Xtrans(Math::Vector3d(0.1, -0.2, 0.15)), j_rev_y, b1, "b2");

    parent_id = model.addBody(parent_id, Math::Xtrans(Math::Vector3d(0.1, -0.2, 0.15)), j_rev_z, b1, "b3");

    Math::VectorNd Q(model.qdot_size), QDot(model.qdot_size), QDDot(model.qdot_size);
    Math::MatrixNd GDot(6, model.qdot_size), G(6, model.qdot_size);
    Q.setZero();
    QDot.setZero();
    QDDot.setZero();
    GDot.setZero();

    for (unsigned int i = 0; i < model.qdot_size; i++)
    {
        Q[i] = 0.4 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        QDot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        QDDot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    }

    updateKinematics(model, Q, QDot, QDDot);

    calcBodySpatialJacobianDot(model, Q, QDot, parent_id, GDot, false);
    calcBodySpatialJacobian(model, Q, parent_id, G, false);

    Math::SpatialVector a_exp = GDot * QDot + G * QDDot;

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(a_exp, model.a[parent_id], unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));

    Joint j_fixed(JointTypeFixed);

    parent_id = model.addBody(parent_id, Math::Xtrans(Math::Vector3d(-0.1, 0.2, 0.1)), j_fixed, b1, "fb1");

    updateKinematics(model, Q, QDot, QDDot);

    calcBodySpatialJacobianDot(model, Q, QDot, parent_id, GDot, false);
    calcBodySpatialJacobian(model, Q, parent_id, G, false);

    a_exp = GDot * QDot + G * QDDot;
    Math::SpatialAcceleration a_fixed = model.a[model.mFixedBodies[parent_id - model.fixed_body_discriminator].mMovableParent];
    a_fixed.changeFrame(model.fixedBodyFrames[parent_id - model.fixed_body_discriminator]);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(a_exp, a_fixed, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));
}

TEST_F(RdlKinematicsSingleChainFixture, calcSpatialAcceleration)
{
    Model model;
    Body b1(1., Math::Vector3d(0.1, -0.1, 0.1), Math::Vector3d(1., 1., 1.));
    Joint j_rev_x(JointTypeRevoluteX);
    Joint j_rev_y(JointTypeRevoluteY);
    Joint j_rev_z(JointTypeRevoluteZ);

    unsigned int parent_id = 0;

    parent_id = model.addBody(parent_id, Math::Xtrans(Math::Vector3d(0.1, -0.2, 0.15)), j_rev_x, b1, "b1");

    parent_id = model.addBody(parent_id, Math::Xtrans(Math::Vector3d(0.1, -0.2, 0.15)), j_rev_y, b1, "b2");

    parent_id = model.addBody(parent_id, Math::Xtrans(Math::Vector3d(0.1, -0.2, 0.15)), j_rev_z, b1, "b3");

    Math::VectorNd Q(model.qdot_size), QDot(model.qdot_size), QDDot(model.qdot_size);
    Math::MatrixNd GDot(6, model.qdot_size), G(6, model.qdot_size);
    Q.setZero();
    QDot.setZero();
    QDDot.setZero();
    GDot.setZero();

    for (unsigned int i = 0; i < model.qdot_size; ++i)
    {
        Q[i] = 0.4 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        QDot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        QDDot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    }

    updateKinematics(model, Q, QDot, QDDot);

    calcBodySpatialJacobianDot(model, Q, QDot, parent_id, GDot, false);
    calcBodySpatialJacobian(model, Q, parent_id, G, false);

    SpatialAcceleration a_calc = calcSpatialAcceleration(model, Q, QDot, QDDot, parent_id, 0);

    Math::SpatialVector a_exp = GDot * QDot + G * QDDot;

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(a_exp, a_calc, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(a_exp, model.a[parent_id], unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));

    Joint j_fixed(JointTypeFixed);

    parent_id = model.addBody(parent_id, Math::Xtrans(Math::Vector3d(-0.1, 0.2, 0.1)), j_fixed, b1, "fb1");

    updateKinematics(model, Q, QDot, QDDot);

    calcBodySpatialJacobianDot(model, Q, QDot, parent_id, GDot, false);
    calcBodySpatialJacobian(model, Q, parent_id, G, false);
    a_calc = calcSpatialAcceleration(model, Q, QDot, QDDot, parent_id, 0);

    a_exp = GDot * QDot + G * QDDot;
    Math::SpatialAcceleration a_fixed = model.a[model.mFixedBodies[parent_id - model.fixed_body_discriminator].mMovableParent];
    a_fixed.changeFrame(model.fixedBodyFrames[parent_id - model.fixed_body_discriminator]);
    a_fixed.setBodyFrame(model.fixedBodyFrames[parent_id - model.fixed_body_discriminator]);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(a_exp, a_fixed, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(a_exp, a_calc, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkModelZeroVectorsAndMatrices(model));
}

TEST_F(Human36, calcBodySpatialJacobianDot)
{
    randomizeStates();

    updateKinematics(*model_emulated, q, qdot, qddot);

    Math::MatrixNd GDot(6, model_emulated->qdot_size), G(6, model_emulated->qdot_size);
    GDot.setZero();
    G.setZero();

    int id = rand() % static_cast<int>(model->mBodies.size() - 1);
    calcBodySpatialJacobianDot(*model_emulated, q, qdot, id, GDot, false);
    calcBodySpatialJacobian(*model_emulated, q, id, G, false);

    Math::SpatialVector a_exp = GDot * qdot + G * qddot;

    EXPECT_TRUE(a_exp.isApprox(model_emulated->a[id], unit_test_utils::E_MINUS_14));

    id = rand() % static_cast<int>(model_3dof->mBodies.size() - 1);

    updateKinematics(*model_3dof, q, qdot, qddot);

    GDot.setZero();
    G.setZero();

    calcBodySpatialJacobianDot(*model_3dof, q, qdot, id, GDot, false);
    calcBodySpatialJacobian(*model_3dof, q, id, G, false);

    a_exp = GDot * qdot + G * qddot;

    EXPECT_TRUE(a_exp.isApprox(model_3dof->a[id], unit_test_utils::E_MINUS_14));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
