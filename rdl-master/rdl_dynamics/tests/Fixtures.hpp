// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include "rdl_dynamics/Model.hpp"
#include "rdl_dynamics/Contacts.hpp"
#include <gtest/gtest.h>

using namespace RobotDynamics;
using namespace RobotDynamics::Math;

class FixedBase3DoFPlanar : public testing::Test
{
  public:
    FixedBase3DoFPlanar()
    {
        srand(time(NULL));
    };

    void randomizeStates()
    {
        Q = VectorNd(model->q_size);
        QDot = VectorNd(model->qdot_size);
        QDDot = VectorNd(model->qdot_size);
        Tau = VectorNd(model->qdot_size);

        for (int i = 0; i < Q.size(); i++)
        {
            Q[i] = 0.4 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            QDot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            Tau[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            QDDot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        }
    }

    void SetUp()
    {
        model = new Model;

        /* Basically a model like this, where there are 3 planar
         * joints rotating about the Y-axis. Link COM's are in
         * the geometric center of the links.
         *
         * ---------------------------- X - axis
         *             * - Y
         *             |
         *             |   body_a length - L1 = 1m
         *             |
         *             * - Y
         *             |
         *             |
         *             |
         *             |   body_b length - L2 = 2m
         *             |
         *             |
         *             * - Y
         *             |
         *             |
         *             |
         *             |
         *             |   body_c length - L3 = 3m
         *             |
         *             |
         *             |
         *             |
         *
         *
         */

        com_a = Vector3d(0., 0., -0.5);
        body_a = Body(6., com_a, Vector3d(1.2, 1.3, 1.4));
        joint_a = Joint(SpatialVector(0., 1., 0., 0., 0., 0.));

        body_a_id = model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a, "body_a");

        com_b = Vector3d(0., 0., -1.);
        body_b = Body(3., com_b, Vector3d(1., 1., 1.));
        joint_b = Joint(SpatialVector(0., 1., 0., 0., 0., 0.));

        body_b_id = model->addBody(1, Xtrans(Vector3d(0., 0., -1.)), joint_b, body_b, "body_b");

        com_c = Vector3d(0., 0., -1.5);
        body_c = Body(4., com_c, Vector3d(1., 1., 1.));
        joint_c = Joint(SpatialVector(0., 1., 0., 0., 0., 0.));

        body_c_id = model->addBody(2, Xtrans(Vector3d(0., 0., -2.)), joint_c, body_c, "body_c");

        Joint fixed_joint(JointType::JointTypeFixed);
        Vector3d r_fixed = Vector3d(0.1, 0.1, 0.1);
        X_fixed_c = Xtrans(Vector3d(0.02, 0.01, -0.25));
        X_fixed_c2 = Xtrans(Vector3d(-0.04, -0.02, 0.5));
        fixed_body = RobotDynamics::Body(0.1, r_fixed, Vector3d(0.1, 0.1, 0.1));
        fixed_body_c_id = model->addBody(body_c_id, X_fixed_c, fixed_joint, fixed_body, "fixed_body_c");
        fixed_body_c_2_id = model->addBody(fixed_body_c_id, X_fixed_c2, fixed_joint, fixed_body, "fixed_body_c_2");

        Q = VectorNd(model->q_size);
        QDot = VectorNd(model->qdot_size);
        QDDot = VectorNd(model->qdot_size);
        Tau = VectorNd(model->qdot_size);
    }

    void TearDown()
    {
        delete model;
    }

    RobotDynamics::Model* model;

    unsigned int body_a_id, body_b_id, body_c_id, ref_body_id, fixed_body_c_id, fixed_body_c_2_id;
    RobotDynamics::Body body_a, body_b, body_c, fixed_body;
    RobotDynamics::Joint joint_a, joint_b, joint_c;
    Vector3d com_a, com_b, com_c;
    SpatialTransform X_fixed_c, X_fixed_c2;

    RobotDynamics::Math::VectorNd Q;
    RobotDynamics::Math::VectorNd QDot;
    RobotDynamics::Math::VectorNd QDDot;
    RobotDynamics::Math::VectorNd Tau;
};

class FixedBaseTwoChain6DoF3D : public testing::Test
{
  public:
    FixedBaseTwoChain6DoF3D(){};

    void SetUp()
    {
        model = new Model;

        /* Two kinematic chains of 3 links.COM's are in
         * the geometric centers of the links.
         *
         *                         /
         *                        /
         *                       /  body_f length - L6 = 1m
         *                      Z
         *                     /
         *                    * - neg-X -----------
         *                    |
         *                    |
         *                    |  body_e length - L5 = 1m
         *                    Z
         *                    |
         *                    * - neg-X ------------
         *                   /
         *                  /
         *                 /  body_d length - L4 = 2m
         *                /
         *               X
         *              /
         * origin/rotx * - neg-Y ----------
         *             |
         *             |   body_a length - L1 = 1m
         *             |
         *      rotx   * - Z ----------
         *             |
         *           neg-Y
         *             |
         *             |   body_b length - L2 = 2m
         *             |
         *             |
         *      rotx   * - Y ----------
         *             |
         *             Z
         *             |
         *             |
         *             |   body_c length - L3 = 3m
         *             |
         *             |
         *             |
         *             |
         *
         *
         */
        SpatialTransform X;

        body_a = Body(1., Vector3d(0., 0., -0.5), Vector3d(1., 1., 1.));
        joint_a = Joint(SpatialVector(1., 0., 0., 0., 0., 0.));

        body_a_id = model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a, "body_a");

        body_b = Body(1., Vector3d(0., -1., 0.), Vector3d(1., 1., 1.));
        joint_b = Joint(SpatialVector(1., 0., 0., 0., 0., 0.));

        X = Xrotx(M_PI_2);
        X.r = Vector3d(0., 0., -1.);

        body_b_id = model->addBody(body_a_id, X, joint_b, body_b, "body_b");

        body_c = Body(1., Vector3d(0., 0., 1.5), Vector3d(1., 1., 1.));
        joint_c = Joint(SpatialVector(1., 0., 0., 0., 0., 0.));

        X = Xrotx(M_PI_2);
        X.r = Vector3d(0., -2., 0.);

        body_c_id = model->addBody(2, X, joint_c, body_c, "body_c");

        body_d = Body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
        joint_d = Joint(SpatialVector(0., 1., 0., 0., 0., 0.));

        body_d_id = model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_d, body_d, "body_d");

        body_e = Body(1., Vector3d(0., 0., 0.5), Vector3d(1., 1., 1.));
        joint_e = Joint(SpatialVector(0., 0., 1., 0., 0., 0.));

        X = Xrotz(M_PI_2);
        X.r = Vector3d(2., 0., 0.);

        body_e_id = model->addBody(body_d_id, X, joint_e, body_e, "body_e");

        body_f = Body(1., Vector3d(0., 0., 0.5), Vector3d(1., 1., 1.));
        joint_f = Joint(SpatialVector(1., 0., 0., 0., 0., 0.));

        X = Xrotx(M_PI_2);
        X.r = Vector3d(0., 0., 1.);

        body_f_id = model->addBody(body_e_id, X, joint_f, body_f, "body_f");

        Q = VectorNd::Constant((size_t)model->dof_count, 0.);
        QDot = VectorNd::Constant((size_t)model->dof_count, 0.);
        QDDot = VectorNd::Constant((size_t)model->dof_count, 0.);
        Tau = VectorNd::Constant((size_t)model->dof_count, 0.);
    }

    void TearDown()
    {
        delete model;
    }

    RobotDynamics::Model* model;

    unsigned int body_a_id, body_b_id, body_c_id, body_d_id, body_e_id, body_f_id;
    RobotDynamics::Body body_a, body_b, body_c, body_d, body_e, body_f;
    RobotDynamics::Joint joint_a, joint_b, joint_c, joint_d, joint_e, joint_f;

    RobotDynamics::Math::VectorNd Q;
    RobotDynamics::Math::VectorNd QDot;
    RobotDynamics::Math::VectorNd QDDot;
    RobotDynamics::Math::VectorNd Tau;
};

class FloatingBaseWith2SingleDofJoints : public testing::Test
{
  public:
    FloatingBaseWith2SingleDofJoints()
    {
        srand(time(nullptr));
    }

    /**
     *                                                           |
     *                                                           |
     *                                                           z3 Body2 - 1.5m, 1kg
     *                                        rootBody - 3m,1kg      |
     *                                   o--x2------- o --x1---y3-o
     *                                   |            |
     *                Body1 - 1m, 1kg   -z2           |
     *                                   |           -z1
     *                                                |
     *                z0                             \/ - this is not a link, just an axis
     *                /\
     *                |
     *                |
     *                |
     *  worldFrame    o-------> x0
     */

    void SetUp()
    {
        model = new Model;

        root_body = Body(1., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.));

        root_joint = Joint(JointTypeFloatingBase);
        SpatialTransform X = SpatialTransform();

        root_body_id = model->appendBody(X, root_joint, root_body, "floating_body");

        body_1 = Body(1., Vector3d(0., 0., -0.5), Vector3d(1., 1., 1.));
        joint_1 = Joint(SpatialVector(0., 1., 0., 0., 0., 0.));

        X.E = Matrix3dIdentity;
        X.r = Vector3d(-1.5, 0., 0.);

        body_1_id = model->addBody(root_body_id, X, joint_1, body_1, "body_1");

        body_2 = Body(1., Vector3d(0., 0., 0.75), Vector3d(1., 1., 1.));
        joint_2 = Joint(SpatialVector(1., 0., 0., 0., 0., 0.));

        X.E = Xrotz(M_PI_2).E;
        X.r = Vector3d(1.5, 0., 0.);

        body_2_id = model->addBody(root_body_id, X, joint_2, body_2, "body_2");

        Q = VectorNd::Constant((size_t)model->dof_count + 1, 0.);  // dof_count+1 because there is a spherical joint
        QDot = VectorNd::Constant((size_t)model->dof_count, 0.);
        QDDot = VectorNd::Constant((size_t)model->dof_count, 0.);
        Tau = VectorNd::Constant((size_t)model->dof_count, 0.);
    }

    void randomizeStates()
    {
        for (int i = 0; i < Q.size(); i++)
        {
            Q[i] = 0.4 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        }
        for (int i = 0; i < QDot.size(); i++)
        {
            QDot[i] = 0.1 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            Tau[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            QDDot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        }

        RobotDynamics::Math::Quaternion q;
        q = model->GetQuaternion(root_body_id, Q);
        q.normalize();
        model->SetQuaternion(root_body_id, q, Q);
    }

    void TearDown()
    {
        delete model;
    }

    RobotDynamics::Model* model;
    RobotDynamics::Body root_body, body_1, body_2;
    RobotDynamics::Joint root_joint, joint_1, joint_2;
    unsigned int root_body_id, body_1_id, body_2_id;

    RobotDynamics::Math::VectorNd Q;
    RobotDynamics::Math::VectorNd QDot;
    RobotDynamics::Math::VectorNd QDDot;
    RobotDynamics::Math::VectorNd Tau;
};

class FixedBase3DoF : public testing::Test
{
  public:
    FixedBase3DoF(){};

    void SetUp()
    {
        model = new Model;

        /* Basically a model like this, where X are the Center of Masses
         * and the CoM of the last (3rd) body comes out of the Y=X=0 plane.
         *
         *      Z
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

        body_b_id = model->addBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b, "body_b");

        body_c = Body(1., Vector3d(0., 0., 1.), Vector3d(1., 1., 1.));
        joint_c = Joint(SpatialVector(0., 0., 1., 0., 0., 0.));

        body_c_id = model->addBody(2, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c, "body_c");

        Q = VectorNd::Constant((size_t)model->dof_count, 0.);
        QDot = VectorNd::Constant((size_t)model->dof_count, 0.);
        QDDot = VectorNd::Constant((size_t)model->dof_count, 0.);
        Tau = VectorNd::Constant((size_t)model->dof_count, 0.);

        point_position = Vector3d::Zero(3);
        point_acceleration = Vector3d::Zero(3);

        ref_body_id = 0;
    }

    void TearDown()
    {
        delete model;
    }

    RobotDynamics::Model* model;

    unsigned int body_a_id, body_b_id, body_c_id, ref_body_id;
    RobotDynamics::Body body_a, body_b, body_c;
    RobotDynamics::Joint joint_a, joint_b, joint_c;

    RobotDynamics::Math::VectorNd Q;
    RobotDynamics::Math::VectorNd QDot;
    RobotDynamics::Math::VectorNd QDDot;
    RobotDynamics::Math::VectorNd Tau;

    RobotDynamics::Math::Vector3d point_position, point_acceleration;
    Math::FrameVector point_accel;
};

struct FixedBase6DoF : public testing::Test
{
    FixedBase6DoF()
    {
    }

    void SetUp()
    {
        using namespace RobotDynamics;
        using namespace RobotDynamics::Math;

        model = new Model;

        model->gravity = MotionVector(0., 0., 0., 0., -9.81, 0.);

        /* 3 DoF (rot.) joint at base
         * 3 DoF (rot.) joint child origin
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
        base_rot_z = Body(0., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.));
        joint_base_rot_z = Joint(SpatialVector(0., 0., 1., 0., 0., 0.));
        base_rot_z_id = model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_base_rot_z, base_rot_z);

        base_rot_y = Body(0., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.));
        joint_base_rot_y = Joint(SpatialVector(0., 1., 0., 0., 0., 0.));
        base_rot_y_id = model->appendBody(Xtrans(Vector3d(0., 0., 0.)), joint_base_rot_y, base_rot_y);

        base_rot_x = Body(1., Vector3d(0.5, 0., 0.), Vector3d(1., 1., 1.));
        joint_base_rot_x = Joint(SpatialVector(1., 0., 0., 0., 0., 0.));
        base_rot_x_id = model->addBody(base_rot_y_id, Xtrans(Vector3d(0., 0., 0.)), joint_base_rot_x, base_rot_x);

        // child body (3 DoF)
        child_rot_z = Body(0., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.));
        joint_child_rot_z = Joint(SpatialVector(0., 0., 1., 0., 0., 0.));
        child_rot_z_id = model->addBody(base_rot_x_id, Xtrans(Vector3d(1., 0., 0.)), joint_child_rot_z, child_rot_z);

        child_rot_y = Body(0., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.));
        joint_child_rot_y = Joint(SpatialVector(0., 1., 0., 0., 0., 0.));
        child_rot_y_id = model->addBody(child_rot_z_id, Xtrans(Vector3d(0., 0., 0.)), joint_child_rot_y, child_rot_y);

        child_rot_x = Body(1., Vector3d(0., 0.5, 0.), Vector3d(1., 1., 1.));
        joint_child_rot_x = Joint(SpatialVector(1., 0., 0., 0., 0., 0.));
        child_rot_x_id = model->addBody(child_rot_y_id, Xtrans(Vector3d(0., 0., 0.)), joint_child_rot_x, child_rot_x);

        Q = VectorNd::Constant(model->mBodies.size() - 1, 0.);
        QDot = VectorNd::Constant(model->mBodies.size() - 1, 0.);
        QDDot = VectorNd::Constant(model->mBodies.size() - 1, 0.);
        Tau = VectorNd::Constant(model->mBodies.size() - 1, 0.);

        contact_body_id = child_rot_x_id;
        contact_point = Vector3d(0.5, 0.5, 0.);
        contact_normal = Vector3d(0., 1., 0.);
    }

    void TearDown()
    {
        delete model;
    }

    RobotDynamics::Model* model;

    unsigned int base_rot_z_id, base_rot_y_id, base_rot_x_id, child_rot_z_id, child_rot_y_id, child_rot_x_id, base_body_id;

    RobotDynamics::Body base_rot_z, base_rot_y, base_rot_x, child_rot_z, child_rot_y, child_rot_x;

    RobotDynamics::Joint joint_base_rot_z, joint_base_rot_y, joint_base_rot_x, joint_child_rot_z, joint_child_rot_y, joint_child_rot_x;

    RobotDynamics::Math::VectorNd Q;
    RobotDynamics::Math::VectorNd QDot;
    RobotDynamics::Math::VectorNd QDDot;
    RobotDynamics::Math::VectorNd Tau;

    unsigned int contact_body_id;
    RobotDynamics::Math::Vector3d contact_point;
    RobotDynamics::Math::Vector3d contact_normal;
    RobotDynamics::ConstraintSet constraint_set;
};

class FloatingBase12DoF : public testing::Test
{
  public:
    FloatingBase12DoF()
    {
    }

    void SetUp()
    {
        using namespace RobotDynamics;
        using namespace RobotDynamics::Math;

        model = new Model;

        model->gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

        /* 3 DoF (rot.) joint at base
         * 3 DoF (rot.) joint child origin
         *
         *          X Contact point (ref child)
         *          |
         *    Base  |
         *   / body |
         *  O-------*
         *           \
         *             Child body
         */

        base_rot_x = Body(1., Vector3d(0.5, 0., 0.), Vector3d(1., 1., 1.));
        base_rot_x_id = model->addBody(0, SpatialTransform(),
                                       Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                                             SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                                       base_rot_x);

        // child body 1 (3 DoF)
        child_rot_z = Body(0., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.));
        joint_child_rot_z = Joint(SpatialVector(0., 0., 1., 0., 0., 0.));
        child_rot_z_id = model->addBody(base_rot_x_id, Xtrans(Vector3d(1., 0., 0.)), joint_child_rot_z, child_rot_z);

        child_rot_y = Body(0., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.));
        joint_child_rot_y = Joint(SpatialVector(0., 1., 0., 0., 0., 0.));
        child_rot_y_id = model->addBody(child_rot_z_id, Xtrans(Vector3d(0., 0., 0.)), joint_child_rot_y, child_rot_y);

        child_rot_x = Body(1., Vector3d(0., 0.5, 0.), Vector3d(1., 1., 1.));
        joint_child_rot_x = Joint(SpatialVector(1., 0., 0., 0., 0., 0.));
        child_rot_x_id = model->addBody(child_rot_y_id, Xtrans(Vector3d(0., 0., 0.)), joint_child_rot_x, child_rot_x);

        // child body (3 DoF)
        child_2_rot_z = Body(0., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.));
        joint_child_2_rot_z = Joint(SpatialVector(0., 0., 1., 0., 0., 0.));
        child_2_rot_z_id = model->addBody(child_rot_x_id, Xtrans(Vector3d(1., 0., 0.)), joint_child_2_rot_z, child_2_rot_z);

        child_2_rot_y = Body(0., Vector3d(0., 0., 0.), Vector3d(1., 1., 1.));
        joint_child_2_rot_y = Joint(SpatialVector(0., 1., 0., 0., 0., 0.));
        child_2_rot_y_id = model->addBody(child_2_rot_z_id, Xtrans(Vector3d(0., 0., 0.)), joint_child_2_rot_y, child_2_rot_y);

        child_2_rot_x = Body(1., Vector3d(0., 0.5, 0.), Vector3d(1., 1., 1.));
        joint_child_2_rot_x = Joint(SpatialVector(1., 0., 0., 0., 0., 0.));
        child_2_rot_x_id = model->addBody(child_2_rot_y_id, Xtrans(Vector3d(0., 0., 0.)), joint_child_2_rot_x, child_2_rot_x);

        Q = VectorNd::Constant(model->dof_count, 0.);
        QDot = VectorNd::Constant(model->dof_count, 0.);
        QDDot = VectorNd::Constant(model->dof_count, 0.);
        Tau = VectorNd::Constant(model->dof_count, 0.);
    }

    void TearDown()
    {
        delete model;
    }

    RobotDynamics::Model* model;

    unsigned int base_rot_z_id, base_rot_y_id, base_rot_x_id, child_rot_z_id, child_rot_y_id, child_rot_x_id, child_2_rot_z_id, child_2_rot_y_id, child_2_rot_x_id,
        base_body_id;

    RobotDynamics::Body base_rot_z, base_rot_y, base_rot_x, child_rot_z, child_rot_y, child_rot_x, child_2_rot_z, child_2_rot_y, child_2_rot_x;

    RobotDynamics::Joint joint_base_rot_z, joint_base_rot_y, joint_base_rot_x, joint_child_rot_z, joint_child_rot_y, joint_child_rot_x, joint_child_2_rot_z,
        joint_child_2_rot_y, joint_child_2_rot_x;

    RobotDynamics::Math::VectorNd Q;
    RobotDynamics::Math::VectorNd QDot;
    RobotDynamics::Math::VectorNd QDDot;
    RobotDynamics::Math::VectorNd Tau;
};

struct SimpleFixture : public testing::Test
{
    SimpleFixture()
    {
    }

    void SetUp()
    {
        model = new RobotDynamics::Model;
        model->gravity = RobotDynamics::Math::SpatialVector(0., 0., 0., 0., -9.81, 0.);
    }

    void TearDown()
    {
        delete model;
    }

    void ResizeVectors()
    {
        Q = RobotDynamics::Math::VectorNd::Zero(model->dof_count);
        QDot = RobotDynamics::Math::VectorNd::Zero(model->dof_count);
        QDDot = RobotDynamics::Math::VectorNd::Zero(model->dof_count);
        Tau = RobotDynamics::Math::VectorNd::Zero(model->dof_count);
    }

    RobotDynamics::Model* model;

    RobotDynamics::Math::VectorNd Q;
    RobotDynamics::Math::VectorNd QDot;
    RobotDynamics::Math::VectorNd QDDot;
    RobotDynamics::Math::VectorNd Tau;
};

struct FixedJoint2DoF : public testing::Test
{
    FixedJoint2DoF()
    {
    }

    void SetUp()
    {
        using namespace RobotDynamics;
        using namespace RobotDynamics::Math;

        model = new Model;

        /* Basically a model like this, where X are the Center of Masses
         * and the CoM of the last (3rd) body comes out of the Y=X=0 plane.
         *
         *      Z
         *      *---*
         *      |
         *      |
         *  Z   |
         *  O---*
         *      Y
         */

        body_a = Body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
        joint_a = Joint(SpatialVector(0., 0., 1., 0., 0., 0.));

        body_a_id = model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

        body_b = Body(1., Vector3d(0., 1., 0.), Vector3d(1., 1., 1.));
        joint_b = Joint(JointTypeFixed);

        body_b_id = model->addBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

        body_c = Body(1., Vector3d(0., 0., 1.), Vector3d(1., 1., 1.));
        joint_c = Joint(SpatialVector(0., 0., 1., 0., 0., 0.));

        body_c_id = model->addBody(2, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

        Q = VectorNd::Constant((size_t)model->dof_count, 0.);
        QDot = VectorNd::Constant((size_t)model->dof_count, 0.);
        QDDot = VectorNd::Constant((size_t)model->dof_count, 0.);

        point_position = Vector3d::Zero(3);
        point_acceleration = Vector3d::Zero(3);

        ref_body_id = 0;
    }

    void TearDown()
    {
        delete model;
    }

    RobotDynamics::Model* model;

    unsigned int body_a_id, body_b_id, body_c_id, ref_body_id;
    RobotDynamics::Body body_a, body_b, body_c;
    RobotDynamics::Joint joint_a, joint_b, joint_c;

    RobotDynamics::Math::VectorNd Q;
    RobotDynamics::Math::VectorNd QDot;
    RobotDynamics::Math::VectorNd QDDot;

    RobotDynamics::Math::Vector3d point_position, point_acceleration;
};

/** \brief Fixture that contains two models of which one has one joint fixed.
 */
struct FixedAndMovableJoint : public testing::Test
{
    FixedAndMovableJoint()
    {
    }

    void SetUp()
    {
        using namespace RobotDynamics;
        using namespace RobotDynamics::Math;

        model_movable = new Model;

        /* Basically a model like this, where X are the Center of Masses
         * and the CoM of the last (3rd) body comes out of the Y=X=0 plane.
         *
         *      Z
         *      *---*
         *      |
         *      |
         *  Z   |
         *  O---*
         *      Y
         */

        body_a = Body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
        joint_a = Joint(SpatialVector(0., 0., 1., 0., 0., 0.));

        body_a_id = model_movable->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

        body_b = Body(1., Vector3d(0., 1., 0.), Vector3d(1., 1., 1.));
        joint_b = Joint(SpatialVector(0., 1., 0., 0., 0., 0.));

        body_b_id = model_movable->addBody(body_a_id, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

        body_c = Body(1., Vector3d(0., 0., 1.), Vector3d(1., 1., 1.));
        joint_c = Joint(SpatialVector(0., 0., 1., 0., 0., 0.));

        body_c_id = model_movable->addBody(body_b_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

        Q = VectorNd::Constant((size_t)model_movable->dof_count, 0.);
        QDot = VectorNd::Constant((size_t)model_movable->dof_count, 0.);
        QDDot = VectorNd::Constant((size_t)model_movable->dof_count, 0.);
        Tau = VectorNd::Constant((size_t)model_movable->dof_count, 0.);
        C_movable = VectorNd::Zero((size_t)model_movable->dof_count);
        H_movable = MatrixNd::Zero((size_t)model_movable->dof_count, (size_t)model_movable->dof_count);

        // Assemble the fixed joint model
        model_fixed = new Model;

        body_a_fixed_id = model_fixed->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a, "body_a");
        Joint joint_fixed(JointTypeFixed);
        body_b_fixed_id = model_fixed->addBody(body_a_fixed_id, Xtrans(Vector3d(1., 0., 0.)), joint_fixed, body_b, "body_b");
        body_c_fixed_id = model_fixed->addBody(body_b_fixed_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c, "body_c");

        Q_fixed = VectorNd::Constant((size_t)model_fixed->dof_count, 0.);
        QDot_fixed = VectorNd::Constant((size_t)model_fixed->dof_count, 0.);
        QDDot_fixed = VectorNd::Constant((size_t)model_fixed->dof_count, 0.);
        Tau_fixed = VectorNd::Constant((size_t)model_fixed->dof_count, 0.);
        C_fixed = VectorNd::Zero((size_t)model_fixed->dof_count);
        H_fixed = MatrixNd::Zero((size_t)model_fixed->dof_count, (size_t)model_fixed->dof_count);

        point_position = Vector3d::Zero(3);
        point_acceleration = Vector3d::Zero(3);

        ref_body_id = 0;
    }

    void TearDown()
    {
        delete model_movable;
        delete model_fixed;
    }

    RobotDynamics::Math::VectorNd CreateDofVectorFromReducedVector(const RobotDynamics::Math::VectorNd& q_fixed)
    {
        assert(q_fixed.size() == model_fixed->dof_count);

        RobotDynamics::Math::VectorNd q_movable(model_movable->dof_count);

        q_movable[0] = q_fixed[0];
        q_movable[1] = 0.;
        q_movable[2] = q_fixed[1];

        return q_movable;
    }

    RobotDynamics::Math::MatrixNd CreateReducedInertiaMatrix(const RobotDynamics::Math::MatrixNd& H_movable)
    {
        assert(H_movable.rows() == model_movable->dof_count);
        assert(H_movable.cols() == model_movable->dof_count);
        RobotDynamics::Math::MatrixNd H(model_fixed->dof_count, model_fixed->dof_count);

        H(0, 0) = H_movable(0, 0);
        H(0, 1) = H_movable(0, 2);
        H(1, 0) = H_movable(2, 0);
        H(1, 1) = H_movable(2, 2);

        return H;
    }

    RobotDynamics::Model* model_fixed;
    RobotDynamics::Model* model_movable;

    unsigned int body_a_id, body_b_id, body_c_id, ref_body_id;
    unsigned int body_a_fixed_id, body_b_fixed_id, body_c_fixed_id;

    RobotDynamics::Body body_a, body_b, body_c;
    RobotDynamics::Joint joint_a, joint_b, joint_c;

    RobotDynamics::Math::VectorNd Q;
    RobotDynamics::Math::VectorNd QDot;
    RobotDynamics::Math::VectorNd QDDot;
    RobotDynamics::Math::VectorNd Tau;
    RobotDynamics::Math::VectorNd C_movable;
    RobotDynamics::Math::MatrixNd H_movable;

    RobotDynamics::Math::VectorNd Q_fixed;
    RobotDynamics::Math::VectorNd QDot_fixed;
    RobotDynamics::Math::VectorNd QDDot_fixed;
    RobotDynamics::Math::VectorNd Tau_fixed;
    RobotDynamics::Math::VectorNd C_fixed;
    RobotDynamics::Math::MatrixNd H_fixed;

    RobotDynamics::Math::Vector3d point_position, point_acceleration;
};

/** Model with two moving bodies and one fixed body
 */
struct RotZRotZYXFixed : public testing::Test
{
    RotZRotZYXFixed()
    {
    }

    void SetUp()
    {
        using namespace RobotDynamics;
        using namespace RobotDynamics::Math;

        model = new Model;

        Joint joint_rot_z(SpatialVector(0., 0., 1., 0., 0., 0.));

        Joint joint_rot_zyx(SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.));

        Body body_a(1., RobotDynamics::Math::Vector3d(1., 0.4, 0.4), RobotDynamics::Math::Vector3d(1., 1., 1.));
        Body body_b(2., RobotDynamics::Math::Vector3d(1., 0.4, 0.4), RobotDynamics::Math::Vector3d(1., 1., 1.));
        Body body_fixed(10., RobotDynamics::Math::Vector3d(1., 0.4, 0.4), RobotDynamics::Math::Vector3d(1., 1., 1.));

        fixture_transform_a = Xtrans(RobotDynamics::Math::Vector3d(1., 2., 3.));
        fixture_transform_b = Xtrans(RobotDynamics::Math::Vector3d(4., 5., 6.));
        fixture_transform_fixed = Xtrans(RobotDynamics::Math::Vector3d(-1., -2., -3.));

        body_a_id = model->addBody(0, fixture_transform_a, joint_rot_z, body_a);
        body_b_id = model->appendBody(fixture_transform_b, joint_rot_zyx, body_b);
        body_fixed_id = model->appendBody(fixture_transform_fixed, Joint(JointTypeFixed), body_fixed);

        Q = VectorNd(model->q_size);
        QDot = VectorNd(model->qdot_size);
        QDDot = VectorNd(model->qdot_size);
        Tau = VectorNd(model->qdot_size);

        Q.setZero();
        QDot.setZero();
        QDDot.setZero();
        Tau.setZero();
    }

    void TearDown()
    {
        delete model;
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

    RobotDynamics::Model* model;

    unsigned int body_a_id, body_b_id, body_fixed_id;

    RobotDynamics::Math::VectorNd Q;
    RobotDynamics::Math::VectorNd QDot;
    RobotDynamics::Math::VectorNd QDDot;
    RobotDynamics::Math::VectorNd Tau;

    RobotDynamics::Math::SpatialTransform fixture_transform_a;
    RobotDynamics::Math::SpatialTransform fixture_transform_b;
    RobotDynamics::Math::SpatialTransform fixture_transform_fixed;
};

struct TwoArms12DoF : public testing::Test
{
    TwoArms12DoF()
    {
        srand(time(nullptr));
    }

    void SetUp()
    {
        using namespace RobotDynamics;
        using namespace RobotDynamics::Math;

        model = new Model;

        /* Basically a model like this, where X are the Center of Masses
         * and the CoM of the last (3rd) body comes out of the Y=X=0 plane.
         *
         * *----O----*
         * |         |
         * |         |
         * *         *
         * |         |
         * |         |
         *
         */

        Body body_upper = Body(1., Vector3d(0., -0.2, 0.), Vector3d(1.1, 1.3, 1.5));
        Body body_lower = Body(0.5, Vector3d(0., -0.15, 0.), Vector3d(0.3, 0.5, 0.2));

        Joint joint_zyx = Joint(SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.));

        right_upper_arm = model->appendBody(Xtrans(Vector3d(0., 0., -0.3)), joint_zyx, body_upper, "RightUpper");
        //		model->appendBody (Xtrans (Vector3d (0., -0.4, 0.)), joint_zyx, body_lower, "RightLower");
        left_upper_arm = model->addBody(0, Xtrans(Vector3d(0., 0., 0.3)), joint_zyx, body_upper, "LeftUpper");
        //		model->appendBody (Xtrans (Vector3d (0., -0.4, 0.)), joint_zyx, body_lower, "LeftLower");

        q = VectorNd::Constant((size_t)model->dof_count, 0.);
        qdot = VectorNd::Constant((size_t)model->dof_count, 0.);
        qddot = VectorNd::Constant((size_t)model->dof_count, 0.);
        tau = VectorNd::Constant((size_t)model->dof_count, 0.);
    }

    void randomizeStates()
    {
        for (int i = 0; i < q.size(); i++)
        {
            q[i] = 0.4 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            qdot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            tau[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            qddot[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        }
    }

    void TearDown()
    {
        delete model;
    }

    RobotDynamics::Model* model;

    RobotDynamics::Math::VectorNd q;
    RobotDynamics::Math::VectorNd qdot;
    RobotDynamics::Math::VectorNd qddot;
    RobotDynamics::Math::VectorNd tau;

    unsigned int right_upper_arm, left_upper_arm;
};

struct FixedBase6DoF12DoFFloatingBase : public testing::Test
{
    FixedBase6DoF12DoFFloatingBase()
    {
    }

    void SetUp()
    {
        using namespace RobotDynamics;
        using namespace RobotDynamics::Math;

        model = new Model;

        model->gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);

        /* 3 DoF (rot.) joint at base
         * 3 DoF (rot.) joint child origin
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
        base_id = model->addBody(0, SpatialTransform(),
                                 Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                                       SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.)),
                                 base);

        // child body 1 (3 DoF)
        child = Body(1., Vector3d(0., 0.5, 0.), Vector3d(1., 1., 1.));
        joint_rotzyx = Joint(SpatialVector(0., 0., 1., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(1., 0., 0., 0., 0., 0.));
        child_id = model->addBody(base_id, Xtrans(Vector3d(1., 0., 0.)), joint_rotzyx, child);

        // child body (3 DoF)
        child_2 = Body(1., Vector3d(0., 0.5, 0.), Vector3d(1., 1., 1.));
        child_2_id = model->addBody(child_id, Xtrans(Vector3d(1., 0., 0.)), joint_rotzyx, child_2);

        Q = VectorNd::Constant(model->dof_count, 0.);
        QDot = VectorNd::Constant(model->dof_count, 0.);
        QDDot = VectorNd::Constant(model->dof_count, 0.);
        Tau = VectorNd::Constant(model->dof_count, 0.);

        contact_body_id = child_id;
        contact_point = Vector3d(0.5, 0.5, 0.);
        contact_normal = Vector3d(0., 1., 0.);
    }

    void TearDown()
    {
        delete model;
    }

    RobotDynamics::Model* model;

    unsigned int base_id, child_id, child_2_id;

    RobotDynamics::Body base, child, child_2;

    RobotDynamics::Joint joint_rotzyx;

    RobotDynamics::Math::VectorNd Q;
    RobotDynamics::Math::VectorNd QDot;
    RobotDynamics::Math::VectorNd QDDot;
    RobotDynamics::Math::VectorNd Tau;

    unsigned int contact_body_id;
    RobotDynamics::Math::Vector3d contact_point;
    RobotDynamics::Math::Vector3d contact_normal;
    RobotDynamics::ConstraintSet constraint_set;
};
