// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_UNIT_TEST_UTILS_HP__
#define __RDL_UNIT_TEST_UTILS_HP__

#include <math.h>
#include <rdl_dynamics/Model.hpp>
#include <rdl_dynamics/Dynamics.hpp>
#include <rdl_dynamics/ReferenceFrame.hpp>
#include <rdl_dynamics/FrameVectorPair.hpp>
#include <rdl_dynamics/Point3.hpp>
#include <rdl_dynamics/rdl_mathutils.hpp>
#include <rdl_dynamics/RdlExceptions.hpp>

using namespace RobotDynamics;
using namespace RobotDynamics::Math;

namespace unit_test_utils
{
const double E_MINUS_16 = 1.0e-16;
const double E_MINUS_15 = 1.0e-15;
const double E_MINUS_14 = 1.0e-14;
const double E_MINUS_13 = 1.0e-13;
const double E_MINUS_12 = 1.0e-12;
const double E_MINUS_11 = 1.0e-11;
const double E_MINUS_10 = 1.0e-10;
const double E_MINUS_9 = 1.0e-9;
const double E_MINUS_5 = 1.e-5;

// Call this from test fixtures to make sure none of the methods called
// are modifying the zero vectors/matrices
// cppcheck-suppress unused-function
bool checkModelZeroVectorsAndMatrices(RobotDynamics::Model& model)
{
    for (unsigned int i = 0; i < model.ndof0_vec.rows(); i++)
    {
        if (model.ndof0_vec[i] != 0.)
        {
            return false;
        }
    }

    for (unsigned int i = 0; i < model.q0_vec.rows(); i++)
    {
        if (model.q0_vec[i] != 0.)
        {
            return false;
        }
    }

    for (unsigned int i = 0; i < model.nbodies0_vec.rows(); i++)
    {
        if (model.nbodies0_vec[i] != 0.)
        {
            return false;
        }
    }

    for (unsigned int i = 0; i < model.ndof0_mat.rows(); i++)
    {
        for (unsigned int j = 0; j < model.ndof0_mat.cols(); j++)
        {
            if (model.ndof0_mat(i, j) != 0.)
            {
                return false;
            }
        }
    }

    for (unsigned int i = 0; i < model.three_x_qd0.rows(); i++)
    {
        for (unsigned int j = 0; j < model.three_x_qd0.cols(); j++)
        {
            if (model.three_x_qd0(i, j) != 0.)
            {
                return false;
            }
        }
    }

    for (unsigned int i = 0; i < model.mCustomJoints.size(); i++)
    {
        CustomJoint* custom_joint = model.mCustomJoints[i];

        for (unsigned int j = 0; j < custom_joint->mDoFCount; j++)
        {
            if (custom_joint->ndof0_vec[j] != 0.)
            {
                return false;
            }
        }
    }

    return true;
}

template <typename T>
static T getRandomNumber()
{
    double val = (2000.0 * rand() / RAND_MAX - 1000.0);
    return static_cast<T>(val);
}

template <typename T>
static T getRandomNumber(T min, T max)
{
    double val = (fabs((max - min)) * rand() / RAND_MAX + min);
    return static_cast<T>(val);
}

static inline Point3d getRandomPoint3()
{
    Point3d point;
    point.x() = getRandomNumber<double>();
    point.y() = getRandomNumber<double>();
    point.z() = getRandomNumber<double>();

    return point;
}

template <typename T>
static T getRandomAngle()
{
    double angle = 2 * (M_PI - 0.01) * rand() / RAND_MAX - (M_PI - 0.01);
    return static_cast<T>(angle);
}

static inline bool checkSpatialMatrixEpsilonClose(const RobotDynamics::Math::SpatialMatrix& t1, const RobotDynamics::Math::SpatialMatrix& t2, const double epsilon)
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            if (fabs(t1(i, j) - t2(i, j)) > epsilon)
            {
                return false;
            }
        }
    }

    return true;
}

static inline bool checkMatrix3dEpsilonClose(const RobotDynamics::Math::Matrix3d& t1, const RobotDynamics::Math::Matrix3d& t2, const double epsilon)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (fabs(t1(i, j) - t2(i, j)) > epsilon)
            {
                return false;
            }
        }
    }

    return true;
}

static inline bool checkMatrixNdEpsilonClose(const RobotDynamics::Math::MatrixNd& t1, const RobotDynamics::Math::MatrixNd& t2, const double epsilon)
{
    if (t1.rows() != t2.rows() || t1.cols() != t2.cols())
    {
        throw RobotDynamics::RdlException("Cannot compare MatrixNd's of different sizes!");
    }

    for (int i = 0; i < t1.rows(); i++)
    {
        for (int j = 0; j < t1.cols(); j++)
        {
            if (fabs(t1(i, j) - t2(i, j)) > epsilon)
            {
                return false;
            }
        }
    }

    return true;
}

static inline bool checkMatrixNdEq(const RobotDynamics::Math::MatrixNd& t1, const RobotDynamics::Math::MatrixNd& t2)
{
    if (t1.rows() != t2.rows() || t1.cols() != t2.cols())
    {
        throw RobotDynamics::RdlException("Cannot compare MatrixNd's of different sizes!");
    }

    for (int i = 0; i < t1.rows(); i++)
    {
        for (int j = 0; j < t1.cols(); j++)
        {
            if (t1(i, j) != t2(i, j))
            {
                return false;
            }
        }
    }

    return true;
}

static inline bool checkMatrix3dEq(const RobotDynamics::Math::Matrix3d& t1, const RobotDynamics::Math::Matrix3d& t2)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (t1(i, j) != t2(i, j))
            {
                return false;
            }
        }
    }

    return true;
}

static inline bool checkVector3dEq(const RobotDynamics::Math::Vector3d& v1, const RobotDynamics::Math::Vector3d& v2)
{
    for (int i = 0; i < 3; i++)
    {
        if (v1(i) != v2(i))
        {
            return false;
        }
    }

    return true;
}

static inline bool checkVector4dEq(const RobotDynamics::Math::Vector4d& v1, const RobotDynamics::Math::Vector4d& v2)
{
    for (int i = 0; i < 4; i++)
    {
        if (v1(i) != v2(i))
        {
            return false;
        }
    }

    return true;
}

static inline bool checkVectorNdEq(const RobotDynamics::Math::VectorNd& v1, const RobotDynamics::Math::VectorNd& v2)
{
    if (v1.rows() != v2.rows())
    {
        throw RobotDynamics::RdlException("Cannot compare vectors because they are not the same length!");
    }

    for (int i = 0; i < v1.rows(); i++)
    {
        if (v1(i) != v2(i))
        {
            return false;
        }
    }

    return true;
}

static inline bool checkVector3dEpsilonClose(const RobotDynamics::Math::Vector3d& v1, const RobotDynamics::Math::Vector3d& v2, const double epsilon)
{
    for (int i = 0; i < 3; i++)
    {
        if (fabs(v1(i) - v2(i)) > epsilon)
        {
            return false;
        }
    }

    return true;
}

static inline bool checkFrameVectorPairEpsilonClose(const RobotDynamics::Math::FrameVectorPair& v1, const RobotDynamics::Math::FrameVectorPair& v2, const double epsilon)
{
    return (v1.linear().getReferenceFrame() == v2.linear().getReferenceFrame() && checkVector3dEpsilonClose(v1.linear(), v2.linear(), epsilon) &&
            checkVector3dEpsilonClose(v1.angular(), v2.angular(), epsilon));
}

static inline bool checkVector4dEpsilonClose(const RobotDynamics::Math::Vector4d& v1, const RobotDynamics::Math::Vector4d& v2, const double epsilon)
{
    for (int i = 0; i < 4; i++)
    {
        if (fabs(v1(i) - v2(i)) > epsilon)
        {
            return false;
        }
    }

    return true;
}

static inline bool checkVectorNdEpsilonClose(const RobotDynamics::Math::VectorNd& v1, const RobotDynamics::Math::VectorNd& v2, const double epsilon)
{
    if (v1.rows() != v2.rows())
    {
        throw RobotDynamics::RdlException("Cannot compare vectors because they are not the same length!");
    }

    for (int i = 0; i < v1.rows(); i++)
    {
        if (fabs(v1(i) - v2(i)) > epsilon)
        {
            return false;
        }
    }

    return true;
}

static inline bool checkSpatialVectorsEpsilonClose(const RobotDynamics::Math::SpatialVector& v1, const RobotDynamics::Math::SpatialVector& v2, const double epsilon)
{
    for (int i = 0; i < 6; i++)
    {
        if (fabs(v1(i) - v2(i)) > epsilon)
        {
            return false;
        }
    }

    return true;
}

static inline bool checkSpatialVectorsEq(const RobotDynamics::Math::SpatialVector& v1, const RobotDynamics::Math::SpatialVector& v2)
{
    for (int i = 0; i < 6; i++)
    {
        if (v1(i) != v2(i))
        {
            return false;
        }
    }

    return true;
}

static inline void updateAllFrames(std::vector<ReferenceFramePtr> frames)
{
    for (unsigned int i = 0; i < frames.size(); ++i)
    {
        frames[i]->update();
    }
}

static inline ReferenceFramePtr getARandomFrame(std::vector<ReferenceFramePtr> frames)
{
    int index = rand() % frames.size();

    return frames[index];
}

static inline RobotDynamics::Math::SpatialTransform createRandomSpatialTransform()
{
    RobotDynamics::Math::SpatialTransform X_x = RobotDynamics::Math::Xrotx(getRandomAngle<double>());
    RobotDynamics::Math::SpatialTransform X_y = RobotDynamics::Math::Xroty(getRandomAngle<double>());
    RobotDynamics::Math::SpatialTransform X_z = RobotDynamics::Math::Xrotz(getRandomAngle<double>());

    RobotDynamics::Math::SpatialTransform X = X_x * X_y * X_z;

    return X;
}

static inline RobotDynamics::Math::SpatialTransform getTransformToRootByClimbingTree(ReferenceFramePtr frame)
{
    if (frame->getParentFrame() == nullptr)
    {
        return RobotDynamics::Math::SpatialTransform();
    }

    RobotDynamics::Math::SpatialTransform transform;
    ReferenceFramePtr parent = frame;
    while (parent != nullptr)
    {
        transform = parent->getTransformToParent() * transform;
        parent = parent->getParentFrame();
    }

    return transform;
}

static inline ReferenceFramePtr createRandomUnchangingFrame(const std::string& frameName, ReferenceFramePtr parentFrame, const unsigned int movableBodyId)
{
    return ReferenceFramePtr(new ReferenceFrame(frameName, parentFrame, createRandomSpatialTransform(), false, movableBodyId));
}

/**
 *
 * @brief  Using simple Euler integration, integrate the system dynamics one step
 * @param model
 * @param Q
 * @param QDot
 * @param x
 * @param tau
 * @param h Integration step size
 * @param f_ext
 *
 * @note This <b>only</b> works for models consisting solely of revolute/prismatic joints and spherical joints. It will not work for any
 * euler joints, etc.
 */
void integrateEuler(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, Math::VectorNd& x, const Math::VectorNd& tau, double h,
                    Math::SpatialForceV* f_ext = nullptr)
{
    Math::VectorNd qddot = Math::VectorNd::Zero(model.qdot_size);

    Math::VectorNd q = Math::VectorNd::Zero(model.q_size);
    Math::VectorNd qdot = Math::VectorNd::Zero(model.qdot_size);

    q = Q;
    qdot = QDot;

    forwardDynamics(model, q, qdot, tau, qddot, f_ext);

    Math::VectorNd x_0 = Math::VectorNd::Zero(model.q_size + model.qdot_size);
    Math::VectorNd dx = Math::VectorNd::Zero(2 * model.qdot_size);

    x_0.head(model.q_size) = Q;
    x_0.tail(model.qdot_size) = QDot;

    dx.head(model.qdot_size) = qdot;
    dx.tail(model.qdot_size) = qddot;

    for (unsigned int i = 0; i < model.mBodies.size(); i++)
    {
        if (model.mJoints[i].mJointType == JointTypeSpherical)
        {
            //              See algorithm in section 4.2.2 of "Quaternion kinematics for the error-state Kalman filter" by Joan Sola
            unsigned int q_index = model.mJoints[i].q_index;
            RobotDynamics::Math::Quaternion w_q(QDot[q_index], QDot[q_index + 1], QDot[q_index + 2], 0.);
            RobotDynamics::Math::Quaternion q = model.GetQuaternion(i, Q);
            RobotDynamics::Math::Quaternion q_int, new_q;

            x[model.q_size + q_index] = x_0[model.q_size + q_index] + h * dx[model.dof_count + q_index];
            x[model.q_size + q_index + 1] = x_0[model.q_size + q_index + 1] + h * dx[model.dof_count + q_index + 1];
            x[model.q_size + q_index + 2] = x_0[model.q_size + q_index + 2] + h * dx[model.dof_count + q_index + 2];

            Vector3d w_np1(x[model.q_size + q_index], x[model.q_size + q_index + 1], x[model.q_size + q_index + 2]);
            Vector3d w_n(QDot[q_index], QDot[q_index + 1], QDot[q_index + 2]);
            Vector3d w_bar = (w_np1 + w_n) / 2.0;
            Vector3d w_bar_axis = w_bar / w_bar.norm();
            Vector3d first_order_term = (h / 24.0) * (w_n.cross(w_np1));
            q_int = Quaternion(w_bar_axis, w_bar.norm() * h);
            q_int += Quaternion(first_order_term[0], first_order_term[1], first_order_term[2], 0.0);
            q_int.normalize();

            new_q = q * q_int;

            new_q.normalize();

            x[q_index] = new_q[0];
            x[q_index + 1] = new_q[1];
            x[q_index + 2] = new_q[2];
            x[model.multdof3_w_index[i]] = new_q[3];
        }
        else
        {
            unsigned int idx = model.mJoints[i].q_index;
            x[idx] = x_0[idx] + h * dx[idx];
            x[model.q_size + idx] = x_0[model.q_size + idx] + h * dx[model.dof_count + idx];
        }
    }
}

/**
 * @brief Using a Runge-Kutta 4th order algorithm, integrate the system dynamics one step
 * @param model
 * @param Q
 * @param QDot
 * @param x
 * @param tau
 * @param h Integration step size
 * @param f_ext
 *
 * @note This <b>only</b> works for models consisting solely of revolute/prismatic joints. It will not work for any
 * euler joints or spherical joints, etc.
 */
void integrateRk4(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, Math::VectorNd& x, const Math::VectorNd& tau, double h,
                  Math::SpatialForceV* f_ext = nullptr)
{
    Math::VectorNd q_n = Q;
    Math::VectorNd qdot_n = QDot;
    Math::VectorNd qddot = Math::VectorNd::Zero(model.qdot_size);
    Math::VectorNd k1 = Math::VectorNd::Zero(2 * model.qdot_size);
    Math::VectorNd k2 = Math::VectorNd::Zero(2 * model.qdot_size);
    Math::VectorNd k3 = Math::VectorNd::Zero(2 * model.qdot_size);
    Math::VectorNd k4 = Math::VectorNd::Zero(2 * model.qdot_size);

    k1.setZero();
    k2.setZero();
    k3.setZero();
    k4.setZero();

    Math::VectorNd q = Math::VectorNd::Zero(model.q_size);
    Math::VectorNd qdot = Math::VectorNd::Zero(model.qdot_size);

    q = Q;
    qdot = QDot;

    forwardDynamics(model, q, qdot, tau, qddot, f_ext);

    k1.head(model.qdot_size) = qdot;
    k1.tail(model.qdot_size) = qddot;

    q = q_n + (h / 2.) * k1.head(model.q_size);
    qdot = qdot_n + (h / 2.) * k1.tail(model.qdot_size);

    forwardDynamics(model, q, qdot, tau, qddot, f_ext);

    k2.head(model.qdot_size) = qdot;
    k2.tail(model.qdot_size) = qddot;

    q = q_n + (h / 2.) * k2.head(model.q_size);
    qdot = qdot_n + (h / 2.) * k2.tail(model.qdot_size);

    forwardDynamics(model, q, qdot, tau, qddot, f_ext);

    k3.head(model.qdot_size) = qdot;
    k3.tail(model.qdot_size) = qddot;

    q = q_n + h * k3.head(model.q_size);
    qdot = qdot_n + h * k3.tail(model.qdot_size);

    forwardDynamics(model, q, qdot, tau, qddot, f_ext);

    k4.head(model.qdot_size) = qdot;
    k4.tail(model.qdot_size) = qddot;

    Math::VectorNd x_0 = Math::VectorNd::Zero(model.q_size + model.qdot_size);

    x_0.head(model.q_size) = Q;
    x_0.tail(model.qdot_size) = QDot;

    x = x_0 + (h / 6.) * (k1 + 2. * k2 + 2. * k3 + k4);
}
}  // namespace unit_test_utils

#endif  //__UNIT_TEST_UTILS__
