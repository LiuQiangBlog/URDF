// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <iomanip>

#include <rdl_dynamics/rdl_utils.hpp>
#include <rdl_dynamics/FramePoint.hpp>
#include <rdl_dynamics/Kinematics.hpp>
#include <rdl_dynamics/SpatialForce.hpp>
#include <rdl_dynamics/SpatialMomentum.hpp>
#include <rdl_dynamics/SpatialMotion.hpp>

namespace RobotDynamics
{
namespace Utils
{
using namespace std;
using namespace Math;

string getDofName(const SpatialVector& joint_dof)
{
    if (joint_dof == SpatialVector(1., 0., 0., 0., 0., 0.))
    {
        return "RX";
    }
    else if (joint_dof == SpatialVector(0., 1., 0., 0., 0., 0.))
    {
        return "RY";
    }
    else if (joint_dof == SpatialVector(0., 0., 1., 0., 0., 0.))
    {
        return "RZ";
    }
    else if (joint_dof == SpatialVector(0., 0., 0., 1., 0., 0.))
    {
        return "TX";
    }
    else if (joint_dof == SpatialVector(0., 0., 0., 0., 1., 0.))
    {
        return "TY";
    }
    else if (joint_dof == SpatialVector(0., 0., 0., 0., 0., 1.))
    {
        return "TZ";
    }

    ostringstream dof_stream(ostringstream::out);
    dof_stream << "custom (" << joint_dof.transpose() << ")";
    return dof_stream.str();
}

string getBodyName(const RobotDynamics::Model& model, unsigned int body_id)
{
    if (model.mBodies[body_id].mIsVirtual)
    {
        // if there is not a unique child we do not know what to do...
        if (model.mu[body_id].size() != 1)
        {
            return "";
        }

        return getBodyName(model, model.mu[body_id][0]);
    }

    return model.GetBodyName(body_id);
}

std::string getModelDOFOverview(const Model& model)
{
    stringstream result("");

    unsigned int q_index = 0;
    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        if (model.mJoints[i].mDoFCount == 1)
        {
            result << setfill(' ') << setw(3) << q_index << ": " << getBodyName(model, i) << "_" << getDofName(model.S[i]) << endl;
            q_index++;
        }
        else
        {
            for (unsigned int j = 0; j < model.mJoints[i].mDoFCount; j++)
            {
                result << setfill(' ') << setw(3) << q_index << ": " << getBodyName(model, i) << "_" << getDofName(model.mJoints[i].mJointAxes[j]) << endl;
                q_index++;
            }
        }
    }

    return result.str();
}

std::string printHierarchy(const RobotDynamics::Model& model, unsigned int body_index = 0, int indent = 0)
{
    stringstream result("");

    for (int j = 0; j < indent; j++)
    {
        result << "  ";
    }

    result << getBodyName(model, body_index);

    if (body_index > 0)
    {
        result << " [ ";
    }

    while (model.mBodies[body_index].mIsVirtual)
    {
        if (model.mu[body_index].size() == 0)
        {
            result << " end";
            break;
        }
        else if (model.mu[body_index].size() > 1)
        {
            cerr << endl
                 << "Error: Cannot determine multi-dof joint as massless body with id " << body_index << " (name: " << model.GetBodyName(body_index)
                 << ") has more than one child:" << endl;
            for (unsigned int ci = 0; ci < model.mu[body_index].size(); ci++)
            {
                cerr << "  id: " << model.mu[body_index][ci] << " name: " << model.GetBodyName(model.mu[body_index][ci]) << endl;
            }
            abort();
        }

        result << getDofName(model.S[body_index]) << ", ";

        body_index = model.mu[body_index][0];
    }

    if (body_index > 0)
    {
        result << getDofName(model.S[body_index]) << " ]";
    }
    result << endl;

    unsigned int child_index = 0;
    for (child_index = 0; child_index < model.mu[body_index].size(); child_index++)
    {
        result << printHierarchy(model, model.mu[body_index][child_index], indent + 1);
    }

    // print fixed children
    for (unsigned int fbody_index = 0; fbody_index < model.mFixedBodies.size(); fbody_index++)
    {
        if (model.mFixedBodies[fbody_index].mMovableParent == body_index)
        {
            for (int j = 0; j < indent + 1; j++)
            {
                result << "  ";
            }

            result << model.GetBodyName(model.fixed_body_discriminator + fbody_index) << " [fixed]" << endl;
        }
    }

    return result.str();
}

std::string getModelHierarchy(const Model& model)
{
    stringstream result("");

    result << printHierarchy(model);

    return result.str();
}

std::string getNamedBodyOriginsOverview(Model& model)
{
    stringstream result("");

    VectorNd Q(VectorNd::Zero(model.dof_count));
    updateKinematicsCustom(model, &Q, NULL, NULL);

    for (unsigned int body_id = 0; body_id < model.mBodies.size(); body_id++)
    {
        std::string body_name = model.GetBodyName(body_id);

        if (body_name.size() == 0)
        {
            continue;
        }

        Vector3d position = model.bodyFrames[body_id]->getInverseTransformToRoot().r;

        result << body_name << ": " << position.transpose() << endl;
    }

    return result.str();
}

void calcCenterOfMass(Model& model, const Math::VectorNd& q, Math::Vector3d& com, bool update_kinematics)
{
    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, nullptr, nullptr);
    }

    com.setZero();
    for (unsigned int i = 1; i < model.mBodies.size(); ++i)
    {
        com += model.mBodies[i].mMass * model.bodyCenteredFrames[i]->getInverseTransformToRoot().r;
    }

    com /= model.mass;
}

void calcCenterOfMass(Model& model, const Math::VectorNd& q, FramePoint& com, bool update_kinematics)
{
    Vector3d com_v;
    calcCenterOfMass(model, q, com_v, update_kinematics);
    com.setIncludingFrame(com_v, model.worldFrame);
}

void calcCenterOfMass(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::FramePoint& com, Math::FrameVector& com_velocity,
                      Math::FrameVector* angular_momentum, bool update_kinematics)
{
    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, &qdot, nullptr);
    }

    for (size_t i = 1; i < model.mBodies.size(); ++i)
    {
        model.Ic[i] = model.I[i];
        model.hc[i] = model.Ic[i] * model.v[i];
    }

    Momentum htot;
    com = FramePoint(model.worldFrame, 0., 0., 0.);
    Vector3d c(0., 0., 0.);
    for (size_t i = model.mBodies.size() - 1; i > 0; i--)
    {
        unsigned int lambda = model.lambda[i];
        c += model.mBodies[i].mMass * model.bodyCenteredFrames[i]->getInverseTransformToRoot().r;
        if (lambda != 0)
        {
            model.hc[lambda] = model.hc[lambda] + model.hc[i].transform_copy(model.bodyFrames[i]->getTransformToParent());
        }
        else
        {
            htot = htot + model.hc[i].transform_copy(model.bodyFrames[i]->getTransformToParent());
        }
    }

    c /= model.mass;
    com = FramePoint(model.worldFrame, c);

    com_velocity = FrameVector(model.worldFrame, htot.tail(3) / model.mass);

    if (angular_momentum)
    {
        htot.transform(SpatialTransform(c));
        *angular_momentum = FrameVector(model.worldFrame, htot.head(3));
    }
}

void calcCenterOfMass(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::Vector3d& com, Math::Vector3d* com_velocity, Vector3d* angular_momentum,
                      bool update_kinematics)
{
    bool do_velocities = com_velocity || angular_momentum;
    if (update_kinematics)
    {
        if (do_velocities)
        {
            updateKinematicsCustom(model, &q, &qdot, nullptr);
        }
        else
        {
            updateKinematicsCustom(model, &q, nullptr, nullptr);
        }
    }

    if (do_velocities)
    {
        for (size_t i = 1; i < model.mBodies.size(); ++i)
        {
            model.Ic[i] = model.I[i];
            model.hc[i] = model.Ic[i] * model.v[i];
        }
    }

    Momentum htot;
    com.setZero();
    for (size_t i = model.mBodies.size() - 1; i > 0; i--)
    {
        unsigned int lambda = model.lambda[i];
        com += model.mBodies[i].mMass * model.bodyCenteredFrames[i]->getInverseTransformToRoot().r;
        if (do_velocities)
        {
            if (lambda != 0)
            {
                model.hc[lambda] = model.hc[lambda] + model.hc[i].transform_copy(model.bodyFrames[i]->getTransformToParent());
            }
            else
            {
                htot = htot + model.hc[i].transform_copy(model.bodyFrames[i]->getTransformToParent());
            }
        }
    }

    com /= model.mass;

    if (com_velocity)
    {
        *com_velocity = Vector3d(htot[3] / model.mass, htot[4] / model.mass, htot[5] / model.mass);
    }

    if (angular_momentum)
    {
        htot.transform(SpatialTransform(com));
        angular_momentum->set(htot[0], htot[1], htot[2]);
    }
}

void calcCenterOfMass(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::FramePoint& com, Math::FrameVector* com_velocity, bool update_kinematics)
{
    if (update_kinematics)
    {
        if (com_velocity)
        {
            updateKinematicsCustom(model, &q, &qdot, nullptr);
        }
        else
        {
            updateKinematicsCustom(model, &q, nullptr, nullptr);
        }
    }

    Vector3d p_com(0., 0., 0.), v_com(0., 0., 0.);
    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        double bodyMass = model.mBodies[i].mMass;
        p_com += bodyMass * model.bodyCenteredFrames[i]->getInverseTransformToRoot().r;

        if (com_velocity)
        {
            v_com += bodyMass *
                     (model.bodyFrames[i]->getTransformToRoot().E * (model.v[i].getLinearPart() - model.mBodies[i].mCenterOfMass.cross(model.v[i].getAngularPart())));
        }
    }

    com.setIncludingFrame(p_com / model.mass, model.worldFrame);

    if (com_velocity)
    {
        com_velocity->setIncludingFrame(v_com / model.mass, model.worldFrame);
    }
}

void calcCenterOfMass(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::Vector3d& com, Math::Vector3d& com_velocity, bool update_kinematics)
{
    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, &qdot, nullptr);
    }

    com.setZero();
    com_velocity.setZero();
    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        double bodyMass = model.mBodies[i].mMass;
        com += bodyMass * model.bodyCenteredFrames[i]->getInverseTransformToRoot().r;
        com_velocity +=
            bodyMass * (model.bodyFrames[i]->getTransformToRoot().E * (model.v[i].getLinearPart() - model.mBodies[i].mCenterOfMass.cross(model.v[i].getAngularPart())));
    }

    com /= model.mass;
    com_velocity /= model.mass;
}

RobotDynamics::Math::FramePoint updateCenterOfMassFrame(Model& model, const Math::VectorNd& q, bool update_kinematics)
{
    RobotDynamics::Math::FramePoint com;
    calcCenterOfMass(model, q, com, update_kinematics);
    updateCenterOfMassFrame(model, com.vec());
    return com;
}

RobotDynamics::Math::FramePoint updateCenterOfMassFrame(Model& model, const Math::VectorNd& q, const Math::Vector3d& euler_ypr, bool update_kinematics)
{
    RobotDynamics::Math::FramePoint com;
    calcCenterOfMass(model, q, com, update_kinematics);
    updateCenterOfMassFrame(model, com.vec(), euler_ypr);
    return com;
}

RobotDynamics::Math::FramePoint updateCenterOfMassFrame(Model& model, const Math::VectorNd& q, const Math::Quaternion& orientation, bool update_kinematics)
{
    RobotDynamics::Math::FramePoint com;
    calcCenterOfMass(model, q, com, update_kinematics);
    updateCenterOfMassFrame(model, com.vec(), orientation);
    return com;
}

void updateCenterOfMassFrame(Model& model, const Math::Vector3d& p_com)
{
    model.comFrame->setTransformFromParent(SpatialTransform(p_com));
}

void updateCenterOfMassFrame(Model& model, const Math::Vector3d& p_com, const Math::Vector3d& euler_ypr)
{
    model.comFrame->setTransformFromParent(Math::XeulerZYX(euler_ypr) * SpatialTransform(p_com));
}

void updateCenterOfMassFrame(Model& model, const Math::Vector3d& p_com, const Math::Quaternion& orientation)
{
    model.comFrame->setTransformFromParent(Math::XrotQuat(orientation) * SpatialTransform(p_com));
}

void calcCenterOfMassVelocity(Model& model, Math::Vector3d& com_vel)
{
    for (size_t i = 1; i < model.mBodies.size(); ++i)
    {
        model.hc[i] = model.I[i] * model.v[i];
    }

    RigidBodyInertia Itot;
    Momentum htot;
    for (size_t i = model.mBodies.size() - 1; i > 0; i--)
    {
        unsigned int lambda = model.lambda[i];

        if (lambda != 0)
        {
            model.hc[lambda] = model.hc[lambda] + model.hc[i].transform_copy(model.bodyFrames[i]->getTransformToParent());
        }
        else
        {
            htot = htot + model.hc[i].transform_copy(model.bodyFrames[i]->getTransformToParent());
        }
    }

    com_vel = htot.tail(3) / model.mass;
}

void calcCenterOfMassVelocity(Model& model, Math::FrameVector& com_vel)
{
    RobotDynamics::Math::Vector3d v;
    calcCenterOfMassVelocity(model, v);
    com_vel.setIncludingFrame(v, model.worldFrame);
}

void calcCenterOfMassVelocity(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::Vector3d& com_vel, bool update_kinematics)
{
    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, &qdot, NULL);
    }

    calcCenterOfMassVelocity(model, com_vel);
}

void calcCenterOfMassVelocity(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::FrameVector& com_vel, bool update_kinematics)
{
    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, &qdot, NULL);
    }

    calcCenterOfMassVelocity(model, com_vel);
}

Math::SpatialForce calcGravityWrenchOnCenterOfMass(Model& model, const Math::VectorNd& q, bool update_kinematics)
{
    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, nullptr, nullptr);
    }

    Math::ForceVector f;
    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        if (!model.mBodies[i].mIsVirtual)
        {
            RobotDynamics::ReferenceFramePtr body_com_frame = model.bodyCenteredFrames[i];
            // Gravity wrench on body in world frame
            Math::ForceVector fg(model.mBodies[i].mMass * model.gravity.transform_copy(model.worldFrame->getTransformToDesiredFrame(body_com_frame)));
            // transform wrench to com frame
            fg.transform(body_com_frame->getTransformToDesiredFrame(model.comFrame));
            f += fg;
        }
    }

    return Math::SpatialForce(model.comFrame, f);
}

void calcCenterOfMassJacobian(Model& model, const Math::VectorNd& q, Math::MatrixNd& jCom, bool update_kinematics)
{
    assert(jCom.cols() == model.qdot_size && jCom.rows() == 3);

    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, nullptr, nullptr);
    }

    Math::SpatialMatrix K;
    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        K.block<3, 3>(3, 0) = -calcSubtreeCenterOfMassScaledByMass(model, i, q, false).toTildeForm();
        K.block<3, 3>(3, 3) = Math::Matrix3dIdentity * calcSubtreeMass(model, i);
        if (model.mJoints[i].mJointType != JointTypeCustom)
        {
            if (model.mJoints[i].mDoFCount == 1)
            {
                jCom.col(model.mJoints[i].q_index) = (K * model.S[i].transform_copy(model.bodyFrames[i]->getTransformToRoot())).block(3, 0, 3, 1);
            }
            else if (model.mJoints[i].mDoFCount == 3)
            {
                jCom.block(0, model.mJoints[i].q_index, 3, 3) = (K * (model.bodyFrames[i]->getTransformToRoot().toMatrix() * model.multdof3_S[i])).block(3, 0, 3, 3);
            }
        }
        else
        {
            jCom.block(0, model.mJoints[i].q_index, 3, model.mJoints[i].mDoFCount) =
                (K * (model.bodyFrames[i]->getTransformToRoot().toMatrix() * model.mCustomJoints[model.mJoints[i].custom_joint_index]->S))
                    .block(3, 0, 3, model.mJoints[i].mDoFCount);
        }
    }

    jCom /= calcSubtreeMass(model, 0);
}

Math::Vector3d calcSubtreeCenterOfMassScaledByMass(Model& model, const unsigned int bodyId, const VectorNd& q, bool updateKinematics)
{
    if (updateKinematics)
    {
        updateKinematicsCustom(model, &q, nullptr, nullptr);
    }

    std::vector<unsigned int> childBodyIds = model.mu[bodyId];

    Math::Vector3d comScaledByMass(model.bodyCenteredFrames[bodyId]->getInverseTransformToRoot().r * model.mBodies[bodyId].mMass);

    for (unsigned int j = 0; j < childBodyIds.size(); j++)
    {
        comScaledByMass += calcSubtreeCenterOfMassScaledByMass(model, childBodyIds[j], q, false);
    }

    return comScaledByMass;
}

double calcSubtreeMass(Model& model, const unsigned int bodyId)
{
    std::vector<unsigned int> childBodyIds = model.mu[bodyId];

    double subtreeMass = model.mBodies[bodyId].mMass;

    for (unsigned int j = 0; j < childBodyIds.size(); j++)
    {
        subtreeMass += calcSubtreeMass(model, childBodyIds[j]);
    }

    return subtreeMass;
}

double calcPotentialEnergy(Model& model, const Math::VectorNd& q, bool update_kinematics)
{
    Vector3d com;
    calcCenterOfMass(model, q, VectorNd::Zero(model.qdot_size), com, NULL, NULL, update_kinematics);

    Vector3d g = -Vector3d(model.gravity[3], model.gravity[4], model.gravity[5]);

    return model.mass * com.dot(g);
}

double calcKineticEnergy(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, bool update_kinematics)
{
    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, &qdot, NULL);
    }

    double result = 0.;

    for (size_t i = 1; i < model.mBodies.size(); i++)
    {
        result += (0.5 * model.v[i].dot(model.I[i] * model.v[i]));
    }
    return result;
}

void calcCentroidalMomentumMatrix(Model& model, const Math::VectorNd& q, Math::MatrixNd& A, Math::Vector3d com, bool update_kinematics)
{
    assert(A.cols() == model.qdot_size && A.rows() == 6);

    A.setZero();
    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, nullptr, nullptr);
    }

    for (size_t i = 1; i < model.mBodies.size(); ++i)
    {
        size_t j = i;
        ReferenceFramePtr bodyFrame = model.bodyFrames[i];
        while (j != 0)
        {
            if (model.mJoints[j].mJointType != JointTypeCustom)
            {
                if (model.mJoints[j].mDoFCount == 1)
                {
                    A.col(model.mJoints[j].q_index) += (model.I[i] * model.S[j].transform_copy(model.bodyFrames[j]->getTransformToDesiredFrame(bodyFrame)))
                                                           .transform_copy(bodyFrame->getTransformToRoot());
                }
                else if (model.mJoints[j].mDoFCount == 3)
                {
                    A.block<6, 3>(0, model.mJoints[j].q_index) +=
                        bodyFrame->getTransformToRoot().toMatrixAdjoint() *
                        (model.I[i].toMatrix() * (model.bodyFrames[j]->getTransformToDesiredFrame(bodyFrame).toMatrix() * model.multdof3_S[j]));
                }
            }
            else
            {
                unsigned int k = model.mJoints[j].custom_joint_index;

                A.block(0, model.mJoints[j].q_index, 6, model.mCustomJoints[k]->mDoFCount) +=
                    bodyFrame->getTransformToRoot().toMatrixAdjoint() *
                    (model.I[i].toMatrix() * model.bodyFrames[j]->getTransformToDesiredFrame(bodyFrame).toMatrix() * model.mCustomJoints[k]->S);
            }

            j = model.lambda[j];
        }
    }

    A = SpatialTransform(com).toMatrixAdjoint() * A;
}

void calcCentroidalMomentumMatrix(Model& model, const Math::VectorNd& q, Math::MatrixNd& A, bool update_kinematics)
{
    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, nullptr, nullptr);
    }

    Vector3d com;
    calcCenterOfMass(model, q, com, false);
    return calcCentroidalMomentumMatrix(model, q, A, com, false);
}

void calcCentroidalMomentumMatrixDot(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::MatrixNd& Adot, Math::Vector3d com,
                                     Math::Vector3d com_velocity, bool update_kinematics)
{
    assert(Adot.cols() == model.qdot_size && Adot.rows() == 6);
    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, &qdot, nullptr);
    }

    Adot.setZero();
    MotionVector com_twist(0., 0., 0., com_velocity.x(), com_velocity.y(), com_velocity.z());

    for (size_t i = 1; i < model.mBodies.size(); ++i)
    {
        size_t j = i;
        ReferenceFramePtr bodyFrame = model.bodyFrames[i];

        SpatialMatrix vcross = MotionVector(com_twist.transform_copy(bodyFrame->getInverseTransformToRoot()) - model.v[i]).crossm().transpose();

        while (j != 0)
        {
            SpatialTransform i_X_j = model.bodyFrames[j]->getTransformToDesiredFrame(bodyFrame);
            if (model.mJoints[j].mJointType != JointTypeCustom)
            {
                if (model.mJoints[j].mDoFCount == 1)
                {
                    Adot.col(model.mJoints[j].q_index) +=
                        bodyFrame->getTransformToRoot().toMatrixAdjoint() * (vcross * (model.I[i] * model.S[j].transform_copy(i_X_j)) +
                                                                             (model.I[i] * MotionVector(model.S_o[j] + model.v[j] % model.S[j]).transform_copy(i_X_j)));
                }
                else if (model.mJoints[j].mDoFCount == 3)
                {
                    Adot.block<6, 3>(0, model.mJoints[j].q_index) +=
                        bodyFrame->getTransformToRoot().toMatrixAdjoint() *
                        (vcross * (model.I[i].toMatrix() * i_X_j.toMatrix() * model.multdof3_S[j]) +
                         (model.I[i].toMatrix() * i_X_j.toMatrix() * (model.multdof3_S_o[j] + model.v[j].crossm() * model.multdof3_S[j])));
                }
            }
            else
            {
                unsigned int k = model.mJoints[j].custom_joint_index;

                Adot.block(0, model.mJoints[j].q_index, 6, model.mCustomJoints[k]->mDoFCount) +=
                    bodyFrame->getTransformToRoot().toMatrixAdjoint() *
                    (vcross * (model.I[i].toMatrix() * i_X_j.toMatrix() * model.mCustomJoints[k]->S) +
                     (model.I[i].toMatrix() * i_X_j.toMatrix() * (model.mCustomJoints[k]->S_o + model.v[j].crossm() * model.mCustomJoints[k]->S)));
            }

            j = model.lambda[j];
        }
    }

    // The above transforms everything to root, but we want it in com frame
    Adot = SpatialTransform(com).toMatrixAdjoint() * Adot;
}

void calcCentroidalMomentumMatrixDot(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::MatrixNd& Adot, bool update_kinematics)
{
    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, &qdot, nullptr);
    }

    Vector3d com, com_velocity;
    calcCenterOfMass(model, q, qdot, com, com_velocity, false);
    calcCentroidalMomentumMatrixDot(model, q, qdot, Adot, com, com_velocity, false);
}

void calcCentroidalMomentumMatrixAndMatrixDot(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::MatrixNd& A, Math::MatrixNd& Adot,
                                              bool update_kinematics)
{
    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, &qdot, nullptr);
    }

    Vector3d com, com_velocity;
    calcCenterOfMass(model, q, qdot, com, com_velocity, false);
    calcCentroidalMomentumMatrixAndMatrixDot(model, q, qdot, A, Adot, com, com_velocity, false);
}

void calcCentroidalMomentumMatrixAndMatrixDot(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::MatrixNd& A, Math::MatrixNd& Adot,
                                              Math::Vector3d com, Math::Vector3d com_velocity, bool update_kinematics)
{
    assert(Adot.cols() == model.qdot_size && Adot.rows() == 6 && A.cols() == model.qdot_size && A.rows() == 6);

    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, &qdot, nullptr);
    }

    A.setZero();
    Adot.setZero();
    MotionVector com_twist(0., 0., 0., com_velocity.x(), com_velocity.y(), com_velocity.z());

    // cppcheck-suppress variableScope
    for (size_t i = 1; i < model.mBodies.size(); ++i)
    {
        size_t j = i;
        ReferenceFramePtr bodyFrame = model.bodyFrames[i];

        SpatialMatrix vcross = MotionVector(com_twist.transform_copy(bodyFrame->getInverseTransformToRoot()) - model.v[i]).crossm().transpose();

        while (j != 0)
        {
            SpatialTransform i_X_j = model.bodyFrames[j]->getTransformToDesiredFrame(bodyFrame);
            if (model.mJoints[j].mJointType != JointTypeCustom)
            {
                if (model.mJoints[j].mDoFCount == 1)
                {
                    Momentum I_S = (model.I[i] * model.S[j].transform_copy(i_X_j));
                    A.col(model.mJoints[j].q_index) += I_S.transform_copy(bodyFrame->getTransformToRoot());
                    Adot.col(model.mJoints[j].q_index) += bodyFrame->getTransformToRoot().toMatrixAdjoint() *
                                                          (vcross * I_S + (model.I[i] * MotionVector(model.S_o[j] + model.v[j] % model.S[j]).transform_copy(i_X_j)));
                }
                else if (model.mJoints[j].mDoFCount == 3)
                {
                    Matrix63 I_S = model.I[i].toMatrix() * i_X_j.toMatrix() * model.multdof3_S[j];
                    A.block<6, 3>(0, model.mJoints[j].q_index) += bodyFrame->getTransformToRoot().toMatrixAdjoint() * (I_S);
                    Adot.block<6, 3>(0, model.mJoints[j].q_index) +=
                        bodyFrame->getTransformToRoot().toMatrixAdjoint() *
                        (vcross * I_S + (model.I[i].toMatrix() * i_X_j.toMatrix() * (model.multdof3_S_o[j] + model.v[j].crossm() * model.multdof3_S[j])));
                }
            }
            else
            {
                unsigned int k = model.mJoints[j].custom_joint_index;
                MatrixNd I_S = model.I[i].toMatrix() * i_X_j.toMatrix() * model.mCustomJoints[k]->S;
                A.block(0, model.mJoints[j].q_index, 6, model.mCustomJoints[k]->mDoFCount) += bodyFrame->getTransformToRoot().toMatrixAdjoint() * I_S;
                Adot.block(0, model.mJoints[j].q_index, 6, model.mCustomJoints[k]->mDoFCount) +=
                    bodyFrame->getTransformToRoot().toMatrixAdjoint() *
                    (vcross * I_S + (model.I[i].toMatrix() * i_X_j.toMatrix() * (model.mCustomJoints[k]->S_o + model.v[j].crossm() * model.mCustomJoints[k]->S)));
            }

            j = model.lambda[j];
        }
    }

    // The above transforms everything to root, but we want it in com frame
    SpatialTransform X_com(com);
    Adot = X_com.toMatrixAdjoint() * Adot;
    A = X_com.toMatrixAdjoint() * A;
}
}  // namespace Utils
}  // namespace RobotDynamics
