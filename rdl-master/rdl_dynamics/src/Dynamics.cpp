// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <limits>
#include <string.h>

#include "rdl_dynamics/Dynamics.hpp"
#include "rdl_dynamics/Kinematics.hpp"
#include "rdl_dynamics/SpatialMomentum.hpp"
#include "rdl_dynamics/SpatialMotion.hpp"

namespace RobotDynamics
{
using namespace Math;

void inverseDynamics(Model& model, const VectorNd& Q, const VectorNd& QDot, const VectorNd& QDDot, VectorNd& Tau, SpatialForceV* f_ext, bool update_kinematics)
{
    // Reset the velocity of the root body
    model.v[0].setZero();
    model.a[0].set(-model.gravity);

    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        unsigned int q_index = model.mJoints[i].q_index;
        unsigned int lambda = model.lambda[i];
        ReferenceFramePtr bodyFrame = model.bodyFrames[i];

        if (update_kinematics)
        {
            jcalc(model, i, Q, QDot);

            model.v[i].set(model.v[lambda].transform_copy(bodyFrame->getTransformFromParent()) + model.v_J[i]);
            model.c[i] = model.c_J[i] + model.v[i] % model.v_J[i];
        }

        if (model.mJoints[i].mJointType != JointTypeCustom)
        {
            if (model.mJoints[i].mDoFCount == 1)
            {
                model.a[i].set(model.a[lambda].transform_copy(bodyFrame->getTransformFromParent()) + model.c[i] + model.S[i] * QDDot[q_index]);
            }
            else if (model.mJoints[i].mDoFCount == 3)
            {
                model.a[i].set(bodyFrame->getTransformFromParent().apply(model.a[lambda]) + model.c[i] +
                               model.multdof3_S[i] * Vector3d(QDDot[q_index], QDDot[q_index + 1], QDDot[q_index + 2]));
            }
        }
        else
        {
            unsigned int k = model.mJoints[i].custom_joint_index;
            model.a[i].set(bodyFrame->getTransformFromParent().apply(model.a[lambda]) + model.c[i] +
                           model.mCustomJoints[k]->S * QDDot.block(q_index, 0, model.mCustomJoints[k]->mDoFCount, 1));
        }

        if (!model.mBodies[i].mIsVirtual)
        {
            model.f_b[i] = f_ext == nullptr ? RobotDynamics::Math::SpatialForce(bodyFrame, model.I[i] * model.a[i] + model.v[i] % (model.I[i] * model.v[i])) :
                                              RobotDynamics::Math::SpatialForce(bodyFrame, model.I[i] * model.a[i] + model.v[i] % (model.I[i] * model.v[i]) -
                                                                                               (*f_ext)[i].changeFrameAndCopy(bodyFrame));
        }
        else
        {
            model.f_b[i].setIncludingFrame(bodyFrame, 0., 0., 0., 0., 0., 0.);
        }
    }

    for (unsigned int i = model.mBodies.size() - 1; i > 0; i--)
    {
        if (model.mJoints[i].mJointType != JointTypeCustom)
        {
            if (model.mJoints[i].mDoFCount == 1)
            {
                Tau[model.mJoints[i].q_index] = model.S[i].dot(model.f_b[i]);
            }
            else if (model.mJoints[i].mDoFCount == 3)
            {
                Tau.block<3, 1>(model.mJoints[i].q_index, 0) = model.multdof3_S[i].transpose() * model.f_b[i];
            }
        }
        else
        {
            unsigned int k = model.mJoints[i].custom_joint_index;
            Tau.block(model.mJoints[i].q_index, 0, model.mCustomJoints[k]->mDoFCount, 1) = model.mCustomJoints[k]->S.transpose() * model.f_b[i];
        }

        if (model.lambda[i] != 0)
        {
            model.f_b[model.lambda[i]] += model.f_b[i].changeFrameAndCopy(model.bodyFrames[model.lambda[i]]);
        }
    }
}

void gravityEffects(Model& model, Math::VectorNd& Tau)
{
    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        RobotDynamics::ReferenceFramePtr frame = model.bodyFrames[i];
        if (!model.mBodies[i].mIsVirtual)
        {
            model.f_b[i].setIncludingFrame(frame, -(model.I[i] * model.gravity.transform_copy(model.worldFrame->getTransformToDesiredFrame(frame))));
        }
        else
        {
            model.f_b[i].setIncludingFrame(frame, 0., 0., 0., 0., 0., 0.);
        }
    }

    for (unsigned int i = model.mBodies.size() - 1; i > 0; i--)
    {
        if (model.mJoints[i].mJointType != JointTypeCustom)
        {
            if (model.mJoints[i].mDoFCount == 1)
            {
                Tau[model.mJoints[i].q_index] = model.S[i].dot(model.f_b[i]);
            }
            else if (model.mJoints[i].mDoFCount == 3)
            {
                Tau.block<3, 1>(model.mJoints[i].q_index, 0) = model.multdof3_S[i].transpose() * model.f_b[i];
            }
        }
        else
        {
            unsigned int k = model.mJoints[i].custom_joint_index;
            Tau.block(model.mJoints[i].q_index, 0, model.mCustomJoints[k]->mDoFCount, 1) = model.mCustomJoints[k]->S.transpose() * model.f_b[i];
        }

        if (model.lambda[i] != 0)
        {
            model.f_b[model.lambda[i]] += model.f_b[i].changeFrameAndCopy(model.bodyFrames[model.lambda[i]]);
        }
    }
}

void coriolisEffects(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, Math::VectorNd& Tau, bool update_kinematics)
{
    model.v[0].setZero();
    model.a[0].setZero();

    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        ReferenceFramePtr bodyFrame = model.bodyFrames[i];
        if (model.lambda[i] == 0)
        {
            if (update_kinematics)
            {
                jcalc(model, i, Q, QDot);
                model.v[i].set(model.v_J[i]);
            }

            model.a[i].setZero();
        }
        else
        {
            if (update_kinematics)
            {
                jcalc(model, i, Q, QDot);
                model.v[i].set(model.v[model.lambda[i]].transform_copy(model.bodyFrames[i]->getTransformFromParent()) + model.v_J[i]);
                model.c[i] = model.c_J[i] + model.v[i] % model.v_J[i];
            }

            model.a[i].set(model.a[model.lambda[i]].transform_copy(bodyFrame->getTransformFromParent()) + model.c[i]);
        }

        if (!model.mBodies[i].mIsVirtual)
        {
            model.f_b[i].setIncludingFrame(bodyFrame, model.I[i] * model.a[i] + model.v[i] % (Math::Momentum(model.I[i], model.v[i])));
        }
        else
        {
            model.f_b[i].setIncludingFrame(bodyFrame, 0., 0., 0., 0., 0., 0.);
        }
    }

    for (unsigned int i = model.mBodies.size() - 1; i > 0; i--)
    {
        if (model.mJoints[i].mJointType != JointTypeCustom)
        {
            if (model.mJoints[i].mDoFCount == 1)
            {
                Tau[model.mJoints[i].q_index] = model.S[i].dot(model.f_b[i]);
            }
            else if (model.mJoints[i].mDoFCount == 3)
            {
                Tau.block<3, 1>(model.mJoints[i].q_index, 0) = model.multdof3_S[i].transpose() * model.f_b[i];
            }
        }
        else
        {
            unsigned int k = model.mJoints[i].custom_joint_index;
            Tau.block(model.mJoints[i].q_index, 0, model.mCustomJoints[k]->mDoFCount, 1) = model.mCustomJoints[k]->S.transpose() * model.f_b[i];
        }

        if (model.lambda[i] != 0)
        {
            model.f_b[model.lambda[i]] += model.f_b[i].changeFrameAndCopy(model.bodyFrames[model.lambda[i]]);
        }
    }
}

void calcBodyGravityWrench(Model& model, unsigned int body_id, RobotDynamics::Math::SpatialForce& gravity_wrench)
{
    assert(model.IsBodyId(body_id));
    assert(body_id > 0);

    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        RobotDynamics::ReferenceFramePtr frame = model.bodyFrames[i];
        if (!model.mBodies[i].mIsVirtual)
        {
            model.f_b[i].setIncludingFrame(frame, (model.I[i] * model.gravity.transform_copy(model.worldFrame->getTransformToDesiredFrame(frame))));
        }
        else
        {
            model.f_b[i].setIncludingFrame(frame, 0., 0., 0., 0., 0., 0.);
        }
    }

    unsigned int i = model.mBodies.size() - 1;
    while (i != body_id)
    {
        // This is very inefficient, need to only go down the chain of the branch the body is on
        model.f_b[model.lambda[i]] += model.f_b[i].changeFrameAndCopy(model.bodyFrames[model.lambda[i]]);
        i--;
    }

    gravity_wrench = model.f_b[body_id];
}

void nonlinearEffects(Model& model, const VectorNd& Q, const VectorNd& QDot, VectorNd& Tau, bool update_kinematics)
{
    model.v[0].setZero();
    model.a[0].set(-model.gravity);

    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        RobotDynamics::ReferenceFramePtr frame = model.bodyFrames[i];
        if (model.lambda[i] == 0)
        {
            if (update_kinematics)
            {
                jcalc(model, i, Q, QDot);
                model.v[i].set(model.v_J[i]);
            }

            model.a[i].set(-model.gravity.transform_copy(frame->getTransformFromParent()));
        }
        else
        {
            if (update_kinematics)
            {
                jcalc(model, i, Q, QDot);
                model.v[i].set(model.v[model.lambda[i]].transform_copy(frame->getTransformFromParent()) + model.v_J[i]);
                model.c[i] = model.c_J[i] + model.v[i] % model.v_J[i];
            }

            model.a[i].set(model.a[model.lambda[i]].transform_copy(frame->getTransformFromParent()) + model.c[i]);
        }

        if (!model.mBodies[i].mIsVirtual)
        {
            model.f_b[i].setIncludingFrame(frame, model.I[i] * model.a[i] + model.v[i] % (model.I[i] * model.v[i]));
        }
        else
        {
            model.f_b[i].setIncludingFrame(frame, 0., 0., 0., 0., 0., 0.);
        }
    }

    for (unsigned int i = model.mBodies.size() - 1; i > 0; i--)
    {
        if (model.mJoints[i].mJointType != JointTypeCustom)
        {
            if (model.mJoints[i].mDoFCount == 1)
            {
                Tau[model.mJoints[i].q_index] = model.S[i].dot(model.f_b[i]);
            }
            else if (model.mJoints[i].mDoFCount == 3)
            {
                Tau.block<3, 1>(model.mJoints[i].q_index, 0) = model.multdof3_S[i].transpose() * model.f_b[i];
            }
        }
        else
        {
            unsigned int k = model.mJoints[i].custom_joint_index;
            Tau.block(model.mJoints[i].q_index, 0, model.mCustomJoints[k]->mDoFCount, 1) = model.mCustomJoints[k]->S.transpose() * model.f_b[i];
        }

        if (model.lambda[i] != 0)
        {
            model.f_b[model.lambda[i]] += model.f_b[i].changeFrameAndCopy(model.bodyFrames[model.lambda[i]]);
        }
    }
}

void compositeRigidBodyAlgorithm(Model& model, const VectorNd& Q, MatrixNd& H, bool update_kinematics)
{
    assert(H.rows() == model.dof_count && H.cols() == model.dof_count);

    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        if (update_kinematics)
        {
            jcalc_X_lambda_S(model, i, Q);
        }
        model.Ic[i] = model.I[i];
    }

    for (unsigned int i = model.mBodies.size() - 1; i > 0; i--)
    {
        if (model.lambda[i] != 0)
        {
            model.Ic[model.lambda[i]] = model.Ic[model.lambda[i]] + model.Ic[i].transform_copy(model.bodyFrames[i]->getTransformToParent());
        }

        unsigned int dof_index_i = model.mJoints[i].q_index;

        if (model.mJoints[i].mDoFCount == 1 && model.mJoints[i].mJointType != JointTypeCustom)
        {
            Momentum F(model.Ic[i], model.S[i]);
            H(dof_index_i, dof_index_i) = model.S[i].dot(F);

            unsigned int j = i;
            unsigned int dof_index_j = dof_index_i;

            while (model.lambda[j] != 0)
            {
                F.transform(model.bodyFrames[j]->getTransformToParent());
                j = model.lambda[j];
                dof_index_j = model.mJoints[j].q_index;

                if (model.mJoints[j].mJointType != JointTypeCustom)
                {
                    if (model.mJoints[j].mDoFCount == 1)
                    {
                        H(dof_index_i, dof_index_j) = F.dot(model.S[j]);
                        H(dof_index_j, dof_index_i) = H(dof_index_i, dof_index_j);
                    }
                    else if (model.mJoints[j].mDoFCount == 3)
                    {
                        Vector3d H_temp2 = (F.transpose() * model.multdof3_S[j]).transpose();

                        H.block<1, 3>(dof_index_i, dof_index_j) = H_temp2.transpose();
                        H.block<3, 1>(dof_index_j, dof_index_i) = H_temp2;
                    }
                }
                else
                {
                    unsigned int k = model.mJoints[j].custom_joint_index;
                    unsigned int dof = model.mCustomJoints[k]->mDoFCount;
                    VectorNd H_temp2 = (F.transpose() * model.mCustomJoints[k]->S).transpose();

                    H.block(dof_index_i, dof_index_j, 1, dof) = H_temp2.transpose();
                    H.block(dof_index_j, dof_index_i, dof, 1) = H_temp2;
                }
            }
        }
        else if (model.mJoints[i].mDoFCount == 3 && model.mJoints[i].mJointType != JointTypeCustom)
        {
            Matrix63 F_63 = model.Ic[i].toMatrix() * model.multdof3_S[i];
            H.block<3, 3>(dof_index_i, dof_index_i) = model.multdof3_S[i].transpose() * F_63;

            unsigned int j = i;
            unsigned int dof_index_j = dof_index_i;

            while (model.lambda[j] != 0)
            {
                F_63 = model.bodyFrames[j]->getTransformFromParent().toMatrixTranspose() * (F_63);
                j = model.lambda[j];
                dof_index_j = model.mJoints[j].q_index;

                if (model.mJoints[j].mJointType != JointTypeCustom)
                {
                    if (model.mJoints[j].mDoFCount == 1)
                    {
                        Vector3d H_temp2 = F_63.transpose() * (model.S[j]);

                        H.block<3, 1>(dof_index_i, dof_index_j) = H_temp2;
                        H.block<1, 3>(dof_index_j, dof_index_i) = H_temp2.transpose();
                    }
                    else if (model.mJoints[j].mDoFCount == 3)
                    {
                        Matrix3d H_temp2 = F_63.transpose() * (model.multdof3_S[j]);

                        H.block<3, 3>(dof_index_i, dof_index_j) = H_temp2;
                        H.block<3, 3>(dof_index_j, dof_index_i) = H_temp2.transpose();
                    }
                }
                else
                {
                    unsigned int k = model.mJoints[j].custom_joint_index;
                    unsigned int dof = model.mCustomJoints[k]->mDoFCount;

                    MatrixNd H_temp2 = F_63.transpose() * (model.mCustomJoints[k]->S);

                    H.block(dof_index_i, dof_index_j, 3, dof) = H_temp2;
                    H.block(dof_index_j, dof_index_i, dof, 3) = H_temp2.transpose();
                }
            }
        }
        else if (model.mJoints[i].mJointType == JointTypeCustom)
        {
            unsigned int kI = model.mJoints[i].custom_joint_index;
            unsigned int dofI = model.mCustomJoints[kI]->mDoFCount;

            MatrixNd F_Nd = model.Ic[i].toMatrix() * model.mCustomJoints[kI]->S;

            H.block(dof_index_i, dof_index_i, dofI, dofI) = model.mCustomJoints[kI]->S.transpose() * F_Nd;

            unsigned int j = i;
            unsigned int dof_index_j = dof_index_i;

            while (model.lambda[j] != 0)
            {
                F_Nd = model.bodyFrames[j]->getTransformFromParent().toMatrixTranspose() * (F_Nd);
                j = model.lambda[j];
                dof_index_j = model.mJoints[j].q_index;

                if (model.mJoints[j].mJointType != JointTypeCustom)
                {
                    if (model.mJoints[j].mDoFCount == 1)
                    {
                        MatrixNd H_temp2 = F_Nd.transpose() * (model.S[j]);
                        H.block(dof_index_i, dof_index_j, H_temp2.rows(), H_temp2.cols()) = H_temp2;
                        H.block(dof_index_j, dof_index_i, H_temp2.cols(), H_temp2.rows()) = H_temp2.transpose();
                    }
                    else if (model.mJoints[j].mDoFCount == 3)
                    {
                        MatrixNd H_temp2 = F_Nd.transpose() * (model.multdof3_S[j]);
                        H.block(dof_index_i, dof_index_j, H_temp2.rows(), H_temp2.cols()) = H_temp2;
                        H.block(dof_index_j, dof_index_i, H_temp2.cols(), H_temp2.rows()) = H_temp2.transpose();
                    }
                }
                else
                {
                    unsigned int k = model.mJoints[j].custom_joint_index;
                    unsigned int dof = model.mCustomJoints[k]->mDoFCount;

                    MatrixNd H_temp2 = F_Nd.transpose() * (model.mCustomJoints[k]->S);

                    H.block(dof_index_i, dof_index_j, 3, dof) = H_temp2;
                    H.block(dof_index_j, dof_index_i, dof, 3) = H_temp2.transpose();
                }
            }
        }
    }
}

void forwardDynamics(Model& model, const VectorNd& Q, const VectorNd& QDot, const VectorNd& Tau, VectorNd& QDDot, SpatialForceV* f_ext, bool update_kinematics)
{
    // Reset the velocity of the root body
    model.v[0].setZero();

    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        unsigned int lambda = model.lambda[i];
        ReferenceFramePtr bodyFrame = model.bodyFrames[i];

        if (update_kinematics)
        {
            jcalc(model, i, Q, QDot);
            model.v[i].set(model.v[lambda].transform_copy(bodyFrame->getTransformFromParent()) + model.v_J[i]);
            model.c[i] = model.c_J[i] + SpatialVector(model.v[i] % model.v_J[i]);
        }

        model.I[i].setSpatialMatrix(model.IA[i]);
        model.pA[i].setIncludingFrame(bodyFrame, f_ext == nullptr ? model.v[i].cross(model.I[i] * model.v[i]) :
                                                                    model.v[i].cross(model.I[i] * model.v[i]) - (*f_ext)[i].changeFrameAndCopy(bodyFrame));
    }

    for (unsigned int i = model.mBodies.size() - 1; i > 0; i--)
    {
        unsigned int q_index = model.mJoints[i].q_index;
        ReferenceFramePtr bodyFrame = model.bodyFrames[i];

        if (model.mJoints[i].mDoFCount == 1 && model.mJoints[i].mJointType != JointTypeCustom)
        {
            model.U[i] = model.IA[i] * model.S[i];
            model.d[i] = model.S[i].dot(model.U[i]);
            model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);

            unsigned int lambda = model.lambda[i];
            if (lambda != 0)
            {
                SpatialMatrix Ia = model.IA[i] - model.U[i] * (model.U[i] / model.d[i]).transpose();
                RobotDynamics::Math::SpatialForce pa(RobotDynamics::Math::SpatialForce(bodyFrame, model.pA[i] + Ia * model.c[i] + model.U[i] * model.u[i] / model.d[i]));

                model.IA[lambda].noalias() += bodyFrame->getTransformFromParent().toMatrixTranspose() * Ia * bodyFrame->getTransformFromParent().toMatrix();
                model.pA[lambda] += pa.changeFrameAndCopy(model.bodyFrames[model.lambda[i]]);
            }
        }
        else if (model.mJoints[i].mDoFCount == 3 && model.mJoints[i].mJointType != JointTypeCustom)
        {
            model.multdof3_U[i] = model.IA[i] * model.multdof3_S[i];

            model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose() * model.multdof3_U[i]).inverse().eval();

            Vector3d tau_temp(Tau[q_index], Tau[q_index + 1], Tau[q_index + 2]);
            model.multdof3_u[i] = tau_temp - model.multdof3_S[i].transpose() * model.pA[i];

            unsigned int lambda = model.lambda[i];
            if (lambda != 0)
            {
                SpatialMatrix Ia = model.IA[i] - model.multdof3_U[i] * model.multdof3_Dinv[i] * model.multdof3_U[i].transpose();
                RobotDynamics::Math::SpatialForce pa(
                    model.pA[i] + RobotDynamics::Math::SpatialForce(bodyFrame, Ia * model.c[i] + model.multdof3_U[i] * model.multdof3_Dinv[i] * model.multdof3_u[i]));

                model.IA[lambda].noalias() += bodyFrame->getTransformFromParent().toMatrixTranspose() * Ia * bodyFrame->getTransformFromParent().toMatrix();
                model.pA[lambda] += pa.changeFrameAndCopy(model.bodyFrames[model.lambda[i]]);
            }
        }
        else if (model.mJoints[i].mJointType == JointTypeCustom)
        {
            unsigned int kI = model.mJoints[i].custom_joint_index;
            model.mCustomJoints[kI]->U = model.IA[i] * model.mCustomJoints[kI]->S;

            model.mCustomJoints[kI]->Dinv = (model.mCustomJoints[kI]->S.transpose() * model.mCustomJoints[kI]->U).inverse().eval();

            model.mCustomJoints[kI]->u = Tau.block(q_index, 0, model.mCustomJoints[kI]->mDoFCount, 1) - model.mCustomJoints[kI]->S.transpose() * model.pA[i];

            unsigned int lambda = model.lambda[i];
            if (lambda != 0)
            {
                SpatialMatrix Ia = model.IA[i] - (model.mCustomJoints[kI]->U * model.mCustomJoints[kI]->Dinv * model.mCustomJoints[kI]->U.transpose());
                RobotDynamics::Math::SpatialForce pa(
                    model.pA[i] + RobotDynamics::Math::SpatialForce(
                                      bodyFrame, Ia * model.c[i] + (model.mCustomJoints[kI]->U * model.mCustomJoints[kI]->Dinv * model.mCustomJoints[kI]->u)));

                model.IA[lambda].noalias() += bodyFrame->getTransformFromParent().toMatrixTranspose() * Ia * bodyFrame->getTransformFromParent().toMatrix();
                model.pA[lambda] += pa.changeFrameAndCopy(model.bodyFrames[model.lambda[i]]);
            }
        }
    }

    model.a[0].set(model.gravity * -1.);

    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        unsigned int q_index = model.mJoints[i].q_index;

        model.a[i].set(model.a[model.lambda[i]].transform_copy(model.bodyFrames[i]->getTransformFromParent()) + model.c[i]);

        if (model.mJoints[i].mDoFCount == 1 && model.mJoints[i].mJointType != JointTypeCustom)
        {
            QDDot[q_index] = (1. / model.d[i]) * (model.u[i] - model.U[i].dot(model.a[i]));
            model.a[i].set(model.a[i] + model.S[i] * QDDot[q_index]);
        }
        else if (model.mJoints[i].mDoFCount == 3 && model.mJoints[i].mJointType != JointTypeCustom)
        {
            Vector3d qdd_temp = model.multdof3_Dinv[i] * (model.multdof3_u[i] - model.multdof3_U[i].transpose() * model.a[i]);
            QDDot[q_index] = qdd_temp[0];
            QDDot[q_index + 1] = qdd_temp[1];
            QDDot[q_index + 2] = qdd_temp[2];
            model.a[i].set(model.a[i] + model.multdof3_S[i] * qdd_temp);
        }
        else if (model.mJoints[i].mJointType == JointTypeCustom)
        {
            unsigned int kI = model.mJoints[i].custom_joint_index;
            unsigned int dofI = model.mCustomJoints[kI]->mDoFCount;

            VectorNd qdd_temp = model.mCustomJoints[kI]->Dinv * (model.mCustomJoints[kI]->u - model.mCustomJoints[kI]->U.transpose() * model.a[i]);

            for (unsigned int z = 0; z < dofI; ++z)
            {
                QDDot[q_index + z] = qdd_temp[z];
            }

            model.a[i].set(model.a[i] + model.mCustomJoints[kI]->S * qdd_temp);
        }
    }
}

void forwardDynamicsLagrangian(Model& model, const VectorNd& Q, const VectorNd& QDot, const VectorNd& Tau, VectorNd& QDDot, Math::MatrixNd& H, Math::VectorNd& C,
                               Math::LinearSolver linear_solver, SpatialForceV* f_ext, bool update_kinematics)
{
    (void)linear_solver;
    assert(H.rows() == model.dof_count && H.cols() == model.dof_count);
    assert(C.rows() == model.dof_count);

    // we set QDDot to zero to compute C properly with the InverseDynamics
    // method.
    QDDot.setZero();

    inverseDynamics(model, Q, QDot, QDDot, C, f_ext, update_kinematics);
    compositeRigidBodyAlgorithm(model, Q, H, false);

    if (!linSolveGaussElimPivot(H, C * -1. + Tau, QDDot))
    {
        throw std::runtime_error("forwardDynamicsContactsKokkevis: linSolveGaussElimPivot failed to solve");
    }
}

void calcMInvTimesTau(Model& model, const VectorNd& Q, const VectorNd& Tau, VectorNd& QDDot, bool update_kinematics)
{
    model.v[0].setZero();
    model.a[0].setZero();

    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        if (update_kinematics)
        {
            jcalc_X_lambda_S(model, i, Q);
        }

        model.pA[i].setIncludingFrame(model.bodyFrames[i], 0., 0., 0., 0., 0., 0.);
        model.I[i].setSpatialMatrix(model.IA[i]);
    }

    // Compute Articulate Body Inertias
    for (unsigned int i = model.mBodies.size() - 1; i > 0; i--)
    {
        RobotDynamics::ReferenceFramePtr frame = model.bodyFrames[i];
        if (model.mJoints[i].mDoFCount == 1 && model.mJoints[i].mJointType != JointTypeCustom)
        {
            model.U[i] = model.IA[i] * model.S[i];
            model.d[i] = model.S[i].dot(model.U[i]);
            unsigned int lambda = model.lambda[i];

            if (lambda != 0)
            {
                SpatialMatrix Ia = model.IA[i] - model.U[i] * (model.U[i] / model.d[i]).transpose();
                model.IA[lambda].noalias() += frame->getTransformFromParent().toMatrixTranspose() * Ia * frame->getTransformFromParent().toMatrix();
            }
        }
        else if (model.mJoints[i].mDoFCount == 3 && model.mJoints[i].mJointType != JointTypeCustom)
        {
            model.multdof3_U[i] = model.IA[i] * model.multdof3_S[i];
            model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose() * model.multdof3_U[i]).inverse().eval();

            unsigned int lambda = model.lambda[i];

            if (lambda != 0)
            {
                SpatialMatrix Ia = model.IA[i] - (model.multdof3_U[i] * model.multdof3_Dinv[i] * model.multdof3_U[i].transpose());
                model.IA[lambda].noalias() += frame->getTransformFromParent().toMatrixTranspose() * Ia * frame->getTransformFromParent().toMatrix();
            }
        }
        else if (model.mJoints[i].mJointType == JointTypeCustom)
        {
            unsigned int kI = model.mJoints[i].custom_joint_index;
            model.mCustomJoints[kI]->U = model.IA[i] * model.mCustomJoints[kI]->S;
            model.mCustomJoints[kI]->Dinv = (model.mCustomJoints[kI]->S.transpose() * model.mCustomJoints[kI]->U).inverse().eval();

            unsigned int lambda = model.lambda[i];

            if (lambda != 0)
            {
                SpatialMatrix Ia = model.IA[i] - (model.mCustomJoints[kI]->U * model.mCustomJoints[kI]->Dinv * model.mCustomJoints[kI]->U.transpose());
                model.IA[lambda].noalias() += frame->getTransformFromParent().toMatrixTranspose() * Ia * frame->getTransformFromParent().toMatrix();
            }
        }
    }

    // compute articulated bias forces
    for (unsigned int i = model.mBodies.size() - 1; i > 0; i--)
    {
        unsigned int q_index = model.mJoints[i].q_index;
        RobotDynamics::ReferenceFramePtr frame = model.bodyFrames[i];

        if (model.mJoints[i].mDoFCount == 1 && model.mJoints[i].mJointType != JointTypeCustom)
        {
            model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);
            unsigned int lambda = model.lambda[i];
            if (lambda != 0)
            {
                RobotDynamics::Math::SpatialForce pa = model.pA[i] + RobotDynamics::Math::SpatialForce(frame, model.U[i] * model.u[i] / model.d[i]);
                model.pA[lambda] += pa.changeFrameAndCopy(model.bodyFrames[lambda]);
            }
        }
        else if (model.mJoints[i].mDoFCount == 3 && model.mJoints[i].mJointType != JointTypeCustom)
        {
            Vector3d tau_temp(Tau[q_index], Tau[q_index + 1], Tau[q_index + 2]);
            model.multdof3_u[i] = tau_temp - model.multdof3_S[i].transpose() * model.pA[i];
            unsigned int lambda = model.lambda[i];

            if (lambda != 0)
            {
                RobotDynamics::Math::SpatialForce pa =
                    model.pA[i] + RobotDynamics::Math::SpatialForce(frame, model.multdof3_U[i] * model.multdof3_Dinv[i] * model.multdof3_u[i]);
                model.pA[lambda] += pa.changeFrameAndCopy(model.bodyFrames[lambda]);
            }
        }
        else if (model.mJoints[i].mJointType == JointTypeCustom)
        {
            unsigned int kI = model.mJoints[i].custom_joint_index;
            unsigned int dofI = model.mCustomJoints[kI]->mDoFCount;
            VectorNd tau_temp = model.mCustomJoints[kI]->ndof0_vec;

            for (unsigned int z = 0; z < dofI; ++z)
            {
                tau_temp(z) = Tau[q_index + z];
            }
            model.mCustomJoints[kI]->u = tau_temp - (model.mCustomJoints[kI]->S.transpose() * model.pA[i]);

            unsigned int lambda = model.lambda[i];

            if (lambda != 0)
            {
                RobotDynamics::Math::SpatialForce pa =
                    model.pA[i] + SpatialForce(frame, model.mCustomJoints[kI]->U * model.mCustomJoints[kI]->Dinv * model.mCustomJoints[kI]->u);
                model.pA[lambda] += pa.changeFrameAndCopy(model.bodyFrames[lambda]);
            }
        }
    }

    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        unsigned int q_index = model.mJoints[i].q_index;
        unsigned int lambda = model.lambda[i];

        model.a[i].set(model.bodyFrames[i]->getTransformFromParent().apply(model.a[lambda]));

        if (model.mJoints[i].mDoFCount == 1 && model.mJoints[i].mJointType != JointTypeCustom)
        {
            QDDot[q_index] = (1. / model.d[i]) * (model.u[i] - model.U[i].dot(model.a[i]));
            model.a[i].set(model.a[i] + model.S[i] * QDDot[q_index]);
        }
        else if (model.mJoints[i].mDoFCount == 3 && model.mJoints[i].mJointType != JointTypeCustom)
        {
            Vector3d qdd_temp = model.multdof3_Dinv[i] * (model.multdof3_u[i] - model.multdof3_U[i].transpose() * model.a[i]);

            QDDot[q_index] = qdd_temp[0];
            QDDot[q_index + 1] = qdd_temp[1];
            QDDot[q_index + 2] = qdd_temp[2];
            model.a[i].set(model.a[i] + model.multdof3_S[i] * qdd_temp);
        }
        else if (model.mJoints[i].mJointType == JointTypeCustom)
        {
            unsigned int kI = model.mJoints[i].custom_joint_index;
            unsigned int dofI = model.mCustomJoints[kI]->mDoFCount;

            VectorNd qdd_temp = model.mCustomJoints[kI]->Dinv * (model.mCustomJoints[kI]->u - model.mCustomJoints[kI]->U.transpose() * model.a[i]);

            for (unsigned int z = 0; z < dofI; ++z)
            {
                QDDot[q_index + z] = qdd_temp[z];
            }

            model.a[i].set(model.a[i] + model.mCustomJoints[kI]->S * qdd_temp);
        }
    }
}

} /* namespace RobotDynamics */
