//
// RDL - Robot Dynamics Library
// Modifications Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
//
// Licensed under the zlib license. See LICENSE for more details.
//

//
// Created by Jordan Lack on 3/28/17.
//

#include <iomanip>
#include <rdl_dynamics/HydroDynamics.hpp>

namespace RobotDynamics
{
void forwardHydroDynamics(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, const Math::VectorNd& Tau, Math::VectorNd& QDDot, Math::SpatialForceV* f_ext,
                          bool update_kinematics)
{
    // Reset the velocity of the root body
    model.v[0].setZero();
    // Assume fluid is not rotating
    model.a_f.setAngularPart(Math::Vector3dZero);
    model.v_f.setAngularPart(Math::Vector3dZero);

    /**
     * Forward pass over all bodies to update the kinematics as well as calculate the bias force(s) and add in
     * any external forces if any.
     */
    Math::MotionVector v_body_wrt_fluid;
    Math::MotionVector a_f_g = model.a_f - model.gravity;
    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        unsigned int lambda = model.lambda[i];
        ReferenceFramePtr bodyFrame = model.bodyFrames[i];
        ReferenceFramePtr centerOfBuoyancyFrame = model.bodyCenterOfBuoyancyFrames[i];

        if (update_kinematics)
        {
            jcalc(model, i, Q, QDot);
            model.v[i].set(model.v[lambda].transform_copy(bodyFrame->getTransformFromParent()) + model.v_J[i]);
            model.c[i] = model.c_J[i] + Math::SpatialVector(model.v[i] % model.v_J[i]);
        }

        model.IA[i] = model.I_H[i];
        model.pA[i].setIncludingFrame(bodyFrame, model.v[i].cross(model.I[i] * model.v[i]));

        // Transform the fluid acceleration and gravity to center of buoyancy frame, then take the difference and multiply
        // by the mass of the fluid displaced

        // Buoyancy bias force
        model.pA[i] -= Math::SpatialForce(centerOfBuoyancyFrame,
                                          model.mBodies[i].volume * model.fluidDensity * a_f_g.transform_copy(centerOfBuoyancyFrame->getInverseTransformToRoot()))
                           .changeFrameAndCopy(bodyFrame);
        // Compute body velocity w.r.t. surrounding fluid. Assuming the fluid is not rotating
        v_body_wrt_fluid = Math::MotionVector(model.v[i] - model.v_f.transform_copy(bodyFrame->getInverseTransformToRoot()));

        Math::MotionVector v_body_wrt_fluid_no_angular;
        v_body_wrt_fluid_no_angular.setLinearPart(v_body_wrt_fluid.getAngularPart().cross(v_body_wrt_fluid.getLinearPart()));

        // added_mass_bias_term = I^A * [0;a_f_g] + I^A * [0;w_b x v_b_r]  + tilde(v_b_r) * I^A * v_b_r where v_b_r is the body's velocity relative to the fluid
        model.pA[i] -= Math::SpatialForce(bodyFrame, model.I_add[i] * (a_f_g.transform_copy(bodyFrame->getInverseTransformToRoot()) + v_body_wrt_fluid_no_angular));
        model.pA[i] += Math::SpatialForce(bodyFrame, v_body_wrt_fluid % Math::ForceVector(model.I_add[i] * v_body_wrt_fluid));

        // Compute bias forces from pressure drag

        DragData dragData = model.mBodies[i].dragData;

        // f_drag = -(a*v + b*v*|v|)
        model.pA[i] -=
            Math::SpatialForce(bodyFrame, v_body_wrt_fluid.cwiseProduct(dragData.linearDrag + dragData.quadraticDrag.cwiseProduct(v_body_wrt_fluid.cwiseAbs())));

        if (f_ext != NULL && (*f_ext)[i] != Math::SpatialVectorZero)
        {
            model.pA[i] -= (*f_ext)[i].changeFrameAndCopy(bodyFrame);
        }
    }

    /**
     * Do a backwards pass over the bodies, calculating the articulated body inertias and articulated
     * bias forces(see RBDA p. 128). Also calculates some intermediate values(U,d,u) to reduce the
     * number of duplicate operations.
     */
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
                Math::SpatialMatrix Ia = model.IA[i] - model.U[i] * (model.U[i] / model.d[i]).transpose();
                Math::SpatialForce pa(bodyFrame, model.pA[i] + Ia * model.c[i] + model.U[i] * model.u[i] / model.d[i]);

                model.IA[lambda].noalias() += bodyFrame->getTransformFromParent().toMatrixTranspose() * Ia * bodyFrame->getTransformFromParent().toMatrix();
                model.pA[lambda].noalias() += pa.changeFrameAndCopy(model.bodyFrames[lambda]);
            }
        }
        else if (model.mJoints[i].mDoFCount == 3 && model.mJoints[i].mJointType != JointTypeCustom)
        {
            model.multdof3_U[i] = model.IA[i] * model.multdof3_S[i];

            model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose() * model.multdof3_U[i]).inverse().eval();

            Math::Vector3d tau_temp(Tau[q_index], Tau[q_index + 1], Tau[q_index + 2]);
            model.multdof3_u[i] = tau_temp - model.multdof3_S[i].transpose() * model.pA[i];

            unsigned int lambda = model.lambda[i];
            if (lambda != 0)
            {
                Math::SpatialMatrix Ia = model.IA[i] - model.multdof3_U[i] * model.multdof3_Dinv[i] * model.multdof3_U[i].transpose();
                Math::SpatialForce pa(bodyFrame, model.pA[i] + Ia * model.c[i] + model.multdof3_U[i] * model.multdof3_Dinv[i] * model.multdof3_u[i]);

                model.IA[lambda].noalias() += bodyFrame->getTransformFromParent().toMatrixTranspose() * Ia * bodyFrame->getTransformFromParent().toMatrix();
                model.pA[lambda].noalias() += pa.changeFrameAndCopy(model.bodyFrames[lambda]);
            }
        }
        else if (model.mJoints[i].mJointType == JointTypeCustom)
        {
            unsigned int kI = model.mJoints[i].custom_joint_index;
            unsigned int dofI = model.mCustomJoints[kI]->mDoFCount;
            model.mCustomJoints[kI]->U = model.IA[i] * model.mCustomJoints[kI]->S;

            model.mCustomJoints[kI]->Dinv = (model.mCustomJoints[kI]->S.transpose() * model.mCustomJoints[kI]->U).inverse().eval();

            Math::VectorNd tau_temp(dofI);
            for (unsigned int z = 0; z < dofI; ++z)
            {
                tau_temp(z) = Tau[q_index + z];
            }
            model.mCustomJoints[kI]->u = tau_temp - model.mCustomJoints[kI]->S.transpose() * model.pA[i];

            unsigned int lambda = model.lambda[i];
            if (lambda != 0)
            {
                Math::SpatialMatrix Ia = model.IA[i] - (model.mCustomJoints[kI]->U * model.mCustomJoints[kI]->Dinv * model.mCustomJoints[kI]->U.transpose());
                Math::SpatialForce pa(bodyFrame,
                                      model.pA[i] + Ia * model.c[i] + (model.mCustomJoints[kI]->U * model.mCustomJoints[kI]->Dinv * model.mCustomJoints[kI]->u));

                model.IA[lambda].noalias() += bodyFrame->getTransformFromParent().toMatrixTranspose() * Ia * bodyFrame->getTransformFromParent().toMatrix();
                model.pA[lambda].noalias() += pa.changeFrameAndCopy(model.bodyFrames[lambda]);
            }
        }
    }

    model.a[0].set(model.gravity * -1.);

    /**
     * Do a final forward pass over the bodies, calculating the accelerations
     */
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
            Math::Vector3d qdd_temp = model.multdof3_Dinv[i] * (model.multdof3_u[i] - model.multdof3_U[i].transpose() * model.a[i]);
            QDDot[q_index] = qdd_temp[0];
            QDDot[q_index + 1] = qdd_temp[1];
            QDDot[q_index + 2] = qdd_temp[2];
            model.a[i].set(model.a[i] + model.multdof3_S[i] * qdd_temp);
        }
        else if (model.mJoints[i].mJointType == JointTypeCustom)
        {
            unsigned int kI = model.mJoints[i].custom_joint_index;
            unsigned int dofI = model.mCustomJoints[kI]->mDoFCount;

            Math::VectorNd qdd_temp = model.mCustomJoints[kI]->Dinv * (model.mCustomJoints[kI]->u - model.mCustomJoints[kI]->U.transpose() * model.a[i]);

            for (unsigned int z = 0; z < dofI; ++z)
            {
                QDDot[q_index + z] = qdd_temp[z];
            }

            model.a[i].set(model.a[i] + model.mCustomJoints[kI]->S * qdd_temp);
        }
    }
}

void compositeRigidBodyAlgorithmHydro(Model& model, const Math::VectorNd& Q, Math::MatrixNd& H, bool update_kinematics)
{
    assert(H.rows() == model.dof_count && H.cols() == model.dof_count);

    /**
     *  Update the kinematic parameters and calculate the combined(rigid body & hydrodynamic inertia) inertia to
     *  a modifiable location
     */
    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        if (update_kinematics)
        {
            jcalc_X_lambda_S(model, i, Q);
        }

        model.Ic_H[i] = model.I_H[i];
    }

    /**
     * Do a backwards pass over the bodies, populating all elements of the inertia matrix that are not zero.
     */
    for (unsigned int i = model.mBodies.size() - 1; i > 0; i--)
    {
        if (model.lambda[i] != 0)
        {
            /**
             *  Calculate the inertia of the subtree rooted at body i assuming that subtree is a single composite rigid body
             */
            model.Ic_H[model.lambda[i]] = model.Ic_H[model.lambda[i]] + model.bodyFrames[i]->getTransformFromParent().toMatrixTranspose() * model.Ic_H[i] *
                                                                            model.bodyFrames[i]->getTransformFromParent().toMatrix();
        }

        unsigned int dof_index_i = model.mJoints[i].q_index;

        if (model.mJoints[i].mDoFCount == 1 && model.mJoints[i].mJointType != JointTypeCustom)
        {
            /**
             * Main diagonal term
             */
            Math::Momentum F(model.Ic_H[i] * model.S[i]);
            H(dof_index_i, dof_index_i) = model.S[i].dot(F);

            unsigned int j = i;
            unsigned int dof_index_j = dof_index_i;

            /**
             * Calculate the non-zero, off diagonal terms
             */
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
                        Math::Vector3d H_temp2 = (F.transpose() * model.multdof3_S[j]).transpose();

                        H.block<1, 3>(dof_index_i, dof_index_j) = H_temp2.transpose();
                        H.block<3, 1>(dof_index_j, dof_index_i) = H_temp2;
                    }
                }
                else
                {
                    unsigned int k = model.mJoints[j].custom_joint_index;
                    unsigned int dof = model.mCustomJoints[k]->mDoFCount;
                    Math::VectorNd H_temp2 = (F.transpose() * model.mCustomJoints[k]->S).transpose();

                    H.block(dof_index_i, dof_index_j, 1, dof) = H_temp2.transpose();
                    H.block(dof_index_j, dof_index_i, dof, 1) = H_temp2;
                }
            }
        }
        else if (model.mJoints[i].mDoFCount == 3 && model.mJoints[i].mJointType != JointTypeCustom)
        {
            Math::Matrix63 F_63 = model.Ic_H[i] * model.multdof3_S[i];
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
                        Math::Vector3d H_temp2 = F_63.transpose() * (model.S[j]);

                        H.block<3, 1>(dof_index_i, dof_index_j) = H_temp2;
                        H.block<1, 3>(dof_index_j, dof_index_i) = H_temp2.transpose();
                    }
                    else if (model.mJoints[j].mDoFCount == 3)
                    {
                        Math::Matrix3d H_temp2 = F_63.transpose() * (model.multdof3_S[j]);

                        H.block<3, 3>(dof_index_i, dof_index_j) = H_temp2;
                        H.block<3, 3>(dof_index_j, dof_index_i) = H_temp2.transpose();
                    }
                }
                else
                {
                    unsigned int k = model.mJoints[j].custom_joint_index;
                    unsigned int dof = model.mCustomJoints[k]->mDoFCount;

                    Math::MatrixNd H_temp2 = F_63.transpose() * (model.mCustomJoints[k]->S);

                    H.block(dof_index_i, dof_index_j, 3, dof) = H_temp2;
                    H.block(dof_index_j, dof_index_i, dof, 3) = H_temp2.transpose();
                }
            }
        }
        else if (model.mJoints[i].mJointType == JointTypeCustom)
        {
            unsigned int kI = model.mJoints[i].custom_joint_index;
            unsigned int dofI = model.mCustomJoints[kI]->mDoFCount;

            Math::MatrixNd F_Nd = model.Ic_H[i] * model.mCustomJoints[kI]->S;

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
                        Math::MatrixNd H_temp2 = F_Nd.transpose() * (model.S[j]);
                        H.block(dof_index_i, dof_index_j, H_temp2.rows(), H_temp2.cols()) = H_temp2;
                        H.block(dof_index_j, dof_index_i, H_temp2.cols(), H_temp2.rows()) = H_temp2.transpose();
                    }
                    else if (model.mJoints[j].mDoFCount == 3)
                    {
                        Math::MatrixNd H_temp2 = F_Nd.transpose() * (model.multdof3_S[j]);
                        H.block(dof_index_i, dof_index_j, H_temp2.rows(), H_temp2.cols()) = H_temp2;
                        H.block(dof_index_j, dof_index_i, H_temp2.cols(), H_temp2.rows()) = H_temp2.transpose();
                    }
                }
                else
                {
                    unsigned int k = model.mJoints[j].custom_joint_index;
                    unsigned int dof = model.mCustomJoints[k]->mDoFCount;

                    Math::MatrixNd H_temp2 = F_Nd.transpose() * (model.mCustomJoints[k]->S);

                    H.block(dof_index_i, dof_index_j, 3, dof) = H_temp2;
                    H.block(dof_index_j, dof_index_i, dof, 3) = H_temp2.transpose();
                }
            }
        }
    }
}

void nonlinearEffectsHydro(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, Math::VectorNd& Tau, bool update_kinematics)
{
    model.v[0].setZero();
    model.a[0].set(-model.gravity);
    model.a_f.setAngularPart(Math::Vector3dZero);
    model.v_f.setAngularPart(Math::Vector3dZero);

    Math::MotionVector v_body_wrt_fluid;
    Math::MotionVector a_f_g = model.a_f - model.gravity;
    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        ReferenceFramePtr bodyFrame = model.bodyFrames[i];
        ReferenceFramePtr centerOfBuoyancyFrame = model.bodyCenterOfBuoyancyFrames[i];
        if (model.lambda[i] == 0)
        {
            if (update_kinematics)
            {
                jcalc(model, i, Q, QDot);
                model.v[i].set(model.v_J[i]);
            }

            model.a[i].set(-model.gravity.transform_copy(bodyFrame->getTransformFromParent()));
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
            model.f_b[i].setIncludingFrame(bodyFrame, model.I_H[i] * model.a[i] + model.v[i] % (Math::Momentum(model.I[i], model.v[i])));

            // Buoyancy bias force
            Math::SpatialForce fb(centerOfBuoyancyFrame,
                                  model.mBodies[i].volume * model.fluidDensity * a_f_g.transform_copy(centerOfBuoyancyFrame->getInverseTransformToRoot()));
            model.f_b[i] -= fb.changeFrameAndCopy(bodyFrame);
            // Compute body velocity w.r.t. surrounding fluid. Assuming the fluid is not rotating
            v_body_wrt_fluid = Math::MotionVector(model.v[i] - model.v_f.transform_copy(bodyFrame->getInverseTransformToRoot()));

            Math::MotionVector v_body_wrt_fluid_no_angular;
            v_body_wrt_fluid_no_angular.setLinearPart(v_body_wrt_fluid.getAngularPart().cross(v_body_wrt_fluid.getLinearPart()));

            // added mass bias force
            model.f_b[i] -= Math::SpatialForce(bodyFrame, model.I_add[i] * (a_f_g.transform_copy(bodyFrame->getInverseTransformToRoot()) + v_body_wrt_fluid_no_angular));
            model.f_b[i] += Math::SpatialForce(bodyFrame, v_body_wrt_fluid % Math::ForceVector(model.I_add[i] * v_body_wrt_fluid));

            // Compute bias forces from pressure drag
            DragData dragData = model.mBodies[i].dragData;
            model.f_b[i] -=
                Math::SpatialForce(bodyFrame, v_body_wrt_fluid.cwiseProduct(dragData.linearDrag + dragData.quadraticDrag.cwiseProduct(v_body_wrt_fluid.cwiseAbs())));
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

void coriolisEffectsHydro(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, Math::VectorNd& Tau, bool update_kinematics)
{
    model.v[0].setZero();
    model.a[0].setZero();
    model.a_f.setAngularPart(Math::Vector3dZero);
    model.v_f.setAngularPart(Math::Vector3dZero);

    Math::MotionVector v_body_wrt_fluid;
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
            model.f_b[i].setIncludingFrame(bodyFrame, model.I_H[i] * model.a[i] + model.v[i] % (Math::Momentum(model.I[i], model.v[i])));

            // Compute body velocity w.r.t. surrounding fluid. Assuming the fluid is not rotating
            v_body_wrt_fluid = Math::MotionVector(model.v[i] - model.v_f.transform_copy(bodyFrame->getInverseTransformToRoot()));

            Math::MotionVector v_body_wrt_fluid_no_angular;
            v_body_wrt_fluid_no_angular.setLinearPart(v_body_wrt_fluid.getAngularPart().cross(v_body_wrt_fluid.getLinearPart()));

            // added mass bias force
            model.f_b[i] -=
                Math::SpatialForce(bodyFrame, model.I_add[i] * (model.a_f.transform_copy(bodyFrame->getInverseTransformToRoot()) + v_body_wrt_fluid_no_angular));
            model.f_b[i] += Math::SpatialForce(bodyFrame, v_body_wrt_fluid % Math::ForceVector(model.I_add[i] * v_body_wrt_fluid));
        }
        else
        {
            model.f_b[i].setZero();
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

void inverseDynamicsHydro(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, const Math::VectorNd& QDDot, Math::VectorNd& Tau, Math::SpatialForceV* f_ext,
                          bool update_kinematics)
{
    // Reset the velocity of the root body
    model.v[0].setZero();
    model.a[0].set(-model.gravity);

    Math::MotionVector v_body_wrt_fluid;
    Math::MotionVector a_f_g = model.a_f - model.gravity;
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
                               model.multdof3_S[i] * Math::Vector3d(QDDot[q_index], QDDot[q_index + 1], QDDot[q_index + 2]));
            }
        }
        else
        {
            unsigned int k = model.mJoints[i].custom_joint_index;
            Math::VectorNd customJointQDDot(model.mCustomJoints[k]->mDoFCount);
            for (unsigned int z = 0; z < model.mCustomJoints[k]->mDoFCount; ++z)
            {
                customJointQDDot[z] = QDDot[q_index + z];
            }
            model.a[i].set(bodyFrame->getTransformFromParent().apply(model.a[lambda]) + model.c[i] + model.mCustomJoints[k]->S * customJointQDDot);
        }

        // If body is not virtual, calculate the body forces
        if (!model.mBodies[i].mIsVirtual)
        {
            ReferenceFramePtr centerOfBuoyancyFrame = model.bodyCenterOfBuoyancyFrames[i];

            model.f_b[i].setIncludingFrame(bodyFrame, model.I_H[i] * model.a[i] + model.v[i] % (Math::Momentum(model.I[i], model.v[i])));

            // Buoyancy bias force

            model.f_b[i] -= Math::SpatialForce(centerOfBuoyancyFrame,
                                               model.mBodies[i].volume * model.fluidDensity * a_f_g.transform_copy(centerOfBuoyancyFrame->getInverseTransformToRoot()))
                                .changeFrameAndCopy(bodyFrame);

            // Compute body velocity w.r.t. surrounding fluid. Assuming the fluid is not rotating
            v_body_wrt_fluid = Math::MotionVector(model.v[i] - model.v_f.transform_copy(bodyFrame->getInverseTransformToRoot()));

            Math::MotionVector v_body_wrt_fluid_no_angular;
            v_body_wrt_fluid_no_angular.setLinearPart(v_body_wrt_fluid.getAngularPart().cross(v_body_wrt_fluid.getLinearPart()));

            // added mass bias force
            model.f_b[i] -= Math::SpatialForce(bodyFrame, model.I_add[i] * (a_f_g.transform_copy(bodyFrame->getInverseTransformToRoot()) + v_body_wrt_fluid_no_angular));

            model.f_b[i] += Math::SpatialForce(bodyFrame, v_body_wrt_fluid % Math::ForceVector(model.I_add[i] * v_body_wrt_fluid));

            // Compute bias forces from pressure drag
            DragData dragData = model.mBodies[i].dragData;
            model.f_b[i] -=
                Math::SpatialForce(bodyFrame, v_body_wrt_fluid.cwiseProduct(dragData.linearDrag + dragData.quadraticDrag.cwiseProduct(v_body_wrt_fluid.cwiseAbs())));
        }
        else
        {
            model.f_b[i].setIncludingFrame(bodyFrame, 0., 0., 0., 0., 0., 0.);
        }
    }

    if (f_ext != nullptr)
    {
        for (unsigned int i = 1; i < model.mBodies.size(); i++)
        {
            model.f_b[i] -= (*f_ext)[i].changeFrameAndCopy(model.bodyFrames[i]);
        }
    }

    // From the body forces, compute the joint torques/forces
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

void buoyancyEffects(Model& model, Math::VectorNd& Tau)
{
    model.a_f.setAngularPart(Math::Vector3dZero);

    Math::MotionVector a_f_g = model.a_f - model.gravity;
    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        ReferenceFramePtr centerOfBuoyancyFrame = model.bodyCenterOfBuoyancyFrames[i];
        if (!model.mBodies[i].mIsVirtual)
        {
            Math::SpatialForce fb(centerOfBuoyancyFrame,
                                  -model.mBodies[i].volume * model.fluidDensity * a_f_g.transform_copy(centerOfBuoyancyFrame->getInverseTransformToRoot()));
            // Buoyancy bias force
            model.f_b[i] = fb.changeFrameAndCopy(model.bodyFrames[i]);
        }
        else
        {
            model.f_b[i].setIncludingFrame(model.bodyFrames[i], 0., 0., 0., 0., 0., 0.);
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

void potentialEffects(Model& model, Math::VectorNd& Tau)
{
    model.a_f.setAngularPart(Math::Vector3dZero);

    Math::MotionVector a_f_g = model.a_f - model.gravity;
    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        ReferenceFramePtr centerOfBuoyancyFrame = model.bodyCenterOfBuoyancyFrames[i];
        ReferenceFramePtr bodyFrame = model.bodyFrames[i];
        if (!model.mBodies[i].mIsVirtual)
        {
            Math::SpatialForce fb(centerOfBuoyancyFrame,
                                  -model.mBodies[i].volume * model.fluidDensity * a_f_g.transform_copy(centerOfBuoyancyFrame->getInverseTransformToRoot()));
            // Buoyancy bias force
            model.f_b[i] = fb.changeFrameAndCopy(bodyFrame) -
                           Math::SpatialForce(bodyFrame, model.I[i] * model.gravity.transform_copy(model.worldFrame->getTransformToDesiredFrame(bodyFrame)));
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

void dragEffects(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, Math::VectorNd& Tau, bool update_kinematics)
{
    model.v[0].setZero();
    model.v_f.setAngularPart(Math::Vector3dZero);

    Math::MotionVector v_body_wrt_fluid;
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
        }
        else
        {
            if (update_kinematics)
            {
                jcalc(model, i, Q, QDot);
                model.v[i].set(model.v[model.lambda[i]].transform_copy(model.bodyFrames[i]->getTransformFromParent()) + model.v_J[i]);
            }
        }

        if (!model.mBodies[i].mIsVirtual)
        {
            // Compute body velocity w.r.t. surrounding fluid. Assuming the fluid is not rotating
            v_body_wrt_fluid = Math::MotionVector(model.v[i] - model.v_f.transform_copy(bodyFrame->getInverseTransformToRoot()));
            // Compute bias forces from pressure drag
            DragData dragData = model.mBodies[i].dragData;
            model.f_b[i].setIncludingFrame(bodyFrame,
                                           -v_body_wrt_fluid.cwiseProduct(dragData.linearDrag + dragData.quadraticDrag.cwiseProduct(v_body_wrt_fluid.cwiseAbs())));
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
}  // namespace RobotDynamics
