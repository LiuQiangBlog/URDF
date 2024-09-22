/*
 * Original Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 *
 * RDL - Robot Dynamics Library
 * Modifications Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include "rdl_dynamics/rdl_hydro_utils.hpp"
#include "rdl_dynamics/Kinematics.hpp"

#include <iomanip>

namespace RobotDynamics
{
namespace HydroUtils
{
using namespace std;
using namespace Math;

void calcCenterOfBuoyancy(Model& model, const Math::VectorNd& q, Math::FramePoint& cob, bool update_kinematics)
{
    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, nullptr, nullptr);
    }

    Vector3d p_cob(0., 0., 0.);
    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        p_cob += model.mBodies[i].volume * model.bodyCenterOfBuoyancyFrames[i]->getInverseTransformToRoot().r;
    }

    cob.setIncludingFrame(p_cob / model.volume, model.worldFrame);
}

void calcCenterOfMassAndCenterOfBuoyancy(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::Vector3d& com, Math::Vector3d& cob,
                                         Math::Vector3d& com_velocity, bool update_kinematics)
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
    com.setZero();
    cob.setZero();
    for (size_t i = model.mBodies.size() - 1; i > 0; i--)
    {
        unsigned int lambda = model.lambda[i];
        com += model.mBodies[i].mMass * model.bodyCenteredFrames[i]->getInverseTransformToRoot().r;
        cob += model.mBodies[i].volume * model.bodyCenterOfBuoyancyFrames[i]->getInverseTransformToRoot().r;

        if (lambda != 0)
        {
            model.hc[lambda] = model.hc[lambda] + model.hc[i].transform_copy(model.bodyFrames[i]->getTransformToParent());
        }
        else
        {
            htot = htot + model.hc[i].transform_copy(model.bodyFrames[i]->getTransformToParent());
        }
    }

    com /= model.mass;
    cob /= model.volume;
    com_velocity = htot.tail(3) / model.mass;
}

void updateCenterOfBuoyancyFrame(Model& model, const Math::Vector3d& p_cob)
{
    model.cobFrame->setTransformFromParent(Math::SpatialTransform(p_cob));
}

void updateCenterOfBuoyancyFrame(Model& model, const Math::Vector3d& p_cob, const Math::Quaternion& orientation)
{
    model.cobFrame->setTransformFromParent(Math::SpatialTransform(orientation) * Math::SpatialTransform(p_cob));
}

void updateCenterOfBuoyancyFrame(Model& model, const Math::Vector3d& p_cob, const Math::Vector3d& euler_ypr)
{
    model.cobFrame->setTransformFromParent(Math::XeulerZYX(euler_ypr) * Math::SpatialTransform(p_cob));
}

RobotDynamics::Math::FramePoint updateCenterOfBuoyancyFrame(Model& model, const Math::VectorNd& q, bool update_kinematics)
{
    RobotDynamics::Math::FramePoint cob;
    calcCenterOfBuoyancy(model, q, cob, update_kinematics);
    updateCenterOfBuoyancyFrame(model, cob.vec());
    return cob;
}

RobotDynamics::Math::FramePoint updateCenterOfBuoyancyFrame(Model& model, const Math::VectorNd& q, const Math::Quaternion& orientation, bool update_kinematics)
{
    RobotDynamics::Math::FramePoint cob;
    calcCenterOfBuoyancy(model, q, cob, update_kinematics);
    updateCenterOfBuoyancyFrame(model, cob.vec(), orientation);
    return cob;
}

RobotDynamics::Math::FramePoint updateCenterOfBuoyancyFrame(Model& model, const Math::VectorNd& q, const Math::Vector3d& euler_ypr, bool update_kinematics)
{
    RobotDynamics::Math::FramePoint cob;
    calcCenterOfBuoyancy(model, q, cob, update_kinematics);
    updateCenterOfBuoyancyFrame(model, cob.vec(), euler_ypr);
    return cob;
}

Math::SpatialForce calcBuoyancyWrenchOnCenterOfMass(Model& model, const Math::VectorNd& q, bool update_kinematics)
{
    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, nullptr, nullptr);
    }

    Math::ForceVector f;
    Math::MotionVector a_f_g = model.a_f - model.gravity;
    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        if (!model.mBodies[i].mIsVirtual)
        {
            RobotDynamics::ReferenceFramePtr body_cob_frame = model.bodyCenterOfBuoyancyFrames[i];
            // Gravity wrench on body in world frame
            Math::ForceVector fg(model.mBodies[i].volume * model.fluidDensity * a_f_g.transform_copy(model.worldFrame->getTransformToDesiredFrame(body_cob_frame)));
            // transform wrench to com frame
            fg.transform(body_cob_frame->getTransformToDesiredFrame(model.comFrame));
            f += fg;
        }
    }

    return Math::SpatialForce(model.comFrame, f);
}

Math::SpatialForce calcDragWrenchOnCenterOfMass(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, bool update_kinematics)
{
    if (update_kinematics)
    {
        updateKinematicsCustom(model, &q, &qdot, nullptr);
    }

    Math::ForceVector f;
    Math::MotionVector v_body_wrt_fluid;
    for (unsigned int i = 1; i < model.mBodies.size(); i++)
    {
        if (!model.mBodies[i].mIsVirtual)
        {
            RobotDynamics::ReferenceFramePtr bodyFrame = model.bodyFrames[i];
            DragData dragData = model.mBodies[i].dragData;
            // Drag wrench in body frame
            v_body_wrt_fluid = Math::MotionVector(model.v[i] - model.v_f.transform_copy(bodyFrame->getInverseTransformToRoot()));

            ForceVector f_body = v_body_wrt_fluid.cwiseProduct(dragData.linearDrag + dragData.quadraticDrag.cwiseProduct(v_body_wrt_fluid.cwiseAbs()));
            f_body.setAngularPart(Vector3dZero);
            // transform wrench to com frame
            f_body.transform(bodyFrame->getTransformToDesiredFrame(model.comFrame));
            f += f_body;
        }
    }

    return Math::SpatialForce(model.comFrame, f);
}

double calcSubtreeVolume(Model& model, const unsigned int bodyId)
{
    std::vector<unsigned int> childBodyIds = model.mu[bodyId];

    double subtreeVolume = model.mBodies[bodyId].volume;

    for (unsigned int j = 0; j < childBodyIds.size(); j++)
    {
        subtreeVolume += calcSubtreeVolume(model, childBodyIds[j]);
    }

    return subtreeVolume;
}

double calcSubtreeWeightInFluid(Model& model, const unsigned int bodyId)
{
    std::vector<unsigned int> childBodyIds = model.mu[bodyId];

    double subtreeWeight = model.mBodies[bodyId].mMass - model.fluidDensity * model.mBodies[bodyId].volume;

    for (unsigned int j = 0; j < childBodyIds.size(); j++)
    {
        subtreeWeight += calcSubtreeWeightInFluid(model, childBodyIds[j]);
    }

    return subtreeWeight;
}
}  // namespace HydroUtils
}  // namespace RobotDynamics
