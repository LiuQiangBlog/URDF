// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <assert.h>
#include <iostream>
#include <limits>
#include <thread>

#include "rdl_dynamics/rdl_mathutils.hpp"
#include "rdl_dynamics/Model.hpp"

using namespace RobotDynamics;
using namespace RobotDynamics::Math;

static std::string ROOT_BODY_NAME = "ROOT";

static std::string create_parameter_name(const rclcpp::Node::SharedPtr& node, const std::string& name)
{
    std::string generated_name = "";
    std::string subnode = node->get_sub_namespace();

    if (subnode.length())
    {
        generated_name += "/" + subnode;
    }

    generated_name += "/" + name;

    generated_name.erase(0, std::min(generated_name.find_first_not_of('/'), generated_name.size() - 1));
    return generated_name;
}

Model::Model(const rclcpp::Node::SharedPtr& n, unsigned int* n_threads) : work(new boost::asio::io_service::work(io_service)), node(n)
{
    worldFrame = ReferenceFrame::getWorldFrame();
    referenceFrameMap[worldFrame->getName()] = worldFrame;
    comFrame.reset(new FixedReferenceFrame("robot_com", worldFrame, Math::SpatialTransform(), 0));
    cobFrame.reset(new FixedReferenceFrame("robot_cob", worldFrame, Math::SpatialTransform(), 0));

    Body root_body;
    Joint root_joint;

    mass = 0.;

    // structural information
    lambda.push_back(0);
    lambda_q.push_back(0);
    mu.push_back(std::vector<unsigned int>());
    dof_count = 0;
    q_size = 0;
    qdot_size = 0;
    previously_added_body_id = 0;

    std::vector<unsigned int> id_chain;
    id_chain.push_back(0);
    lambda_chain.push_back(id_chain);
    if (!n)
    {
        fluidDensity = 1000.;  // Pure water
        gravity = SpatialVector(0., 0., 0., 0., -9.81, 0.);
    }
    else
    {
        rcl_interfaces::msg::ParameterDescriptor gravity_vector_description;
        gravity_vector_description.name = create_parameter_name(node, "gravity_vector");
        gravity_vector_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
        gravity_vector_description.description = "[x, y, z] gravity vector, defaults to [0., -9.81, 0.]";
        gravity_vector_description.read_only = false;
        std::vector<double> gravity_v = n->declare_parameter(gravity_vector_description.name, std::vector<double>({ 0., -9.81, 0. }), gravity_vector_description);
        if (gravity_v.size() != 3)
        {
            throw std::runtime_error("Model: gravity vector parameter must have exactly 3 elements, provided parameter has " + std::to_string(gravity_v.size()));
        }

        gravity = SpatialVector(0., 0., 0., gravity_v[0], gravity_v[1], gravity_v[2]);

        rcl_interfaces::msg::ParameterDescriptor fluid_density_description;
        fluid_density_description.name = create_parameter_name(node, "fluid_density");
        fluid_density_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        fluid_density_description.description = "fluid density to be used in hydrodynamics calculations, defaults to 1000kg/m^3(pure water)";
        fluid_density_description.floating_point_range.resize(1);
        fluid_density_description.floating_point_range[0].from_value = 0.;
        fluid_density_description.floating_point_range[0].to_value = 1.e10;
        fluid_density_description.floating_point_range[0].step = 0.;
        fluid_density_description.read_only = false;
        fluidDensity = n->declare_parameter(fluid_density_description.name, 1000., fluid_density_description);
    }

    /**
     * Initialize fluid velocity/accel to zero
     */
    a_f = MotionVector(0., 0., 0., 0., 0., 0.);
    v_f = MotionVector(0., 0., 0., 0., 0., 0.);

    // state information
    SpatialMotion rootBodySpatialVelocity(worldFrame, worldFrame, worldFrame, SpatialVectorZero);
    SpatialAcceleration rootBodySpatialAcceleration(worldFrame, worldFrame, worldFrame, SpatialVectorZero);

    v.push_back(rootBodySpatialVelocity);
    a.push_back(rootBodySpatialAcceleration);

    // Joints
    mJoints.push_back(root_joint);
    S.push_back(MotionVector(SpatialVectorZero));
    S_o.push_back(MotionVector(SpatialVectorZero));
    X_T.push_back(SpatialTransform());

    X_J.push_back(SpatialTransform());
    v_J.push_back(SpatialMotion(worldFrame, worldFrame, worldFrame, SpatialVectorZero));
    c_J.push_back(SpatialVectorZero);

    // Spherical joints
    multdof3_S.push_back(Matrix63::Zero());
    multdof3_S_o.push_back(Matrix63::Zero());
    multdof3_U.push_back(Matrix63::Zero());
    multdof3_Dinv.push_back(Matrix3d::Zero());
    multdof3_u.push_back(Vector3d::Zero());
    multdof3_w_index.push_back(0);

    // Dynamic variables
    c.push_back(SpatialVectorZero);
    IA.push_back(SpatialMatrixIdentity);
    pA.push_back(RobotDynamics::Math::SpatialForce(worldFrame, 0., 0., 0., 0., 0., 0.));
    U.push_back(SpatialVectorZero);

    u = VectorNd::Zero(1);
    d = VectorNd::Zero(1);

    f.push_back(RobotDynamics::Math::SpatialForce(worldFrame, 0., 0., 0., 0., 0., 0.));
    f_b.push_back(RobotDynamics::Math::SpatialForce(worldFrame, 0., 0., 0., 0., 0., 0.));
    hc.push_back(RobotDynamics::Math::Momentum());

    // Bodies
    X_lambda.push_back(SpatialTransform());

    bodyFrames.push_back(worldFrame);          // 0th body is world
    bodyCenteredFrames.push_back(worldFrame);  // 0th body centered frame is world
    I.push_back(SpatialInertia(worldFrame));
    Ib_c.push_back(SpatialInertia(worldFrame));
    Ic.push_back(RigidBodyInertia());

    bodyCenterOfBuoyancyFrames.push_back(worldFrame);  // 0th body centered frame is world
    I_add.push_back(Math::SpatialMatrixZero);
    I_H.push_back(Math::SpatialMatrixZero);
    Ic_H.push_back(SpatialMatrixZero);

    mBodies.push_back(root_body);
    mBodyNameMap[ROOT_BODY_NAME] = 0;

    fixed_body_discriminator = std::numeric_limits<unsigned int>::max() / 2;

    unsigned int num_threads = n_threads == nullptr ? std::thread::hardware_concurrency() : *n_threads;
    for (unsigned int i = 0; i < num_threads; ++i)
    {
        threads.push_back(thread_pool.create_thread(boost::bind(&boost::asio::io_service::run, &io_service)));
    }
}

unsigned int addBodyFixedJoint(Model& model, const unsigned int parent_id, const SpatialTransform& joint_frame, const Joint& joint, const Body& body,
                               std::string body_name)
{
    (void)joint;
    FixedBody fbody = FixedBody::CreateFromBody(body);
    model.mass += fbody.mMass;
    model.volume += fbody.volume;
    fbody.mMovableParent = parent_id;
    fbody.mParentTransform = joint_frame;

    if (model.IsFixedBodyId(parent_id))
    {
        FixedBody fixed_parent = model.mFixedBodies.at(parent_id - model.fixed_body_discriminator);

        fbody.mMovableParent = fixed_parent.mMovableParent;
        fbody.mParentTransform = joint_frame * fixed_parent.mParentTransform;
    }

    // merge the two bodies
    Body parent_body = model.mBodies.at(fbody.mMovableParent);
    parent_body.join(fbody.mParentTransform, body);
    model.mBodies.at(fbody.mMovableParent) = parent_body;

    model.I.at(fbody.mMovableParent).set(createFromMassComInertiaC(parent_body.mMass, parent_body.mCenterOfMass, parent_body.mInertia));
    model.Ib_c.at(fbody.mMovableParent).set(createFromMassComInertiaC(parent_body.mMass, Vector3dZero, parent_body.mInertia));

    model.I_H.at(fbody.mMovableParent) = (model.I.at(fbody.mMovableParent).toMatrix() + model.I_add.at(fbody.mMovableParent));
    model.Ic_H.at(fbody.mMovableParent) = model.I_H.at(fbody.mMovableParent);

    // Since the bodies have been combined and there is a new COM, need to update the body's COM frame xform to parent.
    model.bodyCenteredFrames.at(fbody.mMovableParent)->setTransformFromParent(Xtrans(parent_body.mCenterOfMass));
    model.bodyCenterOfBuoyancyFrames.at(fbody.mMovableParent)->setTransformFromParent(Xtrans(parent_body.mCenterOfBuoyancy));

    model.mFixedBodies.push_back(fbody);

    if (model.mFixedBodies.size() > std::numeric_limits<unsigned int>::max() - model.fixed_body_discriminator)
    {
        std::string msg = "Error: cannot add more than " + std::to_string(std::numeric_limits<unsigned int>::max() - model.mFixedBodies.size()) +
                          " fixed bodies. You need to modify Model::fixed_body_discriminator for this.";
        throw RdlException(msg);
    }

    ReferenceFramePtr fixedBodyFrame;
    fixedBodyFrame.reset(new FixedReferenceFrame(body_name, model.bodyFrames.at(fbody.mMovableParent), fbody.mParentTransform, fbody.mMovableParent));
    model.fixedBodyFrames.push_back(fixedBodyFrame);

    if (body_name.size() != 0)
    {
        if (model.mBodyNameMap.find(body_name) != model.mBodyNameMap.end())
        {
            std::string msg = "Error: Fixed body with name '" + body_name + "' already exists!";
            throw RdlException(msg);
        }
        model.mBodyNameMap[body_name] = model.mFixedBodies.size() + model.fixed_body_discriminator - 1;
        model.referenceFrameMap[body_name] = fixedBodyFrame;
    }

    return model.mFixedBodies.size() + model.fixed_body_discriminator - 1;
}

unsigned int addBodyMultiDofJoint(Model& model, const unsigned int parent_id, const SpatialTransform& joint_frame, const Joint& joint, const Body& body,
                                  std::string body_name)
{
    // Here we emulate multi DoF joints by simply adding nullbodies. This
    // allows us to use fixed size elements for S,v,a, etc. which is very
    // fast in Eigen.
    unsigned int joint_count = 0;
    if (joint.mJointType == JointType1DoF)
    {
        joint_count = 1;
    }
    else if (joint.mJointType == JointType2DoF)
    {
        joint_count = 2;
    }
    else if (joint.mJointType == JointType3DoF)
    {
        joint_count = 3;
    }
    else if (joint.mJointType == JointType4DoF)
    {
        joint_count = 4;
    }
    else if (joint.mJointType == JointType5DoF)
    {
        joint_count = 5;
    }
    else if (joint.mJointType == JointType6DoF)
    {
        joint_count = 6;
    }
    else if (joint.mJointType == JointTypeFloatingBase)
    // no action required
    {
    }
    else
    {
        throw std::runtime_error("Invalid joint type " + joint.mJointType);
    }

    Body null_body(0., Vector3d(0., 0., 0.), Vector3d(0., 0., 0.));
    null_body.mIsVirtual = true;

    unsigned int null_parent = parent_id;
    SpatialTransform joint_frame_transform;

    if (joint.mJointType == JointTypeFloatingBase)
    {
        null_parent = model.addBody(parent_id, joint_frame, Joint(JointTypeTranslationXYZ), null_body);

        return model.addBody(null_parent, SpatialTransform(), Joint(JointTypeSpherical), body, body_name);
    }

    Joint single_dof_joint;
    unsigned int j;

    // Here we add multiple virtual bodies that have no mass or inertia for
    // which each is attached to the model with a single degree of freedom
    // joint.
    for (j = 0; j < joint_count; j++)
    {
        single_dof_joint = Joint(joint.mJointAxes[j]);
        if (single_dof_joint.mJointType == JointType1DoF)
        {
            Vector3d rotation(joint.mJointAxes[j][0], joint.mJointAxes[j][1], joint.mJointAxes[j][2]);
            Vector3d translation(joint.mJointAxes[j][3], joint.mJointAxes[j][4], joint.mJointAxes[j][5]);

            if (rotation == Vector3d(0., 0., 0.))
            {
                single_dof_joint = Joint(JointTypePrismatic, translation);
            }
            else if (translation == Vector3d(0., 0., 0.))
            {
                single_dof_joint = Joint(JointTypeRevolute, rotation);
            }
            else
            {
                throw std::runtime_error("Invalid joint axis for body " + body_name + ". Helical joints not (yet) supported.");
            }
        }

        // the first joint has to be transformed by joint_frame, all the
        // others must have a null transformation
        if (j == 0)
        {
            joint_frame_transform = joint_frame;
        }
        else
        {
            joint_frame_transform = SpatialTransform();
        }

        if (j == joint_count - 1)
        {
            // if we are at the last we must add the real body
            break;
        }
        else
        {
            // otherwise we just add an intermediate body
            null_parent = model.addBody(null_parent, joint_frame_transform, single_dof_joint, null_body);
        }
    }

    return model.addBody(null_parent, joint_frame_transform, single_dof_joint, body, body_name);
}

unsigned int Model::addBody(const unsigned int parent_id, const SpatialTransform& joint_frame, const Joint& joint, const Body& body, std::string body_name)
{
    assert(lambda.size() > 0);
    assert(joint.mJointType != JointTypeUndefined);

    if (joint.mJointType == JointTypeFixed)
    {
        previously_added_body_id = addBodyFixedJoint(*this, parent_id, joint_frame, joint, body, body_name);

        return previously_added_body_id;
    }
    else if ((joint.mJointType == JointTypeSpherical) || (joint.mJointType == JointTypeEulerZYX) || (joint.mJointType == JointTypeEulerXYZ) ||
             (joint.mJointType == JointTypeEulerYXZ) || (joint.mJointType == JointTypeTranslationXYZ) || (joint.mJointType == JointTypeCustom))
    {
        // no action required
    }
    else if (joint.mJointType != JointTypePrismatic && joint.mJointType != JointTypeRevolute && joint.mJointType != JointTypeRevoluteX &&
             joint.mJointType != JointTypeRevoluteY && joint.mJointType != JointTypeRevoluteZ)
    {
        previously_added_body_id = addBodyMultiDofJoint(*this, parent_id, joint_frame, joint, body, body_name);
        return previously_added_body_id;
    }

    // If we add the body to a fixed body we have to make sure that we
    // actually add it to its movable parent.
    unsigned int movable_parent_id = parent_id;
    SpatialTransform movable_parent_transform;

    if (IsFixedBodyId(parent_id))
    {
        unsigned int fbody_id = parent_id - fixed_body_discriminator;
        movable_parent_id = mFixedBodies.at(fbody_id).mMovableParent;
        movable_parent_transform = mFixedBodies.at(fbody_id).mParentTransform;
    }

    // structural information
    lambda.push_back(movable_parent_id);
    unsigned int lambda_q_last = mJoints.back().q_index;

    if (mJoints[mJoints.size() - 1].mDoFCount > 0 && mJoints[mJoints.size() - 1].mJointType != JointTypeCustom)
    {
        lambda_q_last = lambda_q_last + mJoints.back().mDoFCount;
    }
    else if (mJoints[mJoints.size() - 1].mJointType == JointTypeCustom)
    {
        lambda_q_last = lambda_q_last + mCustomJoints.back()->mDoFCount;
    }

    for (unsigned int i = 0; i < joint.mDoFCount; i++)
    {
        lambda_q.push_back(lambda_q_last + i);
    }
    mu.push_back(std::vector<unsigned int>());
    mu.at(movable_parent_id).push_back(mBodies.size());

    num_branch_ends = 0;
    for (unsigned int i = 0; i < mu.size(); ++i)
    {
        if (mu[i].size() == 0)
        {
            num_branch_ends++;
        }
    }

    // Bodies
    X_lambda.push_back(joint_frame);

    ReferenceFramePtr bodyFrame;
    ReferenceFramePtr parentBodyFrame;
    if (!movable_parent_id)
    {
        parentBodyFrame = ReferenceFrame::getWorldFrame();
        // movable_parent_id = 0 so this is the root body, and it's parent frame is world frame
        bodyFrame.reset(new ReferenceFrame(body_name, parentBodyFrame, X_lambda.back(), true, mBodies.size()));
    }
    else
    {
        parentBodyFrame = bodyFrames.at(movable_parent_id);
        bodyFrame.reset(new ReferenceFrame(body_name, parentBodyFrame, joint_frame * movable_parent_transform, true, mBodies.size()));
    }

    bodyFrames.push_back(bodyFrame);
    referenceFrameMap[body_name] = bodyFrame;

    mBodies.push_back(body);
    nbodies0_vec.conservativeResize(mBodies.size());
    nbodies0_vec.setZero();

    if (!body_name.empty())
    {
        if (mBodyNameMap.find(body_name) != mBodyNameMap.end())
        {
            std::string msg = "Error: Body with name '" + body_name + "' already exists!";
            throw RdlException(msg);
        }
        mBodyNameMap[body_name] = mBodies.size() - 1;
    }

    // state information
    // Body spatial velocity. Expressed in body frame
    v.push_back(SpatialMotion(bodyFrame, ReferenceFrame::getWorldFrame(), bodyFrame, SpatialVectorZero));
    a.push_back(SpatialAcceleration(bodyFrame, ReferenceFrame::getWorldFrame(), bodyFrame, SpatialVectorZero));

    // Joints
    unsigned int prev_joint_index = mJoints.size() - 1;
    mJoints.push_back(joint);

    mJoints.back().q_index = mJoints.at(prev_joint_index).q_index + mJoints.at(prev_joint_index).mDoFCount;

    S.push_back(MotionVector(joint.mJointAxes[0]));
    S_o.push_back(MotionVector(SpatialVectorZero));
    // Joint state variables
    X_J.push_back(SpatialTransform());
    c_J.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));

    v_J.push_back(SpatialMotion(bodyFrame, parentBodyFrame, bodyFrame, joint.mJointAxes[0]));

    // workspace for joints with 3 dof
    multdof3_S.push_back(Matrix63::Zero(6, 3));
    multdof3_S_o.push_back(Matrix63::Zero(6, 3));
    multdof3_U.push_back(Matrix63::Zero());
    multdof3_Dinv.push_back(Matrix3d::Zero());
    multdof3_u.push_back(Vector3d::Zero());
    multdof3_w_index.push_back(0);

    dof_count = dof_count + joint.mDoFCount;

    ndof0_mat.conservativeResize(dof_count, dof_count);
    ndof0_mat.setZero();

    ndof0_vec.conservativeResize(dof_count);
    ndof0_vec.setZero();

    // update the w components of the Quaternions. They are stored at the end
    // of the q vector
    int multdof3_joint_counter = 0;
    for (unsigned int i = 1; i < mJoints.size(); i++)
    {
        if (mJoints.at(i).mJointType == JointTypeSpherical)
        {
            multdof3_w_index.at(i) = dof_count + multdof3_joint_counter;
            multdof3_joint_counter++;
        }
    }

    q_size = dof_count + multdof3_joint_counter;
    q0_vec.conservativeResize(q_size);
    q0_vec.setZero();

    qdot_size = qdot_size + joint.mDoFCount;

    three_x_qd0.conservativeResize(3, qdot_size);
    three_x_qd0.setZero();

    // we have to invert the transformation as it is later always used from the
    // child bodies perspective.
    X_T.push_back(joint_frame * movable_parent_transform);

    // Dynamic variables
    c.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
    IA.push_back(SpatialMatrix::Zero(6, 6));
    pA.push_back(RobotDynamics::Math::SpatialForce(bodyFrame, 0., 0., 0., 0., 0., 0.));
    U.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));

    d = VectorNd::Zero(mBodies.size());
    u = VectorNd::Zero(mBodies.size());

    f.push_back(RobotDynamics::Math::SpatialForce(bodyFrame, 0., 0., 0., 0., 0., 0.));
    f_b.push_back(RobotDynamics::Math::SpatialForce(bodyFrame, 0., 0., 0., 0., 0., 0.));

    ReferenceFramePtr centerOfMassFrame;
    centerOfMassFrame.reset(new FixedReferenceFrame(body_name + "_com", bodyFrames.back(), Xtrans(body.mCenterOfMass), bodyFrame->getMovableBodyId()));

    bodyCenteredFrames.push_back(centerOfMassFrame);

    if (!validateRigidBodyInertia(body))
    {
        if (!node)
        {
            std::cerr << "\033[1;31m Body " << body_name << " failed inertia validation\033[0m" << std::endl;
        }
        else
        {
            RCLCPP_ERROR_STREAM(node->get_logger(), "Body " << body_name << " failed inertia validation");
        }
    }

    ReferenceFramePtr centerOfBuoyancyFrame(
        new FixedReferenceFrame(body_name + "_cob", bodyFrames.back(), Xtrans(body.mCenterOfBuoyancy), bodyFrame->getMovableBodyId()));
    bodyCenterOfBuoyancyFrames.push_back(centerOfBuoyancyFrame);

    SpatialInertia bodyInertia(bodyFrames.back(), createFromMassComInertiaC(body.mMass, body.mCenterOfMass, body.mInertia));
    SpatialInertia bodyCenteredInertia(centerOfMassFrame, createFromMassComInertiaC(body.mMass, Vector3dZero, body.mInertia));

    I.push_back(bodyInertia);
    Ib_c.push_back(bodyCenteredInertia);
    Ic.push_back(bodyInertia);

    /**
     * Hydrodynamics
     *
     * Added mass matrix I_add contains the added mass matrix plus the bodies rigid body inertia matrix
     */
    Math::SpatialMatrix added_mass_matrix = body.addedMassMatrix;
    I_add.push_back(added_mass_matrix);
    I_H.push_back(bodyInertia.toMatrix() + added_mass_matrix);
    Ic_H.push_back(bodyInertia.toMatrix() + added_mass_matrix);

    hc.push_back(RobotDynamics::Math::Momentum());

    if (mBodies.size() == fixed_body_discriminator)
    {
        throw std::runtime_error("Error: cannot add more than " + std::to_string(fixed_body_discriminator) +
                                 " movable bodies. You need to modify "
                                 "Model::fixed_body_discriminator for this.");
    }

    previously_added_body_id = mBodies.size() - 1;

    mJointUpdateOrder.clear();

    // update the joint order computation
    std::vector<std::pair<JointType, unsigned int>> joint_types;
    for (unsigned int i = 0; i < mJoints.size(); i++)
    {
        joint_types.push_back(std::pair<JointType, unsigned int>(mJoints[i].mJointType, i));
        mJointUpdateOrder.push_back(mJoints[i].mJointType);
    }

    mJointUpdateOrder.clear();
    JointType current_joint_type = JointTypeUndefined;
    while (joint_types.size() != 0)
    {
        current_joint_type = joint_types[0].first;

        std::vector<std::pair<JointType, unsigned int>>::iterator type_iter = joint_types.begin();

        while (type_iter != joint_types.end())
        {
            if (type_iter->first == current_joint_type)
            {
                mJointUpdateOrder.push_back(type_iter->second);
                type_iter = joint_types.erase(type_iter);
            }
            else
            {
                ++type_iter;
            }
        }
    }

    std::vector<unsigned int> parent_chain(lambda_chain.at(movable_parent_id).size() + 1);
    for (unsigned int idx = 0; idx < lambda_chain.at(movable_parent_id).size(); idx++)
    {
        parent_chain[idx] = lambda_chain.at(movable_parent_id).at(idx);
    }

    parent_chain.back() = previously_added_body_id;  // add this joint
    lambda_chain.push_back(parent_chain);

    mass += body.mMass;
    volume += body.volume;
    return previously_added_body_id;
}

unsigned int Model::appendBody(const Math::SpatialTransform& joint_frame, const Joint& joint, const Body& body, std::string body_name)
{
    return Model::addBody(previously_added_body_id, joint_frame, joint, body, body_name);
}

unsigned int Model::addBodyCustomJoint(const unsigned int parent_id, const Math::SpatialTransform& joint_frame, CustomJoint* custom_joint, const Body& body,
                                       std::string body_name)
{
    custom_joint->ndof0_vec = VectorNd::Zero(custom_joint->mDoFCount);
    Joint proxy_joint(JointTypeCustom, custom_joint->mDoFCount);
    proxy_joint.custom_joint_index = mCustomJoints.size();

    // proxy_joint.mDoFCount = custom_joint->mDoFCount; //MM added. Otherwise
    // model.q_size = 0, which is not good.

    mCustomJoints.push_back(custom_joint);

    unsigned int body_id = addBody(parent_id, joint_frame, proxy_joint, body, body_name);

    return body_id;
}

void Model::setupRosParameters()
{
    if (!node)
    {
        return;
    }
    for (const std::pair<std::string, unsigned int> p : mBodyNameMap)
    {
        if (p.first == ROOT_BODY_NAME || IsFixedBodyId(p.second))
        {
            // root body is world, don't need to give access its params, they are inconsequential
            // skip fixed bodies since their info is lumped in to their parent body
            continue;
        }

        rcl_interfaces::msg::ParameterDescriptor mass_description;
        mass_description.name = create_parameter_name(node, p.first + "_mass");
        mass_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        mass_description.description = "mass for link " + p.first;
        mass_description.floating_point_range.resize(1);
        mass_description.floating_point_range[0].from_value = 0.;
        mass_description.floating_point_range[0].to_value = 1.e10;
        mass_description.floating_point_range[0].step = 0.;
        mass_description.read_only = false;
        double link_mass = node->declare_parameter(mass_description.name, mBodies[p.second].mMass, mass_description);

        rcl_interfaces::msg::ParameterDescriptor volume_description;
        volume_description.name = create_parameter_name(node, p.first + "_volume");
        volume_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        volume_description.description = "volume for link " + p.first;
        volume_description.floating_point_range.resize(1);
        volume_description.floating_point_range[0].from_value = 0.;
        volume_description.floating_point_range[0].to_value = 1.e10;
        volume_description.floating_point_range[0].step = 0.;
        volume_description.read_only = false;
        double link_volume = node->declare_parameter(volume_description.name, mBodies[p.second].volume, volume_description);

        rcl_interfaces::msg::ParameterDescriptor comx_description;
        comx_description.name = create_parameter_name(node, p.first + "_comx");
        comx_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        comx_description.description = "x component of center of mass for link " + p.first;
        comx_description.floating_point_range.resize(1);
        comx_description.floating_point_range[0].from_value = -1.e5;
        comx_description.floating_point_range[0].to_value = 1.e5;
        comx_description.floating_point_range[0].step = 0.;
        comx_description.read_only = false;
        double comx = node->declare_parameter(comx_description.name, mBodies[p.second].mCenterOfMass.x(), comx_description);

        rcl_interfaces::msg::ParameterDescriptor comy_description;
        comy_description.name = create_parameter_name(node, p.first + "_comy");
        comy_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        comy_description.description = "y component of center of mass for link " + p.first;
        comy_description.floating_point_range.resize(1);
        comy_description.floating_point_range[0].from_value = -1.e5;
        comy_description.floating_point_range[0].to_value = 1.e5;
        comy_description.floating_point_range[0].step = 0.;
        comy_description.read_only = false;
        double comy = node->declare_parameter(comy_description.name, mBodies[p.second].mCenterOfMass.y(), comy_description);

        rcl_interfaces::msg::ParameterDescriptor comz_description;
        comz_description.name = create_parameter_name(node, p.first + "_comz");
        comz_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        comz_description.description = "z component of center of mass for link " + p.first;
        comz_description.floating_point_range.resize(1);
        comz_description.floating_point_range[0].from_value = -1.e5;
        comz_description.floating_point_range[0].to_value = 1.e5;
        comz_description.floating_point_range[0].step = 0.;
        comz_description.read_only = false;
        double comz = node->declare_parameter(comz_description.name, mBodies[p.second].mCenterOfMass.z(), comz_description);

        rcl_interfaces::msg::ParameterDescriptor cobx_description;
        cobx_description.name = create_parameter_name(node, p.first + "_cobx");
        cobx_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        cobx_description.description = "x component of center of buoyancy for link " + p.first;
        cobx_description.floating_point_range.resize(1);
        cobx_description.floating_point_range[0].from_value = -1.e5;
        cobx_description.floating_point_range[0].to_value = 1.e5;
        cobx_description.floating_point_range[0].step = 0.;
        cobx_description.read_only = false;
        double cobx = node->declare_parameter(cobx_description.name, mBodies[p.second].mCenterOfBuoyancy.x(), cobx_description);

        rcl_interfaces::msg::ParameterDescriptor coby_description;
        coby_description.name = create_parameter_name(node, p.first + "_coby");
        coby_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        coby_description.description = "y component of center of buoyancy for link " + p.first;
        coby_description.floating_point_range.resize(1);
        coby_description.floating_point_range[0].from_value = -1.e5;
        coby_description.floating_point_range[0].to_value = 1.e5;
        coby_description.floating_point_range[0].step = 0.;
        coby_description.read_only = false;
        double coby = node->declare_parameter(coby_description.name, mBodies[p.second].mCenterOfBuoyancy.y(), coby_description);

        rcl_interfaces::msg::ParameterDescriptor cobz_description;
        cobz_description.name = create_parameter_name(node, p.first + "_cobz");
        cobz_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        cobz_description.description = "z component of center of buoyancy for link " + p.first;
        cobz_description.floating_point_range.resize(1);
        cobz_description.floating_point_range[0].from_value = -1.e5;
        cobz_description.floating_point_range[0].to_value = 1.e5;
        cobz_description.floating_point_range[0].step = 0.;
        cobz_description.read_only = false;
        double cobz = node->declare_parameter(cobz_description.name, mBodies[p.second].mCenterOfBuoyancy.z(), cobz_description);

        rcl_interfaces::msg::ParameterDescriptor ixx_description;
        ixx_description.name = create_parameter_name(node, p.first + "_ixx");
        ixx_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        ixx_description.description = "Ixx component of inertia matrix for link " + p.first;
        ixx_description.floating_point_range.resize(1);
        ixx_description.floating_point_range[0].from_value = 0.;
        ixx_description.floating_point_range[0].to_value = 1.e10;
        ixx_description.floating_point_range[0].step = 0.;
        ixx_description.read_only = false;
        double ixx = node->declare_parameter(ixx_description.name, mBodies[p.second].mInertia(0, 0), ixx_description);

        rcl_interfaces::msg::ParameterDescriptor iyy_description;
        iyy_description.name = create_parameter_name(node, p.first + "_iyy");
        iyy_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        iyy_description.description = "Iyy component of inertia matrix for link " + p.first;
        iyy_description.floating_point_range.resize(1);
        iyy_description.floating_point_range[0].from_value = 0.;
        iyy_description.floating_point_range[0].to_value = 1.e10;
        iyy_description.floating_point_range[0].step = 0.;
        iyy_description.read_only = false;
        double iyy = node->declare_parameter(iyy_description.name, mBodies[p.second].mInertia(1, 1), iyy_description);

        rcl_interfaces::msg::ParameterDescriptor izz_description;
        izz_description.name = create_parameter_name(node, p.first + "_izz");
        izz_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        izz_description.description = "Izz component of inertia matrix for link " + p.first;
        izz_description.floating_point_range.resize(1);
        izz_description.floating_point_range[0].from_value = 0.;
        izz_description.floating_point_range[0].to_value = 1.e10;
        izz_description.floating_point_range[0].step = 0.;
        izz_description.read_only = false;
        double izz = node->declare_parameter(izz_description.name, mBodies[p.second].mInertia(2, 2), izz_description);

        rcl_interfaces::msg::ParameterDescriptor ixy_description;
        ixy_description.name = create_parameter_name(node, p.first + "_ixy");
        ixy_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        ixy_description.description = "Ixy component of inertia matrix for link " + p.first;
        ixy_description.floating_point_range.resize(1);
        ixy_description.floating_point_range[0].from_value = -1.e10;
        ixy_description.floating_point_range[0].to_value = 1.e10;
        ixy_description.floating_point_range[0].step = 0.;
        ixy_description.read_only = false;
        double ixy = node->declare_parameter(ixy_description.name, mBodies[p.second].mInertia(0, 1), ixy_description);

        rcl_interfaces::msg::ParameterDescriptor ixz_description;
        ixz_description.name = create_parameter_name(node, p.first + "_ixz");
        ixz_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        ixz_description.description = "Ixz component of inertia matrix for link " + p.first;
        ixz_description.floating_point_range.resize(1);
        ixz_description.floating_point_range[0].from_value = -1.e10;
        ixz_description.floating_point_range[0].to_value = 1.e10;
        ixz_description.floating_point_range[0].step = 0.;
        ixz_description.read_only = false;
        double ixz = node->declare_parameter(ixz_description.name, mBodies[p.second].mInertia(0, 2), ixz_description);

        rcl_interfaces::msg::ParameterDescriptor iyz_description;
        iyz_description.name = create_parameter_name(node, p.first + "_iyz");
        iyz_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        iyz_description.description = "Iyz component of inertia matrix for link " + p.first;
        iyz_description.floating_point_range.resize(1);
        iyz_description.floating_point_range[0].from_value = -1.e10;
        iyz_description.floating_point_range[0].to_value = 1.e10;
        iyz_description.floating_point_range[0].step = 0.;
        iyz_description.read_only = false;
        double iyz = node->declare_parameter(iyz_description.name, mBodies[p.second].mInertia(1, 2), iyz_description);

        rcl_interfaces::msg::ParameterDescriptor drag_linear_fx_description;
        drag_linear_fx_description.name = create_parameter_name(node, p.first + "_drag_linear_fx");
        drag_linear_fx_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        drag_linear_fx_description.description = "fx component of linear drag for link " + p.first;
        drag_linear_fx_description.floating_point_range.resize(1);
        drag_linear_fx_description.floating_point_range[0].from_value = -1.e10;
        drag_linear_fx_description.floating_point_range[0].to_value = 0.;
        drag_linear_fx_description.floating_point_range[0].step = 0.;
        drag_linear_fx_description.read_only = false;
        double linear_drag_fx = node->declare_parameter(drag_linear_fx_description.name, mBodies[p.second].dragData.linearDrag[3], drag_linear_fx_description);

        rcl_interfaces::msg::ParameterDescriptor drag_linear_fy_description;
        drag_linear_fy_description.name = create_parameter_name(node, p.first + "_drag_linear_fy");
        drag_linear_fy_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        drag_linear_fy_description.description = "fy component of linear drag for link " + p.first;
        drag_linear_fy_description.floating_point_range.resize(1);
        drag_linear_fy_description.floating_point_range[0].from_value = -1.e10;
        drag_linear_fy_description.floating_point_range[0].to_value = 0.;
        drag_linear_fy_description.floating_point_range[0].step = 0.;
        drag_linear_fy_description.read_only = false;
        double linear_drag_fy = node->declare_parameter(drag_linear_fy_description.name, mBodies[p.second].dragData.linearDrag[4], drag_linear_fy_description);

        rcl_interfaces::msg::ParameterDescriptor drag_linear_fz_description;
        drag_linear_fz_description.name = create_parameter_name(node, p.first + "_drag_linear_fz");
        drag_linear_fz_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        drag_linear_fz_description.description = "fz component of linear drag for link " + p.first;
        drag_linear_fz_description.floating_point_range.resize(1);
        drag_linear_fz_description.floating_point_range[0].from_value = -1.e10;
        drag_linear_fz_description.floating_point_range[0].to_value = 0.;
        drag_linear_fz_description.floating_point_range[0].step = 0.;
        drag_linear_fz_description.read_only = false;
        double linear_drag_fz = node->declare_parameter(drag_linear_fz_description.name, mBodies[p.second].dragData.linearDrag[5], drag_linear_fz_description);

        rcl_interfaces::msg::ParameterDescriptor drag_linear_mx_description;
        drag_linear_mx_description.name = create_parameter_name(node, p.first + "_drag_linear_mx");
        drag_linear_mx_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        drag_linear_mx_description.description = "mx component of linear drag for link " + p.first;
        drag_linear_mx_description.floating_point_range.resize(1);
        drag_linear_mx_description.floating_point_range[0].from_value = -1.e10;
        drag_linear_mx_description.floating_point_range[0].to_value = 0.;
        drag_linear_mx_description.floating_point_range[0].step = 0.;
        drag_linear_mx_description.read_only = false;
        double linear_drag_mx = node->declare_parameter(drag_linear_mx_description.name, mBodies[p.second].dragData.linearDrag[0], drag_linear_mx_description);

        rcl_interfaces::msg::ParameterDescriptor drag_linear_my_description;
        drag_linear_my_description.name = create_parameter_name(node, p.first + "_drag_linear_my");
        drag_linear_my_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        drag_linear_my_description.description = "my component of linear drag for link " + p.first;
        drag_linear_my_description.floating_point_range.resize(1);
        drag_linear_my_description.floating_point_range[0].from_value = -1.e10;
        drag_linear_my_description.floating_point_range[0].to_value = 0.;
        drag_linear_my_description.floating_point_range[0].step = 0.;
        drag_linear_my_description.read_only = false;
        double linear_drag_my = node->declare_parameter(drag_linear_my_description.name, mBodies[p.second].dragData.linearDrag[1], drag_linear_my_description);

        rcl_interfaces::msg::ParameterDescriptor drag_linear_mz_description;
        drag_linear_mz_description.name = create_parameter_name(node, p.first + "_drag_linear_mz");
        drag_linear_mz_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        drag_linear_mz_description.description = "mz component of linear drag for link " + p.first;
        drag_linear_mz_description.floating_point_range.resize(1);
        drag_linear_mz_description.floating_point_range[0].from_value = -1.e10;
        drag_linear_mz_description.floating_point_range[0].to_value = 0.;
        drag_linear_mz_description.floating_point_range[0].step = 0.;
        drag_linear_mz_description.read_only = false;
        double linear_drag_mz = node->declare_parameter(drag_linear_mz_description.name, mBodies[p.second].dragData.linearDrag[2], drag_linear_mz_description);

        rcl_interfaces::msg::ParameterDescriptor drag_quadratic_fx_description;
        drag_quadratic_fx_description.name = create_parameter_name(node, p.first + "_drag_quadratic_fx");
        drag_quadratic_fx_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        drag_quadratic_fx_description.description = "fx component of quadratic drag for link " + p.first;
        drag_quadratic_fx_description.floating_point_range.resize(1);
        drag_quadratic_fx_description.floating_point_range[0].from_value = -1.e10;
        drag_quadratic_fx_description.floating_point_range[0].to_value = 0.;
        drag_quadratic_fx_description.floating_point_range[0].step = 0.;
        drag_quadratic_fx_description.read_only = false;
        double quadratic_drag_fx =
            node->declare_parameter(drag_quadratic_fx_description.name, mBodies[p.second].dragData.quadraticDrag[3], drag_quadratic_fx_description);

        rcl_interfaces::msg::ParameterDescriptor drag_quadratic_fy_description;
        drag_quadratic_fy_description.name = create_parameter_name(node, p.first + "_drag_quadratic_fy");
        drag_quadratic_fy_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        drag_quadratic_fy_description.description = "fy component of quadratic drag for link " + p.first;
        drag_quadratic_fy_description.floating_point_range.resize(1);
        drag_quadratic_fy_description.floating_point_range[0].from_value = -1.e10;
        drag_quadratic_fy_description.floating_point_range[0].to_value = 0.;
        drag_quadratic_fy_description.floating_point_range[0].step = 0.;
        drag_quadratic_fy_description.read_only = false;
        double quadratic_drag_fy =
            node->declare_parameter(drag_quadratic_fy_description.name, mBodies[p.second].dragData.quadraticDrag[4], drag_quadratic_fy_description);

        rcl_interfaces::msg::ParameterDescriptor drag_quadratic_fz_description;
        drag_quadratic_fz_description.name = create_parameter_name(node, p.first + "_drag_quadratic_fz");
        drag_quadratic_fz_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        drag_quadratic_fz_description.description = "fz component of quadratic drag for link " + p.first;
        drag_quadratic_fz_description.floating_point_range.resize(1);
        drag_quadratic_fz_description.floating_point_range[0].from_value = -1.e10;
        drag_quadratic_fz_description.floating_point_range[0].to_value = 0.;
        drag_quadratic_fz_description.floating_point_range[0].step = 0.;
        drag_quadratic_fz_description.read_only = false;
        double quadratic_drag_fz =
            node->declare_parameter(drag_quadratic_fz_description.name, mBodies[p.second].dragData.quadraticDrag[5], drag_quadratic_fz_description);

        rcl_interfaces::msg::ParameterDescriptor drag_quadratic_mx_description;
        drag_quadratic_mx_description.name = create_parameter_name(node, p.first + "_drag_quadratic_mx");
        drag_quadratic_mx_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        drag_quadratic_mx_description.description = "mx component of quadratic drag for link " + p.first;
        drag_quadratic_mx_description.floating_point_range.resize(1);
        drag_quadratic_mx_description.floating_point_range[0].from_value = -1.e10;
        drag_quadratic_mx_description.floating_point_range[0].to_value = 0.;
        drag_quadratic_mx_description.floating_point_range[0].step = 0.;
        drag_quadratic_mx_description.read_only = false;
        double quadratic_drag_mx =
            node->declare_parameter(drag_quadratic_mx_description.name, mBodies[p.second].dragData.quadraticDrag[0], drag_quadratic_mx_description);

        rcl_interfaces::msg::ParameterDescriptor drag_quadratic_my_description;
        drag_quadratic_my_description.name = create_parameter_name(node, p.first + "_drag_quadratic_my");
        drag_quadratic_my_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        drag_quadratic_my_description.description = "my component of quadratic drag for link " + p.first;
        drag_quadratic_my_description.floating_point_range.resize(1);
        drag_quadratic_my_description.floating_point_range[0].from_value = -1.e10;
        drag_quadratic_my_description.floating_point_range[0].to_value = 0.;
        drag_quadratic_my_description.floating_point_range[0].step = 0.;
        drag_quadratic_my_description.read_only = false;
        double quadratic_drag_my =
            node->declare_parameter(drag_quadratic_my_description.name, mBodies[p.second].dragData.quadraticDrag[1], drag_quadratic_my_description);

        rcl_interfaces::msg::ParameterDescriptor drag_quadratic_mz_description;
        drag_quadratic_mz_description.name = create_parameter_name(node, p.first + "_drag_quadratic_mz");
        drag_quadratic_mz_description.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
        drag_quadratic_mz_description.description = "mz component of quadratic drag for link " + p.first;
        drag_quadratic_mz_description.floating_point_range.resize(1);
        drag_quadratic_mz_description.floating_point_range[0].from_value = -1.e10;
        drag_quadratic_mz_description.floating_point_range[0].to_value = 0.;
        drag_quadratic_mz_description.floating_point_range[0].step = 0.;
        drag_quadratic_mz_description.read_only = false;
        double quadratic_drag_mz =
            node->declare_parameter(drag_quadratic_mz_description.name, mBodies[p.second].dragData.quadraticDrag[2], drag_quadratic_mz_description);

        DragData d(SpatialVector(linear_drag_mx, linear_drag_my, linear_drag_mz, linear_drag_fx, linear_drag_fy, linear_drag_fz),
                   SpatialVector(quadratic_drag_mx, quadratic_drag_my, quadratic_drag_mz, quadratic_drag_fx, quadratic_drag_fy, quadratic_drag_fz));

        Body b(link_mass, Vector3d(comx, comy, comz), Matrix3d(ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz), Vector3d(cobx, coby, cobz), link_volume,
               mBodies[p.second].addedMassMatrix, d);

        // in case the mass/volume is different here than previously, subtract off the old and add the new
        mass += (b.mMass - mBodies[p.second].mMass);
        volume += (b.volume - mBodies[p.second].volume);
        mBodies[p.second] = b;

        bodyCenteredFrames.at(p.second)->setTransformFromParent(Xtrans(b.mCenterOfMass));
        bodyCenterOfBuoyancyFrames.at(p.second)->setTransformFromParent(Xtrans(b.mCenterOfBuoyancy));

        SpatialInertia bodyInertia(bodyFrames.at(p.second), createFromMassComInertiaC(b.mMass, b.mCenterOfMass, b.mInertia));
        SpatialInertia bodyCenteredInertia(bodyCenteredFrames.at(p.second), createFromMassComInertiaC(b.mMass, Vector3dZero, b.mInertia));

        I.at(p.second) = bodyInertia;
        Ib_c.at(p.second) = bodyCenteredInertia;
        Ic.at(p.second) = bodyInertia;

        Math::SpatialMatrix added_mass_matrix = b.addedMassMatrix;
        I_add[p.second] = added_mass_matrix;
        I_H[p.second] = bodyInertia.toMatrix() + added_mass_matrix;
        Ic_H[p.second] = bodyInertia.toMatrix() + added_mass_matrix;
    }

    callback_handle = node->add_on_set_parameters_callback(std::bind(&Model::paramCb, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult Model::paramCb(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.reason = "success";

    for (const std::pair<std::string, unsigned int> p : mBodyNameMap)
    {
        if (p.first == ROOT_BODY_NAME || IsFixedBodyId(p.second))
        {
            // root body is world, don't need to give access its params, they are inconsequential
            // skip fixed bodies since their info is lumped in to their parent body
            continue;
        }

        bool param_changed = false;
        result.successful = true;
        for (const rclcpp::Parameter& param : parameters)
        {
            if (param.get_name() == create_parameter_name(node, p.first + "_mass"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_comx"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_comy"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_comz"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_volume"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_cobx"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_coby"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_cobz"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_drag_linear_fx"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_drag_linear_fy"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_drag_linear_fz"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_drag_linear_mx"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_drag_linear_my"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_drag_linear_mz"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_drag_quadratic_fx"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_drag_quadratic_fy"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_drag_quadratic_fz"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_drag_quadratic_mx"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_drag_quadratic_my"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_drag_quadratic_mz"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_ixx"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_ixy"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_ixz"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_iyz"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_iyy"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
            if (param.get_name() == create_parameter_name(node, p.first + "_izz"))
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    result.reason = "Wrong type for " + param.get_name();
                    result.successful = false;
                    continue;
                }
                else
                {
                    param_changed = true;
                }
            }
        }

        if (result.successful && param_changed)
        {
            Body b = mBodies.at(p.second);  // get the current body
            for (const rclcpp::Parameter& param : parameters)
            {
                RCLCPP_INFO_STREAM(node->get_logger(), "Model.paramCb: " << param.get_name() << " = " << param.value_to_string());
                if (param.get_name() == create_parameter_name(node, p.first + "_mass"))
                {
                    b.mMass = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_volume"))
                {
                    b.volume = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_comx"))
                {
                    b.mCenterOfMass.x() = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_comy"))
                {
                    b.mCenterOfMass.y() = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_comz"))
                {
                    b.mCenterOfMass.z() = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_cobx"))
                {
                    b.mCenterOfBuoyancy.x() = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_coby"))
                {
                    b.mCenterOfBuoyancy.y() = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_cobz"))
                {
                    b.mCenterOfBuoyancy.z() = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_drag_linear_fx"))
                {
                    b.dragData.linearDrag[3] = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_drag_linear_fy"))
                {
                    b.dragData.linearDrag[4] = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_drag_linear_fz"))
                {
                    b.dragData.linearDrag[5] = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_drag_linear_mx"))
                {
                    b.dragData.linearDrag[0] = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_drag_linear_my"))
                {
                    b.dragData.linearDrag[1] = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_drag_linear_mz"))
                {
                    b.dragData.linearDrag[2] = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_drag_quadratic_fx"))
                {
                    b.dragData.quadraticDrag[3] = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_drag_quadratic_fy"))
                {
                    b.dragData.quadraticDrag[4] = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_drag_quadratic_fz"))
                {
                    b.dragData.quadraticDrag[5] = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_drag_quadratic_mx"))
                {
                    b.dragData.quadraticDrag[0] = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_drag_quadratic_my"))
                {
                    b.dragData.quadraticDrag[1] = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_drag_quadratic_mz"))
                {
                    b.dragData.quadraticDrag[2] = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_ixx"))
                {
                    b.mInertia(0, 0) = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_ixy"))
                {
                    b.mInertia(0, 1) = param.as_double();
                    b.mInertia(1, 0) = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_ixz"))
                {
                    b.mInertia(0, 2) = param.as_double();
                    b.mInertia(2, 0) = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_iyz"))
                {
                    b.mInertia(1, 2) = param.as_double();
                    b.mInertia(2, 1) = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_iyy"))
                {
                    b.mInertia(1, 1) = param.as_double();
                    continue;
                }
                if (param.get_name() == create_parameter_name(node, p.first + "_izz"))
                {
                    b.mInertia(2, 2) = param.as_double();
                    continue;
                }
            }

            // in case the mass is different here than previously, subtract off the old mass and add the new mass
            mass += (b.mMass - mBodies[p.second].mMass);
            volume += (b.volume - mBodies[p.second].volume);
            mBodies.at(p.second) = b;

            bodyCenteredFrames[p.second]->setTransformFromParent(Xtrans(b.mCenterOfMass));
            bodyCenterOfBuoyancyFrames[p.second]->setTransformFromParent(Xtrans(b.mCenterOfBuoyancy));

            SpatialInertia bodyInertia(bodyFrames[p.second], createFromMassComInertiaC(b.mMass, b.mCenterOfMass, b.mInertia));
            SpatialInertia bodyCenteredInertia(bodyCenteredFrames[p.second], createFromMassComInertiaC(b.mMass, Vector3dZero, b.mInertia));

            I.at(p.second) = bodyInertia;
            Ib_c.at(p.second) = bodyCenteredInertia;
            Ic.at(p.second) = bodyInertia;

            Math::SpatialMatrix added_mass_matrix = b.addedMassMatrix;
            I_add.at(p.second) = added_mass_matrix;
            I_H.at(p.second) = bodyInertia.toMatrix() + added_mass_matrix;
            Ic_H.at(p.second) = bodyInertia.toMatrix() + added_mass_matrix;
        }
        else if (!result.successful)
        {
            RCLCPP_ERROR_STREAM(node->get_logger(), "Problem encountered updating parameters for " << p.first << ": " << result.reason);
            return result;
        }
    }

    bool param_changed = false;
    for (const rclcpp::Parameter& param : parameters)
    {
        if (param.get_name() == create_parameter_name(node, "gravity_vector"))
        {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY || param.as_double_array().size() != 3)
            {
                result.reason = "Wrong type for " + param.get_name() + " or wrong size of " + std::to_string(param.as_double_array().size()) + ". size must be exactly 3";
                result.successful = false;
                continue;
            }
            else
            {
                param_changed = true;
            }
        }

        if (param.get_name() == create_parameter_name(node, "fluid_density"))
        {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                result.reason = "Wrong type for " + param.get_name();
                result.successful = false;
                continue;
            }
            else
            {
                param_changed = true;
            }
        }
    }

    if (result.successful && param_changed)
    {
        for (const rclcpp::Parameter& param : parameters)
        {
            if (param.get_name() == create_parameter_name(node, "gravity_vector"))
            {
                std::vector<double> v = param.as_double_array();
                gravity = SpatialVector(0., 0., 0., v[0], v[1], v[2]);
                RCLCPP_INFO_STREAM(node->get_logger(), "Model.paramCb: Setting gravity vector (" << v[0] << ", " << v[1] << ", " << v[2] << ")");
                continue;
            }

            if (param.get_name() == create_parameter_name(node, "fluid_density"))
            {
                RCLCPP_INFO_STREAM(node->get_logger(), "Model.paramCb: Setting fluid density " << param.as_double());
                fluidDensity = param.as_double();
                continue;
            }
        }
    }

    return result;
}

void Model::crawlChainKinematics(unsigned int b_id, std::atomic<unsigned int>* branch_ends, const Math::VectorNd* Q, const Math::VectorNd* QDot,
                                 const Math::VectorNd* QDDot)
{
    if (b_id > 0)
    {
        if (Q && !QDot && !QDDot)
        {
            jcalc(*this, b_id, (*Q), q0_vec);
        }
        else if (Q && QDot && !QDDot)
        {
            jcalc(*this, b_id, *Q, *QDot);

            ReferenceFramePtr bodyFrame = bodyFrames[b_id];
            unsigned int lambda_id = lambda[b_id];

            if (lambda_id != 0)
            {
                v[b_id].set(v[lambda_id].transform_copy(bodyFrame->getTransformFromParent()) + v_J[b_id]);
                c[b_id] = c_J[b_id] + v[b_id] % v_J[b_id];
            }
            else
            {
                v[b_id].set(v_J[b_id]);
                c[b_id] = c_J[b_id] + v[b_id] % v_J[b_id];
            }
        }
        else if (Q && QDot && QDDot)
        {
            jcalc(*this, b_id, *Q, *QDot);

            ReferenceFramePtr bodyFrame = bodyFrames[b_id];
            unsigned int lambda_id = lambda[b_id];

            if (lambda_id != 0)
            {
                v[b_id].set(v[lambda_id].transform_copy(bodyFrame->getTransformFromParent()) + v_J[b_id]);
                c[b_id] = c_J[b_id] + v[b_id] % v_J[b_id];
            }
            else
            {
                v[b_id].set(v_J[b_id]);
                c[b_id] = c_J[b_id] + v[b_id] % v_J[b_id];
            }

            unsigned int q_index = mJoints[b_id].q_index;

            if (lambda_id != 0)
            {
                a[b_id].set(a[lambda_id].transform_copy(bodyFrame->getTransformFromParent()) + c[b_id]);
            }
            else
            {
                a[b_id].set(c[b_id]);
            }

            if (mJoints[b_id].mJointType != JointTypeCustom)
            {
                if (mJoints[b_id].mDoFCount == 1)
                {
                    a[b_id].set(a[b_id] + S[b_id] * (*QDDot)[q_index]);
                }
                else if (mJoints[b_id].mDoFCount == 3)
                {
                    Vector3d omegadot_temp((*QDDot)[q_index], (*QDDot)[q_index + 1], (*QDDot)[q_index + 2]);
                    a[b_id].set(a[b_id] + multdof3_S[b_id] * omegadot_temp);
                }
            }
            else
            {
                unsigned int custom_index = mJoints[b_id].custom_joint_index;
                const CustomJoint* custom_joint = mCustomJoints[custom_index];
                unsigned int joint_dof_count = custom_joint->mDoFCount;

                a[b_id].set(a[b_id] + (mCustomJoints[custom_index]->S * (QDDot->block(q_index, 0, joint_dof_count, 1))));
            }
        }
    }

    if (mu[b_id].size() > 1)
    {
        unsigned int mu_size = mu[b_id].size();
        for (unsigned int i = 0; i < mu_size - 1; ++i)
        {
            // spawn threads here
            io_service.post(boost::bind(&Model::crawlChainKinematics, this, mu[b_id][i], branch_ends, Q, QDot, QDDot));
        }

        // we have a current thread that can process this one, so no need to spawn yet another thread
        crawlChainKinematics(mu[b_id][mu_size - 1], branch_ends, Q, QDot, QDDot);

        return;
    }
    else if (mu[b_id].size() == 1)
    {
        crawlChainKinematics(mu[b_id][0], branch_ends, Q, QDot, QDDot);
        return;
    }

    (*branch_ends)++;
}
