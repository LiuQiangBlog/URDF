/*
 * Original Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 *
 * RDL - Robot Dynamics Library
 * Modifications Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

/**
 * @file examples.hpp
 */

#ifndef __RDL_EXAMPLES_HPP__
#define __RDL_EXAMPLES_HPP__

namespace RobotDynamics
{
namespace Examples
{
/**
 * @page examples_page Examples
 *
 * This page shows a number of examples that hopefully will bootstrap the learning process for users
 * wanting to use RDL. If there is a specific example or use case that you would like to see that is
 * not already shown here, please open an <a href="https://gitlab.com/jlack/rdl/issues">issue</a> so we can get
 * it added.
 *
 * @section double_pendulum 2 Link Pendulum
 *
 * In this example, we will construct a robot model of a 2 link pendulum robot.
 *
 * @code{.cpp}
 *
 * // For this example, we will create a shared pointer to a model, but it's also perfectly valid to
 * // construct an instance directly. Many RDL functions take pointers to a model, this is really the
 * // only motivation for using a pointer instead of an instance directly.
 *
 * // Upon construction, a body for world and a world frame are created, but no more bodies/frames exist until we add them.
 * RobotDynamics::ModelPtr model(new RobotDynamics::Model());
 *
 * // Create body 1
 * RobotDynamics::Body body1(1.0, Math::Vector3d(0.,0.,-0.1), Math::Vector3d(0.1,0.1,0.1);
 *
 * // Create body 2
 * RobotDynamics::Body body2(2.0, Math::Vector3d(0.,0.,-0.2), Math::Vector3d(0.1,0.1,0.1);
 *
 * // Now add the two bodies of the pendulum. We will use Model::appendBody here, but Model::addBody would work
 * // as well. The 'addBody' function requires as an argument the parent body you are adding to, but 'appendBody'
 * // adds the body to the previously added body.
 * unsigned int body1_id = model->appendBody(Math::SpatialTransform(), JointTypeRevoluteX,body1,"body1");
 * unsigned int body2_id =  model->appendBody(Math::SpatialTransform(RobotDynamics::Math::Xtrans(RobotDynamics::Math::Vector3d(0.,0.,-1.0))),
 JointTypeRevoluteX,body2,"body2");
 *
 * // Each time a body is added, a reference frame is created for the body. These frames can be accessed by,
 * RobotDynamics::ReferenceFramePtr body1_frame = model->bodyFrames[body1_id];
 * RobotDynamics::ReferenceFramePtr body2_frame = model->bodyFrames[body2_id];
 *
 * // Frames are alse created at the body's center of mass. These frames can be accessed by,
 * RobotDynamics::ReferenceFramePtr body1_com_frame = model->bodyCenteredFrames[body1_id];
 * RobotDynamics::ReferenceFramePtr body2_com_frame = model->bodyCenteredFrames[body2_id];
 *
 * // Now update kinematics(see the available functions from the kinematics module or look in Kinematics.h)
 * void updateKinematics(*model, Q, QDot, QDDot) // Q,QDot, and QDDot here are vectors of joint positions, velocities, and accelerations

 * // Until kinematics have been updated, the transforms stored in each reference frame will not necessarily be correct.
 * // Also, you should be sure to update the kinematics each control tick.
 *
 * // Now some examples of some simple things you can do utilizing the reference frames
 * // Create a point at the origin of body2_frame
 * FramePoint body2_origin(body2_frame, 0.,0.,0.);
 *
 * // Now lets express that point in body1_frame
 * body2_origin.changeFrame(body1_frame) // body2_origin will now be expressed in the reference frame attached to body1
 *
 * // To express the frame point(or any geometric quantity that has a frame) in world frame, this con be done by,
 * body2_origin.changeFrame(model->worldFrame); // Now it's in world frame
 *
 * @endcode
 *
 */
}
}  // namespace RobotDynamics

#endif
