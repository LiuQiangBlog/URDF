// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef RDL_KINEMATICS_H
#define RDL_KINEMATICS_H

/**
 * @file Kinematics.hpp
 */

#include <assert.h>
#include <iostream>
#include <rdl_dynamics/FrameVectorPair.hpp>
#include <rdl_dynamics/Model.hpp>

namespace RobotDynamics
{
/** \page kinematics_page Kinematics
 * All functions related to kinematics are specified in the \ref
 * kinematics_group "Kinematics Module".
 *
 * \note Please note that in the Robot Dynamics Library all angles
 * are specified in radians.
 *
 * \defgroup kinematics_group Kinematics
 * @{
 *
 * \note Please note that in the Robot Dynamics Library all angles
 * are specified in radians.
 *
 * @note Please note that before making any calls that involve reference frames
 * you must first call either updateKinematics or updateKinematicsCustom. If this is
 * not done then operations involving frames will return incorrect results.
 */

/** @brief Updates and computes velocities and accelerations of the bodies
 *
 * This function updates the kinematic variables such as body velocities, accelerations,
 * and reference frames in the model to reflect the variables passed to this function.
 *
 * @param model the model
 * @param Q     the positional variables of the model
 * @param QDot  the generalized velocities of the joints
 * @param QDDot the generalized accelerations of the joints
 */
void updateKinematics(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, const Math::VectorNd& QDDot);

/** @brief Selectively updates model internal states of body positions, velocities and/or accelerations.
 *
 * This function updates the kinematic variables such as body velocities,
 * accelerations, and reference frames in the model to reflect the variables passed to this function.
 *
 * In contrast to RobotDynamics::updateKinematics() this function allows to update the model
 * state with values one is interested and thus reduce computations (e.g. only
 * positions, only positions + velocities, positions + velocities + accelerations).

 * @param model the model
 * @param Q     the positional variables of the model
 * @param QDot  the generalized velocities of the joints
 * @param QDDot the generalized accelerations of the joints
 *
 * @note for each non-null input, the previous input must also be non-null. For example, this call is invalid, updateKinematicsCustom(model, &q, nullptr, &qddot).
 * Because QDot is null, the input qddot will be ignored.
 */
void updateKinematicsCustom(Model& model, const Math::VectorNd* Q, const Math::VectorNd* QDot = nullptr, const Math::VectorNd* QDDot = nullptr);

/** @brief Updates and computes velocities and accelerations of the bodies
 *
 * This function updates the kinematic variables such as body velocities, accelerations,
 * and reference frames in the model to reflect the variables passed to this function.
 *
 * @param model the model
 * @param Q     the positional variables of the model
 * @param QDot  the generalized velocities of the joints
 * @param QDDot the generalized accelerations of the joints
 *
 * @note This function utilizes threading that may result in faster computation of the kinematics for certain models depending on the structure of the
 * kinematic tree as well as the computational capabailities of the computer.
 *
 * @note If the number of additional threads available to the model(See the RobotDynamics::Model constructor and it's behavior) this will default to calling
 * updateKinematics
 *
 * @note The main thread will busy wait until all threads have finished their computation. This may result in high CPU usage for the main
 * thread while it waits; however, in comparing the performance of parallel computation of kinematics it was found that implementing the wait
 * functionality with standard synchronization tools(conditionals etc) actually resulted in longer computation times probably due to the time it takes for the
 * main thread to sleep and then wake up again.
 */
void updateKinematicsParallel(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, const Math::VectorNd& QDDot);

/** @brief Selectively updates model internal states of body positions, velocities and/or accelerations and spawns threads far each branched chain
 *
 * This function updates the kinematic variables such as body velocities,
 * accelerations, and reference frames in the model to reflect the variables passed to this function.
 *
 * In contrast to RobotDynamics::updateKinematics() this function allows to update the model
 * state with values one is interested and thus reduce computations (e.g. only
 * positions, positions + velocities, or positions + velocities + accelerations).

 * @param model the model
 * @param Q     the positional variables of the model
 * @param QDot  the generalized velocities of the joints, defaults nullptr
 * @param QDDot the generalized accelerations of the joints, defaults nullptr
 *
 * @note This function utilizes threading that may result in faster computation of the kinematics for certain models depending on the structure of the
 * kinematic tree as well as the computational capabailities of the computer.
 *
 * @note If the number of additional threads available to the model(See the RobotDynamics::Model constructor and it's behavior) this will default to calling
 updateKinematicsCustom
 *
 * @note for each non-null input, the previous input must also be non-null. For example, this call is invalid, updateKinematicsCustom(model, &q, nullptr, &qddot).
 * Because QDot is null, the input QDDot will be ignored.
 *
 * @note The main thread will busy wait until all threads have finished their computation. This may result in high CPU usage for the main
 * thread while it waits; however, in comparing the performance of parallel computation of kinematics it was found that implementing the wait
 * functionality with standard synchronization tools(conditionals etc) actually resulted in longer computation times probably due to the time it takes for the
 * main thread to sleep and then wake up again.
 */
void updateKinematicsCustomParallel(Model& model, const Math::VectorNd* Q, const Math::VectorNd* QDot = nullptr, const Math::VectorNd* QDDot = nullptr);

/**
 * @brief Computes only the accelerations of the bodies
 *
 * @param model the model
 * @param QDDot the generalized accelerations of the joints
 *
 */
void updateAccelerations(Model& model, const Math::VectorNd& QDDot);

/** @brief Computes the 3D point jacobian for a point on a body
 *
 * If a position of a point is computed by a function \f$g(q(t))\f$ for which its
 * time derivative is \f$\frac{d}{dt} g(q(t)) = G(q)\dot{q}\f$ then this
 * function computes the jacobian matrix \f$G(q)\f$.
 *
 * @param model   rigid body model
 * @param Q       state vector of the internal joints
 * @param body_id the id of the body
 * @param point_position the position of the point in body-local data
 * @param G       a matrix of dimensions 3 x \#qdot_size where the result will be stored in
 * @param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * The result will be returned via the G argument.
 *
 * @note This function only evaluates the entries of G that are non-zero.
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling G.setZero().
 *
 */
void calcPointJacobian(Model& model, const Math::VectorNd& Q, unsigned int body_id, const Math::Vector3d& point_position, Math::MatrixNd& G,
                       bool update_kinematics = true);

/**
 * @brief Compute the 3D point jacobian of the origin of a reference frame
 * If a position of a point is computed by a function \f$g(q(t))\f$ for which its
 * time derivative is \f$\frac{d}{dt} g(q(t)) = G(q)\dot{q}\f$ then this
 * function computes the jacobian matrix \f$G(q)\f$.
 * @param model Rigid body model
 * @param Q     Vector of joint positions
 * @param G     \f$ 3 \times qdot_size\f$ Matrix to store result
 * @param frame The point jacobian will be at the origin of the frame
 * @param update_kinematics If true, kinematics will be calculated
 *
 * @note This function only evaluates the entries of G that are non-zero.
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling G.setZero().
 */
void calcPointJacobian(Model& model, const Math::VectorNd& Q, Math::MatrixNd& G, ReferenceFramePtr frame, bool update_kinematics = true);

/**
 * @brief Compute the 6D point jacobian of the origin of a reference frame
 * If a position of a point is computed by a function \f$g(q(t))\f$ for which its
 * time derivative is \f$\frac{d}{dt} g(q(t)) = G(q)\dot{q}\f$ then this
 * function computes the jacobian matrix \f$G(q)\f$.
 *
 * @param model Rigid body model
 * @param Q     Vector of joint positions
 * @param G     \f$ 6 \times qdot_size\f$ Matrix to store result
 * @param frame The point jacobian will be at the origin of the frame
 * @param update_kinematics If true, kinematics will be calculated
 *
 * @note This function only evaluates the entries of G that are non-zero.
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling G.setZero().
 */
void calcPointJacobian6D(Model& model, const Math::VectorNd& Q, Math::MatrixNd& G, ReferenceFramePtr frame, bool update_kinematics = true);

/**
 * @brief Compute the 6D jacobian of the origin of a reference frame relative
 * to the origin of another reference frame and express the result in a third frame.
 *
 * @param model Rigid body model
 * @param Q Vector of joint positions
 * @param G \f$ 6 \times qdot_size\f$ Matrix to store result
 * @param baseFrame Multiplying the resulting jacobian by the vector of joint velocities will
 * result in a motion of this frame relative to the argument relativeFrame
 * @param relativeFrame Multiplying the resulting jacobian by the vector of joint velocities will
 * result in a motion vector of baseFrame relative to this reference frame
 * @param expressedInFrame Multiplying the resulting jacobian by the vector of joint velocities will
 * result in a motion vector expressed in this frame
 * @param update_kinematics If true, kinematics will be calculated, defaults to true.
 */
void calcRelativePointJacobian6D(Model& model, const Math::VectorNd& Q, Math::MatrixNd& G, ReferenceFramePtr baseFrame, ReferenceFramePtr relativeFrame,
                                 ReferenceFramePtr expressedInFrame = RobotDynamics::ReferenceFrame::getWorldFrame(), bool update_kinematics = true);
/**
 * @brief Compute the time derivative of the 6D jacobian of the origin of a reference frame relative
 * to the origin of another reference frame and express the result in a third frame.
 *
 * @param model Rigid body model
 * @param Q Vector of joint positions
 * @param QDot Vector of joint positions
 * @param G \f$ 6 \times qdot_size\f$ Matrix to store result
 * @param baseFrame Multiplying the resulting jacobian by the vector of joint velocities will
 * result in a motion of this frame relative to the argument relativeFrame
 * @param relativeFrame Multiplying the resulting jacobian by the vector of joint velocities will
 * result in a motion vector of baseFrame relative to this reference frame
 * @param expressedInFrame Multiplying the resulting jacobian by the vector of joint velocities will
 * result in a motion vector expressed in this frame
 * @param update_kinematics If true, kinematics will be calculated, defaults to true.
 */
void calcRelativePointJacobianDot6D(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, Math::MatrixNd& G, ReferenceFramePtr baseFrame,
                                    ReferenceFramePtr relativeFrame, ReferenceFramePtr expressedInFrame = RobotDynamics::ReferenceFrame::getWorldFrame(),
                                    bool update_kinematics = true);

/**
 * @brief Compute the point Jacobian of the origin of baseFrame, relative to the origin of relativeFrame, expressed in
 * expressedInFrame. Also compute that point Jacobians time derivative.
 *
 * @param model Rigid body model
 * @param Q Vector of joint positions
 * @param QDot Vector of joint positions
 * @param G \f$ 6 \times qdot_size\f$ Matrix to store jacobian
 * @param GDot \f$ 6 \times qdot_size\f$ Matrix to store jacobian dot
 * @param baseFrame Multiplying the resulting jacobian by the vector of joint velocities will
 * result in a motion of this frame relative to the argument relativeFrame
 * @param relativeFrame Multiplying the resulting jacobian by the vector of joint velocities will
 * result in a motion vector of baseFrame relative to this reference frame
 * @param expressedInFrame Multiplying the resulting jacobian by the vector of joint velocities will
 * result in a motion vector expressed in this frame
 * @param update_kinematics If true, kinematics will be calculated, defaults to true.
 *
 * @note If you need both the jacobian and it's time derivative, this function should be more efficient than calculating them
 * both with separate function calls.
 *
 * @note Make sure to call G.setZero() and GDot.setZero() before calling this function as it will only evaluate the elements of the jacobian and it's derivative
 * that are nonzero
 */
void calcRelativePointJacobianAndJacobianDot6D(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, Math::MatrixNd& G, Math::MatrixNd& GDot,
                                               ReferenceFramePtr baseFrame, ReferenceFramePtr relativeFrame,
                                               ReferenceFramePtr expressedInFrame = RobotDynamics::ReferenceFrame::getWorldFrame(), bool update_kinematics = true);

/**
 * @brief Computes the jacobian of a frame \f$J_{i}^{k,j}\f$ with \f$i\f$ being the "base" frame, \f$j\f$ being the
 *"relative" frame, and \f$k\f$ being
 * the "expressed in" frame. Multiplying this jacobian by a vector of joint velocities will result in the spatial motion of
 *the baseFrame w.r.t
 * relativeFrame expressed in expressedInFrame, a.k.a \f$v_{i}^{k,j} = J_{i}^{k,j}\dot{q}\f$
 *
 * @param model
 * @param Q
 * @param G
 * @param baseFrame
 * @param relativeFrame
 * @param expressedInFrame
 * @param update_kinematics
 *
 * @note This function only modifies the elements of the jacobian that are nonzero, so be sure the other elements are
 * not nonzero because those elements will not be zero'd. Best practice would be to call G.setZero() before calling
 * this function
 *
 * @note <b>If expressedInFrame=nullptr then the expressedInFrame will be set to baseFrame</b>
 */
void calcRelativeBodySpatialJacobian(Model& model, const Math::VectorNd& Q, Math::MatrixNd& G, ReferenceFramePtr baseFrame, ReferenceFramePtr relativeFrame,
                                     ReferenceFramePtr expressedInFrame = nullptr, bool update_kinematics = true);

/**
 * @brief Computes the rate of change of the jacobian, \f$\dot{J}_{i}^{k,j}\f$, with \f$i\f$ being the "base" frame,
 * \f$j\f$ being the relative" frame, and \f$k\f$ being the "expressed in" frame. This jacobian is such that the
 * following is true, \f$a_{i}^{k,j} = J_{i}^{k,j}\ddot{q} + \dot{J}_{i}^{k,j}\dot{q}\f$ where
 * \f$a_{i}^{k,j}\f$ is the spatial acceleration of frame \f$i\f$ w.r.t frame \f$j\f$, expressed in
 * frame \f$k\f$. Additionally the jacobian \f$J_{i}^{k,j}\f$ can be computed by
 * RobotDynamics::calcRelativeBodySpatialJacobian.
 * @param model
 * @param Q
 * @param QDot
 * @param G
 * @param baseFrame
 * @param relativeFrame
 * @param expressedInFrame
 * @param update_kinematics
 *
 * * @note <b>If expressedInFrame=nullptr then the expressedInFrame will be set to baseFrame</b>
 */
void calcRelativeBodySpatialJacobianDot(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, Math::MatrixNd& G, ReferenceFramePtr baseFrame,
                                        ReferenceFramePtr relativeFrame, ReferenceFramePtr expressedInFrame = nullptr, bool update_kinematics = true);

/**
 * @brief Computes both the body spatial jacobian and its time derivative. This function will be a bit more efficient at computing
 * the jacobian and it's time derivative if you need both matrices rather than computing them with separate function calls.
 * @param model
 * @param Q
 * @param QDot
 * @param G
 * @param GDot
 * @param baseFrame
 * @param relativeFrame
 * @param expressedInFrame
 * @param update_kinematics
 *
 * @note <b>Only non-zero elements of G and GDot will be evaluated. Be sure to call G.setZero() and GDot.setZero() before calling this function</b>
 */
void calcRelativeBodySpatialJacobianAndJacobianDot(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, Math::MatrixNd& G, Math::MatrixNd& GDot,
                                                   ReferenceFramePtr baseFrame, ReferenceFramePtr relativeFrame, ReferenceFramePtr expressedInFrame = nullptr,
                                                   bool update_kinematics = true);

/**
 * @brief Computes the time derivative of the linear components the a point jacobian on a body.
 *
 * For a point \f$p_{i}\f$ on body \f$i\f$ expressed in body \f$i\f$'s frame, this function
 * computes \f$ \dot{J} \f$ such that \f$a_{i} = J(q)\ddot{q} + \dot{J}(q,\dot{q})\dot{q}\f$, where
 * \f$a_{i}\f$ is the 3D linear acceleration of point \f$p\f$, and \f$a_{i}\f$ is expressed in
 * world frame.
 *
 * @param model Rigid body model
 * @param Q Joint positions
 * @param QDot Joint velocities
 * @param body_id Id of the body
 * @param point_position 3D position on body
 * @param G Matrix where the result is stored
 * @param update_kinematics Defaults to true. If true, kinematics will be calculated. If kinematics have already
 * been calculated, setting this to false will save time
 *
 * @note This function only evaluates the entries of G that are non-zero. One
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling G.setZero().
 */
void calcPointJacobianDot(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, unsigned int body_id, const Math::Vector3d& point_position,
                          Math::MatrixNd& G, bool update_kinematics = true);

/**
 * @brief Computes the time derivative of a point jacobian of a point on a body
 *
 * For a reference frame with origin \f$p_{i}\f$, this function
 * computes \f$ \dot{J} \f$ such that \f$a_{i} = J(q)\ddot{q} + \dot{J}(q,\dot{q})\dot{q}\f$, where
 * \f$a_{i}\f$ is the 3D linear acceleration of point \f$p\f$, and \f$a_{i}\f$ is expressed in
 * world frame.
 *
 * @param model Rigid body model
 * @param Q Joint positions
 * @param QDot Joint velocities
 * @param frame Resulting point jacobian will be of the origin of this frame
 * @param G Matrix where the result is stored, must be a \f$ 6 \times qdot_size \f$ matrix
 * @param update_kinematics Defaults to true. If true, kinematics will be calculated. If kinematics have already
 * been calculated, setting this to false will save time
 *
 * @note This function only evaluates the entries of G that are non-zero. One
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling G.setZero().
 */
void calcPointJacobianDot6D(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, RobotDynamics::ReferenceFramePtr frame, Math::MatrixNd& G,
                            bool update_kinematics = true);

/**
 * @brief Computes the time derivative of a point jacobian of a point on a body. Only computes the linear elemets
 *
 * For a reference frame with origin \f$p_{i}\f$, this function
 * computes \f$ \dot{J} \f$ such that \f$a_{i} = J(q)\ddot{q} + \dot{J}(q,\dot{q})\dot{q}\f$, where
 * \f$a_{i}\f$ is the 3D linear acceleration of point \f$p\f$, and \f$a_{i}\f$ is expressed in
 * world frame.
 *
 * @param model Rigid body model
 * @param Q Joint positions
 * @param QDot Joint velocities
 * @param frame Resulting point jacobian will be of the origin of this frame
 * @param G Matrix where the result is stored, must be a \f$ 3 \times qdot_size \f$ matrix
 * @param update_kinematics Defaults to true. If true, kinematics will be calculated. If kinematics have already
 * been calculated, setting this to false will save time
 *
 * @note This function only evaluates the entries of G that are non-zero. One
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling G.setZero().
 */
void calcPointJacobianDot(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, RobotDynamics::ReferenceFramePtr frame, Math::MatrixNd& G,
                          bool update_kinematics = true);

/** @brief Computes a 6-D Jacobian for a point on a body
 *
 * Computes the 6-D Jacobian \f$G(q)\f$ that when multiplied with
 * \f$\dot{q}\f$ gives a 6-D vector that has the angular velocity as the
 * first three entries and the linear velocity as the last three entries.
 *
 * @param model   rigid body model
 * @param Q       state vector of the internal joints
 * @param body_id the id of the body
 * @param point_position the position of the point in body-local data
 * @param G       a matrix of dimensions 6 x \#qdot_size where the result will be stored in
 * @param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * The result will be returned via the G argument.
 *
 * @note This function only evaluates the entries of G that are non-zero. One
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling G.setZero().
 *
 */
void calcPointJacobian6D(Model& model, const Math::VectorNd& Q, unsigned int body_id, const Math::Vector3d& point_position, Math::MatrixNd& G,
                         bool update_kinematics = true);

/**
 * @brief Computes the 6D time derivative of a point jacobian on a body.
 *
 * For a point \f$ p_{i} \f$ on body \f$i\f$ expressed in body \f$ i \f$'s frame, this function
 * computes \f$ \dot{J} \f$ such that \f$ a_{i} = J(q)\ddot{q} + \dot{J}(q,\dot{q})\dot{q}\f$, where
 * \f$a_{i}\f$ is the 6D acceleration of point \f$p\f$, and \f$a_{i}\f$ is expressed in
 * world frame.
 *
 * Essentially, this method computes the jacobian of a reference frame located at point \f$p_{i}\f$, but aligned
 * with the world frame.
 *
 * @param model Rigid body model
 * @param Q Joint positions
 * @param QDot Joint velocities
 * @param body_id Id of the body
 * @param point_position 3D position on body
 * @param G Matrix where the result is stored
 * @param update_kinematics Defaults to true. If true, kinematics will be calculated. If kinematics have already
 * been calculated, setting this to false will save time
 *
 * @note This function only evaluates the entries of G that are non-zero. One
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling G.setZero().
 */
void calcPointJacobianDot6D(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, unsigned int body_id, const Math::Vector3d& point_position,
                            Math::MatrixNd& G, bool update_kinematics = true);

/** @brief Computes the spatial jacobian for a body.
 * The result will be returned via the G argument and represents the
 * body Jacobian expressed at the origin of the body. The corresponding
 * spatial velocity of the body w.r.t the world frame expressed in
 * body frame can be calculated, for the \f$ ith \f$ body, as
 * \f$ v^{i,0}_{i} = G(q) \dot{q} \f$. For the spatial jacobian of
 * body \f$i\f$, each column j is computed as follows
 *
 * \f[
 *
 * G_{j}
 * =
 * \begin{cases}
 * \mathbf{0} & \text{if $j \notin \lambda_{i}$} \\
 * ^{i}X_{j}S_{j} & \text{otherwise}
 * \end{cases}
 *
 * \f]
 *
 * @param model   rigid body model
 * @param Q       state vector of the internal joints
 * @param body_id the id of the body
 * @param G       a matrix of size 6 x \#qdot_size where the result will be stored in
 * @param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * @note This function only evaluates the entries of G that are non-zero. One
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling G.setZero().
 */
void calcBodySpatialJacobian(Model& model, const Math::VectorNd& Q, unsigned int body_id, Math::MatrixNd& G, bool update_kinematics = true);

/** @brief Computes the time derivative of the spatial jacobian for a body.
 * The result will be returned via the G argument and represents the time
 * derivative of the body Jacobian expressed at the origin of the body. The corresponding
 * spatial acceleration of the body w.r.t the world frame expressed in
 * body frame can be calculated, for the \f$i\f$th body, as
 * \f$ a^{i,0}_{i} = G(q) \ddot{q} + \dot{G}(q)\dot{q} \f$ where \f$G(q)\f$
 * is the body jacobian of body \f$i\f$.
 *
 * @param model   rigid body model
 * @param Q       joint positions
 * @param QDot    joint velocities
 * @param body_id the id of the body
 * @param G       a matrix of size 6 x \#qdot_size where the result will be stored in
 * @param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * @note This function only evaluates the entries of G that are non-zero. One
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling G.setZero().
 */
void calcBodySpatialJacobianDot(Model& model, const Math::VectorNd& Q, const Math::VectorNd QDot, unsigned int body_id, Math::MatrixNd& G,
                                const bool update_kinematics = true);

/** @brief Computes the velocity of a point on a body
 *
 * @param model   rigid body model
 * @param Q       state vector of the internal joints
 * @param QDot    velocity vector of the internal joints
 * @param body_id the id of the body
 * @param point_position the position of the point in body-local data
 * @param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * @returns A FrameVector representing the points velocity in world frame.
 */
Math::FrameVector calcPointVelocity(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, unsigned int body_id, const Math::Vector3d& point_position,
                                    bool update_kinematics = true);

/** @brief Computes angular and linear velocity of a point that is fixed on a body
 *
 * @param model   rigid body model
 * @param Q       state vector of the internal joints
 * @param QDot    velocity vector of the internal joints
 * @param body_id the id of the body
 * @param point_position the position of the point in body-local data
 * @param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * @returns The frame vector pair in the global reference system.
 */
Math::FrameVectorPair calcPointVelocity6D(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, unsigned int body_id, const Math::Vector3d& point_position,
                                          bool update_kinematics = true);

/** @brief Computes angular and linear velocity of the origin of a reference frame relative
 * to world and expressed in world frame
 *
 * @param model   rigid body model
 * @param Q       state vector of the internal joints
 * @param QDot    velocity vector of the internal joints
 * @param frame   The frame whose origin will be used to calc the point velocity
 * @param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * @returns The a frame vector pair representing the two 3d vectors that describe the point velocity
 */
Math::FrameVectorPair calcPointVelocity6D(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, RobotDynamics::ReferenceFramePtr frame,
                                          bool update_kinematics = true);

/** @brief Computes angular and linear velocity of the origin of a reference frame relative to
 * another reference frame and expressed in a third reference frame
 *
 * @param model   rigid body model
 * @param Q       state vector of the internal joints
 * @param QDot    velocity vector of the internal joints
 * @param baseFrame   The frame whose origin the point jacobian will be about
 * @param relativeFrame   The resultant velocity will be relative to the origin of this frame
 * @param expressedInFrame The resultant velocity will be expressed in this frame
 * @param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * @returns A frame vector pair representing the point velocity of baseFrame relative to relativeFrame and
 * expressed in expressedInFrame
 *
 */
Math::FrameVectorPair calcPointVelocity6D(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, RobotDynamics::ReferenceFramePtr baseFrame,
                                          RobotDynamics::ReferenceFramePtr relativeFrame,
                                          RobotDynamics::ReferenceFramePtr expressedInFrame = RobotDynamics::ReferenceFrame::getWorldFrame(),
                                          bool update_kinematics = true);

/** @brief Computes the linear acceleration of a point on a body
 *
 * @param model   rigid body model
 * @param Q       state vector of the internal joints
 * @param QDot    velocity vector of the internal joints
 * @param QDDot    velocity vector of the internal joints
 * @param body_id the id of the body
 * @param point_position the position of the point in body-local data
 * @param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * @returns The cartesian acceleration of the point in world frame (output)
 *
 * The kinematic state of the model has to be updated before valid
 * values can be obtained. This can either be done by calling
 * UpdateKinematics() or setting the last parameter update_kinematics to
 * true (default).
 *
 * @warning  If this function is called after ForwardDynamics() without
 * an update of the kinematic state one has to add the gravity
 * acceleration has to be added to the result.
 */

Math::FrameVector calcPointAcceleration(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, const Math::VectorNd& QDDot, unsigned int body_id,
                                        const Math::Vector3d& point_position, bool update_kinematics = true);

/**
 * @brief Compute the spatial velocity of any frame with respect to any other frame, expressed in an arbirtary third frame. The returned
 **RobotDynamicS::Math::SpatialMotion
 * is expressed in body_frame unless th expressedInFrame is provided. Each time RobotDynamics::updateKinematics is called, the spatial velocity
 **of
 * each body with respect to the world, and expressed in body frame is calculated. For body \f$ i \f$ this is written
 * as \f$ v^{i,0}_{i} \f$. Given another body, body \f$ j \f$, the velocity of body \f$ i \f$ relative to body \f$ j \f$ and
 * expressed in body frame \f$ i \f$ is computed by,
 * \f[
 *  v^{i,j}_{i} = v^{i,0}_{i} - ^{i}X_{j} v^{j,0}_{j}
 * \f]
 *
 * @param model A RDL robot model
 * @param Q Vector of joint positions
 * @param QDot Vector of joint velocities
 * @param body_frame The primary frame
 * @param relative_body_frame The frame the result will be w.r.t
 * @param expressedInFrame Frame the result will be expressed in. Default value=nullptr in which case the result will be expressed in the frame corresponding to body_id
 * @param update_kinematics
 * @return
 *
 * @note If no expressedInFrame is given then the result will be expressed in the frame corresponding to the arg body_id.
 */
Math::SpatialMotion calcSpatialVelocity(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, ReferenceFramePtr body_frame,
                                        ReferenceFramePtr relative_body_frame, ReferenceFramePtr expressedInFrame = nullptr, const bool update_kinematics = true);

/**
 * @brief Compute the spatial velocity of any body with respect to any other body. The returned
 **RobotDynamicS::Math::SpatialMotion
 * is expressed in the body frame of body body_id. Each time RobotDynamics::updateKinematics is called, the spatial velocity
 **of
 * each body with respect to the world, and expressed in body frame is calculated. For body \f$ i \f$ this is written
 * as \f$ v^{i,0}_{i} \f$. Given another body, body \f$ j \f$, the velocity of body \f$ i \f$ relative to body \f$ j \f$ and
 * expressed in body frame \f$ i \f$ is computed by,
 * \f[
 *  v^{i,j}_{i} = v^{i,0}_{i} - ^{i}X_{j} v^{j,0}_{j}
 * \f]
 *
 * @param model A RDL robot model
 * @param Q Vector of joint positions
 * @param QDot Vector of joint velocities
 * @param body_id The
 * @param relative_body_id
 * @param expressedInFrame Frame the result will be expressed in. Default value=nullptr in which case the result will be expressed in the frame corresponding to body_id
 * @param update_kinematics
 * @return
 *
 * @note If no expressedInFrame is given then the result will be expressed in the frame corresponding to the arg body_id.
 */
Math::SpatialMotion calcSpatialVelocity(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, const unsigned int body_id,
                                        const unsigned int relative_body_id, ReferenceFramePtr expressedInFrame = nullptr, const bool update_kinematics = true);

/**
 * @brief Compute the spatial acceleration of any body with respect to any other body and express the
 * result in an arbitrary reference frame. The returned RobotDynamicS::Math::SpatialAcceleration
 * can be expressed in the reference frame of choice by supplying the expressedInFrame argument. If left to
 * the default value of nullptr, the result will be expressed in the reference frame corresponding to body_id.
 *
 * @note If the expressedInFrame arg is nullptr, the resulting acceleration will be expressed in the referenceFrame corresponding to body_id
 *
 * @note that in this notation \f$\widetilde{v}\f$ is the same as the cross operator,\f$v\times \f$, commonly used in RBD
 * by Featherstone.
 *
 * @param model A RDL robot model
 * @param Q Vector of joint positions
 * @param QDot Vector of joint velocities
 * @param QDDot Vector of joint accelerations
 * @param body_id The
 * @param relative_body_id
 * @param update_kinematics
 * @param expressedInFrame The reference frame to express the result
 * @return
 */
Math::SpatialAcceleration calcSpatialAcceleration(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, const Math::VectorNd& QDDot,
                                                  const unsigned int body_id, const unsigned int relative_body_id, ReferenceFramePtr expressedInFrame = nullptr,
                                                  const bool update_kinematics = true);

/**
 * @brief Compute the spatial acceleration of any frame with respect to any other frame and express the
 * result in an arbitrary third frame. The returned RobotDynamicS::Math::SpatialAcceleration
 * can be expressed in the reference frame of choice by supplying the expressedInFrame argument. If left to
 * the default value of nullptr, the result will be expressed in body_frame.
 *
 * @note If the expressedInFrame arg is nullptr, the resulting acceleration will be expressed in body_frame
 *
 * @note that in this notation \f$\widetilde{v}\f$ is the same as the cross operator,\f$v\times \f$, commonly used in RBD
 * by Featherstone.
 *
 * @param model A RDL robot model
 * @param Q Vector of joint positions
 * @param QDot Vector of joint velocities
 * @param QDDot Vector of joint accelerations
 * @param body_frame The frame of the body
 * @param relative_body_frame The body frame of the relative body
 * @param expressedInFrame The frame the result will be expressed in. Defaults to nullptr which will be world frame
 * @param update_kinematics If true, kinematics will be calculated
 * @return Spatial acceleration of body_frame relative to relative_frame expressed in expressedInFrame
 */
Math::SpatialAcceleration calcSpatialAcceleration(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, const Math::VectorNd& QDDot,
                                                  ReferenceFramePtr body_frame, ReferenceFramePtr relative_body_frame, ReferenceFramePtr expressedInFrame = nullptr,
                                                  const bool update_kinematics = true);

/** @brief Computes linear and angular acceleration of a point on a body
 *
 * @param model   rigid body model
 * @param Q       state vector of the internal joints
 * @param QDot    velocity vector of the internal joints
 * @param QDDot    velocity vector of the internal joints
 * @param body_id the id of the body
 * @param point_position the position of the point in body-local data
 * @param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * @returns A FrameVectorPair of the desired point with respect to world expressed in world frame
 *
 * The kinematic state of the model has to be updated before valid
 * values can be obtained. This can either be done by calling
 * updateKinematics() or setting the last parameter update_kinematics to
 * true (default).
 *
 * @warning  If this function is called after ForwardDynamics() without
 * an update of the kinematic state one has to add the gravity
 * acceleration has to be added to the result.
 */
Math::FrameVectorPair calcPointAcceleration6D(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, const Math::VectorNd& QDDot, unsigned int body_id,
                                              const Math::Vector3d& point_position, bool update_kinematics = true);

/** @brief Computes linear and angular acceleration of a reference frame, relative to another reference frame
 * and expressed in a third reference frame
 *
 * @param model   rigid body model
 * @param Q       state vector of the internal joints
 * @param QDot    velocity vector of the internal joints
 * @param QDDot    velocity vector of the internal joints
 * @param body_frame frame to calculate the acceleration of
 * @param relative_body_frame calculated acceleration will be relative to this frame
 * @param expressedInFrame resulting acceleration will be expressed in this frame
 * @param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * @returns A frame vector pair representing the acceleration of the origin of body_frame w.r.t
 * relative_body_frame and expressed in expressedInFrame
 *
 * The kinematic state of the model has to be updated before valid
 * values can be obtained. This can either be done by calling
 * updateKinematics() or setting the last parameter update_kinematics to
 * true (default).
 *
 * @warning  If this function is called after ForwardDynamics() without
 * an update of the kinematic state one has to add the gravity
 * acceleration has to be added to the result.
 */
Math::FrameVectorPair calcPointAcceleration6D(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, const Math::VectorNd& QDDot,
                                              ReferenceFramePtr body_frame, ReferenceFramePtr relative_body_frame, ReferenceFramePtr expressedInFrame = nullptr,
                                              const bool update_kinematics = true);

/** \brief Computes the inverse kinematics iteratively using a damped Levenberg-Marquardt method (also known as Damped Least
   Squares method)
 *
 * @param model rigid body model
 * @param Qinit initial guess for the state
 * @param body_id a vector of all bodies for which we we have kinematic target positions
 * @param body_point a vector of points in body local coordinates that are
 * to be matched to target positions
 * @param target_pos a vector of target positions
 * @param Qres output of the computed inverse kinematics
 * @param step_tol tolerance used for convergence detection
 * @param lambda damping factor for the least squares function
 * @param max_iter maximum number of steps that should be performed
 * @returns true on success, false otherwise
 *
 * This function repeatedly computes
 *   \f[ Qres = Qres + \Delta \theta\f]
 *   \f[ \Delta \theta = G^T (G^T G + \lambda^2 I)^{-1} e \f]
 * where \f$G = G(q) = \frac{d}{dt} g(q(t))\f$ and \f$e\f$ is the
 * correction of the body points so that they coincide with the target
 * positions. The function returns true when \f$||\Delta \theta||_2 \le\f$
 * step_tol or if the error between body points and target gets smaller
 * than step_tol. Otherwise it returns false.
 *
 * The parameter \f$\lambda\f$ is the damping factor that has to
 * be chosen carefully. In case of unreachable positions higher values (e.g
 * 0.9) can be helpful. Otherwise values of 0.0001, 0.001, 0.01, 0.1 might
 * yield good results. See the literature for best practices.
 *
 * @warning The actual accuracy might be rather low (~1.0e-2)! Use this function with a
 * grain of suspicion.
 */
bool inverseKinematics(Model& model, const Math::VectorNd& Qinit, const std::vector<unsigned int>& body_id, const std::vector<Math::Vector3d>& body_point,
                       const std::vector<Math::Vector3d>& target_pos, Math::VectorNd& Qres, double step_tol = 1.0e-12, double lambda = 0.01, unsigned int max_iter = 50);

/** @} */
}  // namespace RobotDynamics

/* RDL_KINEMATICS_H */
#endif  // ifndef RDL_KINEMATICS_H
