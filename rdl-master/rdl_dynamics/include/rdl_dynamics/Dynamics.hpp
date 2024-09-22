// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef RDL_DYNAMICS_HPP
#define RDL_DYNAMICS_HPP

#include <assert.h>
#include <iostream>

#include "rdl_dynamics/SpatialForce.hpp"
#include "rdl_dynamics/rdl_mathutils.hpp"

namespace RobotDynamics
{
struct Model;

/** \page dynamics_page Dynamics
 *
 * All functions related to kinematics are specified in the \ref
 * dynamics_group "Dynamics Module".
 *
 * \defgroup dynamics_group Dynamics
 * @{
 */

/** \@brief Computes inverse dynamics with the Newton-Euler Algorithm
 *
 * This function computes the generalized forces from given generalized
 * states, velocities, and accelerations:
 *   \f$ \tau = M(q) \ddot{q} + N(q, \dot{q}) \f$
 *
 * @param model rigid body model
 * @param Q     state vector of the internal joints
 * @param QDot  velocity vector of the internal joints
 * @param QDDot accelerations of the internals joints
 * @param Tau   actuations of the internal joints (output)
 * @param f_ext External forces acting on the body (optional, defaults to nullptr)
 * @param update_kinematics whether or not to calculate kinematics. Defaults to true which will compute kinematic quatities
 *
 * @note If you have already updated kinematics by calling either RobotDynamics::updateKinematics or RobotDynamics::updateKinematicsCustom then you
 * may safely pass update_kinematics=false here which will save you a little bit of computation
 */
void inverseDynamics(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, const Math::VectorNd& QDDot, Math::VectorNd& Tau,
                     Math::SpatialForceV* f_ext = nullptr, bool update_kinematics = true);

/** @brief Computes the coriolis forces
 *
 * This function computes the generalized forces from given generalized
 * states, and velocities from coriolis effects
 *   \f$ \tau = C(q, \dot{q}) \f$
 *
 * @param model rigid body model
 * @param Q     state vector of the internal joints
 * @param QDot  velocity vector of the internal joints
 * @param Tau   actuations of the internal joints (output)
 * @param update_kinematics  whether the kinematics should be updated (safer, but at a higher computational cost!)
 *
 * @note If you pass update_kinematics=false, then you must have previously calculated the kinematics from one of
 * the other functions, e.g. updateKinematics or updateKinematicsCustom. If you have not computed kinematics, then
 * passing update_kinematics will give an incorrect result
 *
 */
void coriolisEffects(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, Math::VectorNd& Tau, bool update_kinematics = true);

/** @brief Computes the gravity vector
 *
 * This function computes the gravity vector for an RDL model
 *
 * @param model rigid body model
 * @param Tau   Gravity vector. Modified
 *
 * @note This function WILL NOT compute any of the kinematics, so you MUST call
 * either updateKinematics or updateKinematicsCustom prior to calling this function or
 * the result will be wrong.
 */
void gravityEffects(Model& model, Math::VectorNd& Tau);

/**
 * @brief Calculate the wrench due to gravity on a body
 * @param model RDL model
 * @param body_id Wrench will be of this body
 * @param gravity_wrench Wrench due to gravity on body body_id. Modified
 *
 * @note This function WILL NOT compute any of the kinematics, so you MUST call
 * either updateKinematics or updateKinematicsCustom prior to calling this function or
 * the result will be wrong.
 */
void calcBodyGravityWrench(Model& model, unsigned int body_id, RobotDynamics::Math::SpatialForce& gravity_wrench);

/** @brief Computes the coriolis forces
 *
 * This function computes the generalized forces from given generalized
 * states, velocities, and accelerations:
 *   \f$ \tau = M(q) \ddot{q} + N(q, \dot{q}) \f$
 *
 * @param model rigid body model
 * @param Q     state vector of the internal joints
 * @param QDot  velocity vector of the internal joints
 * @param Tau   actuations of the internal joints (output)
 * @param update_kinematics whether or not to calculate kinematics. Defaults to true which will compute kinematic quatities
 *
 * @note If you have already updated kinematics by calling either RobotDynamics::updateKinematics or RobotDynamics::updateKinematicsCustom then you
 * may safely pass update_kinematics=false here which will save you a little bit of computation
 */
void nonlinearEffects(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, Math::VectorNd& Tau, bool update_kinematics = true);

/** @brief Computes the joint space inertia matrix by using the Composite Rigid Body Algorithm
 *
 * This function computes the joint space inertia matrix from a given model and
 * the generalized state vector:
 *   \f$ M(q) \f$
 *
 * @param model rigid body model
 * @param Q     state vector of the model
 * @param H     a matrix where the result will be stored in
 * @param update_kinematics  whether the kinematics should be updated (safer, but at a higher computational cost!)
 *
 * @note This function only evaluates the entries of H that are non-zero. One
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling H.setZero().
 */
void compositeRigidBodyAlgorithm(Model& model, const Math::VectorNd& Q, Math::MatrixNd& H, bool update_kinematics = true);

/** @brief Computes forward dynamics with the Articulated Body Algorithm
 *
 * This function computes the generalized accelerations from given
 * generalized states, velocities and forces:
 *   \f$ \ddot{q} = M(q)^{-1} ( -N(q, \dot{q}) + \tau)\f$
 * It does this by using the recursive Articulated Body Algorithm that runs
 * in \f$O(n_{dof})\f$ with \f$n_{dof}\f$ being the number of joints.
 *
 * @param model rigid body model
 * @param Q     state vector of the internal joints
 * @param QDot  velocity vector of the internal joints
 * @param Tau   actuations of the internal joints
 * @param QDDot accelerations of the internal joints (output)
 * @param f_ext External forces acting on the body (optional, defaults to NULL)
 * @param update_kinematics If true, compute kinematics. Default true
 */
void forwardDynamics(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, const Math::VectorNd& Tau, Math::VectorNd& QDDot,
                     Math::SpatialForceV* f_ext = nullptr, bool update_kinematics = true);

/** @brief Computes forward dynamics by building and solving the full Lagrangian equation
 *
 * This method builds and solves the linear system
 * \f[      H \ddot{q} = -C + \tau	\f]
 * for \f$\ddot{q}\f$ where \f$H\f$ is the joint space inertia matrix
 * computed with the CompositeRigidBodyAlgorithm(), \f$C\f$ the bias
 * force (sometimes called "non-linear effects").
 *
 * @param model rigid body model
 * @param Q     state vector of the internal joints
 * @param QDot  velocity vector of the internal joints
 * @param Tau   actuations of the internal joints
 * @param QDDot accelerations of the internal joints (output)
 * @param linear_solver specification which method should be used for solving the linear system
 * @param f_ext External forces acting on the body (optional, defaults to NULL)
 * @param H     joint space inertia matrix of size dof_count x dof_count. Modified
 * @param C     right hand side vector of size dof_count x 1. Modified
 * @param update_kinematics whether or not to calculate kinematics. Defaults to true which will compute kinematic quatities
 *
 * @note If you have already updated kinematics by calling either RobotDynamics::updateKinematics or RobotDynamics::updateKinematicsCustom then you
 * may safely pass update_kinematics=false here which will save you a little bit of computation
 *
 * @note It is up to the caller to ensure the H matrix and C vector are appropriately sized before calling this function.
 */
void forwardDynamicsLagrangian(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, const Math::VectorNd& Tau, Math::VectorNd& QDDot, Math::MatrixNd& H,
                               Math::VectorNd& C, Math::LinearSolver linear_solver = Math::LinearSolverColPivHouseholderQR, Math::SpatialForceV* f_ext = nullptr,
                               bool update_kinematics = true);

/** @brief Computes the effect of multiplying the inverse of the joint
 * space inertia matrix with a vector in linear time.
 *
 * @param model rigid body model
 * @param Q     state vector of the generalized positions
 * @param Tau   the vector that should be multiplied with the inverse of
 *              the joint space inertia matrix
 * @param QDDot vector where the result will be stored
 * @param update_kinematics whether the kinematics should be computed.
 *
 * @note If you have already updated kinematics by calling either RobotDynamics::updateKinematics or RobotDynamics::updateKinematicsCustom then you
 * may safely pass update_kinematics=false here which will save you a little bit of computation
 *
 * This function uses a reduced version of the Articulated %Body Algorithm
 * to compute
 *
 *   \f$ \ddot{q} = M(q)^{-1}*\tau \f$
 *
 * in \f$O(n_{\textit{dof}}\f$) time.
 *
 */
void calcMInvTimesTau(Model& model, const Math::VectorNd& Q, const Math::VectorNd& Tau, Math::VectorNd& QDDot, bool update_kinematics = true);

/** @} */
}  // namespace RobotDynamics

/* RDL_DYNAMICS_H */
#endif  // ifndef RDL_DYNAMICS_H
