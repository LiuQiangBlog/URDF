//
// RDL - Robot Dynamics Library
// Modifications Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
//
// Licensed under the zlib license. See LICENSE for more details.
//

/**
 * @file HydroDynamics.hpp
 */

#ifndef __RDL_HYDRODYNAMICS_HPP__
#define __RDL_HYDRODYNAMICS_HPP__

#include <rdl_dynamics/Model.hpp>

/** \page dynamics_page Dynamics
 *
 * All functions related to hydro dynamics are specified in the \ref
 * hydro_dynamics_group "Hydro Dynamics Module".
 *
 * \defgroup hydro_dynamics_group HydroDynamics
 * @{
 */

namespace RobotDynamics
{
/** @brief Computes forward dynamics with the Articulated Body Algorithm including hydrodynamics effects
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
 * @param update_kinematics Default true. If false, kinematics will not be computed
 *
 * @note If you pass update_kinematics=false, then you must have previously calculated the kinematics from one of
 * the other functions, e.g. updateKinematics or updateKinematicsCustom. If you have not computed kinematics, then
 * passing update_kinematics will give an incorrect result
 */
void forwardHydroDynamics(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, const Math::VectorNd& Tau, Math::VectorNd& QDDot,
                          Math::SpatialForceV* f_ext = nullptr, bool update_kinematics = true);

/** @brief Computes the joint space inertia matrix by using the Composite Rigid Body Algorithm. The resulting
 * inertia matrix will include the effects of fluid added inertia
 *
 * This function computes the joint space inertia matrix from a given model and
 * the generalized state vector:
 *   \f$ M(q) = M_{rb}(q) + M_{A} \f$ where \f$ M(q)_{rb} \f$ is what you would get from RobotDynamics::compositeRigidBodyAlgorithm
 *   and \f$ M(q)_{A} \f$ is the inertia dependent from the effects of the fluid. Notice that it is not dependent on
 *   the joint position variables. It is only geometry dependent.
 *
 * @param model rigid body model
 * @param Q     state vector of the model
 * @param H     a matrix where the result will be stored in
 * @param update_kinematics  whether the kinematics should be updated (safer, but at a higher computational cost!)
 *
 * @note This function only evaluates the entries of H that are non-zero.
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling H.setZero().
 */
void compositeRigidBodyAlgorithmHydro(Model& model, const Math::VectorNd& Q, Math::MatrixNd& H, bool update_kinematics = true);

/** @brief Computes the coriolis forces, gravity/buoyancy forces as well as hydroydnamic drag and added mass
 *
 * This function computes the generalized forces from given generalized
 * states, and velocities from gravity and coriolis effects including hydrodynamic terms
 * like buoyancy, added mass, and drag:
 *   \f$ \tau = N(q, \dot{q}) \f$
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
 */
void nonlinearEffectsHydro(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, Math::VectorNd& Tau, bool update_kinematics = true);

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
 */
void coriolisEffectsHydro(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, Math::VectorNd& Tau, bool update_kinematics = true);

/** @brief Computes the drag vector
 *
 * This function computes the drag vector for an RDL model
 *
 * @param model rigid body model
 * @param Tau   Drag vector. Modified
 */
void dragEffects(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, Math::VectorNd& Tau, bool update_kinematics = true);

/** @brief Computes the buoyancy vector
 *
 * This function computes the buoyancy vector for an RDL model
 *
 * @param model rigid body model
 * @param Tau   Buoyancy vector. Modified
 *
 * @note This function WILL NOT compute any of the kinematics, so you MUST call
 * either updateKinematics or updateKinematicsCustom prior to calling this function or
 * the result will be wrong.
 */
void buoyancyEffects(Model& model, Math::VectorNd& Tau);

/** @brief Computes the combined buoyancy+gravity vector
 *
 * This function computes the buoyancy+gravity vector for an RDL model
 *
 * @param model rigid body model
 * @param Tau   Vector of torques from gravity and buoyancy. Modified
 *
 * @note This function WILL NOT compute any of the kinematics, so you MUST call
 * either updateKinematics or updateKinematicsCustom prior to calling this function or
 * the result will be wrong.
 */
void potentialEffects(Model& model, Math::VectorNd& Tau);

/** \@brief Computes inverse dynamics with the Newton-Euler Algorithm. This method includes hydrodynamic effects
 * from buoyancy, added mass, and drag
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
 * @param f_ext External forces acting on the body (optional, defaults to NULL)
 * @param update_kinematics  whether the kinematics should be updated (safer, but at a higher computational cost!)
 *
 * @note If you pass update_kinematics=false, then you must have previously calculated the kinematics from one of
 * the other functions, e.g. updateKinematics or updateKinematicsCustom. If you have not computed kinematics, then
 * passing update_kinematics will give an incorrect result
 */
void inverseDynamicsHydro(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, const Math::VectorNd& QDDot, Math::VectorNd& Tau,
                          Math::SpatialForceV* f_ext = nullptr, bool update_kinematics = true);
}  // namespace RobotDynamics

/** @} */

#endif  //__RDL_HYDRODYNAMICS_HPP__
