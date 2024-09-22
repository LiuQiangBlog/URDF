// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_UTILS_H__
#define __RDL_UTILS_H__

/**
 * @file rdl_utils.hpp
 */

#include <string>

#include "rdl_dynamics/FramePoint.hpp"
#include "rdl_dynamics/Model.hpp"
#include "rdl_dynamics/rdl_mathutils.hpp"

namespace RobotDynamics
{
/** \page utils_page Utility Functions
 *
 * All utility functions are specified in the \ref utils_group "Utilities Module".
 *
 * \defgroup utils_group Utilities
 * @{
 *
 * Utility functions are those not necessarily required for kinematics/dynamics, but
 * provide utility by giving model information, calculating useful quantities such as
 * the position/velocity of the center of mass, etc.
 *
 */

struct Model;

/** \brief Namespace that contains optional helper functions */
namespace Utils
{
/** @brief get string abbreviation for dof name from spatial vector. */
std::string getDofName(const Math::SpatialVector& joint_dof);

/** @brief get body name, returns empty string if bodyid is virtual and has multiple child bodies */
std::string getBodyName(const RobotDynamics::Model& model, unsigned int body_id);

/** \brief Creates a human readable overview of the model. */
std::string getModelHierarchy(const Model& model);

/** \brief Creates a human readable overview of the Degrees of Freedom. */
std::string getModelDOFOverview(const Model& model);

/** \brief Creates a human readable overview of the locations of all bodies that have names. */
std::string getNamedBodyOriginsOverview(Model& model);

/** @brief Computes the Center of Mass (COM) position.
 *
 * @param model The model for which we want to compute the COM
 * @param q The current joint positions
 * @param com (output) location of the Center of Mass of the model in world frame
 * @param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 *
 * @note Updates the transforms of Model::comFrame
 */
void calcCenterOfMass(Model& model, const Math::VectorNd& q, Math::Vector3d& com, bool update_kinematics = true);

/** @brief Computes the Center of Mass (COM) position.
 *
 * @param model The model for which we want to compute the COM
 * @param q The current joint positions
 * @param com (output) location of the Center of Mass of the model in world frame
 * @param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 *
 */
void calcCenterOfMass(Model& model, const Math::VectorNd& q, Math::FramePoint& com, bool update_kinematics = true);

/** @brief Computes the Center of Mass (COM) and optionally its linear velocity.
 *
 * When only interested in computing the location of the COM you can use
 * nullptr as value for com_velocity.
 *
 * @param model The model for which we want to compute the COM
 * @param q The current joint positions
 * @param qdot The current joint velocities
 * @param com (output) location of the Center of Mass of the model in world frame
 * @param com_velocity (optional output) linear velocity of the COM in world frame
 * @param angular_momentum (optional output) angular momentum of the model at the COM in world frame
 * @param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 *
 */
void calcCenterOfMass(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::Vector3d& com, Math::Vector3d* com_velocity = NULL,
                      Math::Vector3d* angular_momentum = NULL, bool update_kinematics = true);

/** @brief Computes the Center of Mass (COM) and optionally its linear velocity and/or angular momentum.
 *
 * When only interested in computing the location of the COM you can use
 * nullptr as value for com_velocity/angular_momentum.
 *
 * @ingroup reference_frame
 *
 * @param model The model for which we want to compute the COM
 * @param q The current joint positions
 * @param qdot The current joint velocities
 * @param com location of the Center of Mass of the model in world frame
 * @param com_velocity linear velocity of the COM in world frame
 * @param angular_momentum (optional output) angular momentum of the model at the COM in a reference frame aligned with
 * the world frame, but located at the center of mass
 * @param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 */
void calcCenterOfMass(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::FramePoint& com, Math::FrameVector& com_velocity,
                      Math::FrameVector* angular_momentum = nullptr, bool update_kinematics = true);

/** @brief Computes the Center of Mass (COM) and optionally its linear velocity.
 *
 * When only interested in computing the location of the COM you can use
 * nullptr as value for com_velocity.
 *
 * @ingroup reference_frame
 *
 * @param model The model for which we want to compute the COM
 * @param q The current joint positions
 * @param qdot The current joint velocities
 * @param com (output) location of the Center of Mass of the model in world frame
 * @param com_velocity (optional output) linear velocity of the COM in world frame
 * @param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 */
void calcCenterOfMass(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::FramePoint& com, Math::FrameVector* com_velocity = nullptr,
                      bool update_kinematics = true);

/** @brief Computes the Center of Mass (COM) and its linear velocity.
 *
 * @param model The model for which we want to compute the COM
 * @param q The current joint positions
 * @param qdot The current joint velocities
 * @param com (output) location of the Center of Mass of the model in world frame
 * @param com_velocity linear velocity of the COM in world frame
 * @param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 */
void calcCenterOfMass(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::Vector3d& com, Math::Vector3d& com_velocity,
                      bool update_kinematics = true);

/** @brief Computes the center of mass and updates the center of mass reference frame of the model
 *
 * @ingroup reference_frame
 *
 * @param model The model for which we want to compute the COM
 * @param q The current joint positions
 * @param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 *
 * @return a frame point that represents the position of the origin of the center of mass frame expressed in world frame
 */
RobotDynamics::Math::FramePoint updateCenterOfMassFrame(Model& model, const Math::VectorNd& q, bool update_kinematics = true);

/** @brief Computes the center of mass and updates the center of mass reference frame of the model
 *
 * @ingroup reference_frame
 *
 * @param model The model for which we want to compute the COM
 * @param q The current joint positions
 * @param euler_ypr The desired orientation of the center of mass frame
 * @param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 *
 * @return a frame point that represents the position of the origin of the center of mass frame expressed in world frame
 */
RobotDynamics::Math::FramePoint updateCenterOfMassFrame(Model& model, const Math::VectorNd& q, const Math::Vector3d& euler_ypr, bool update_kinematics = true);

/** @brief Computes the center of mass and updates the center of mass reference frame of the model
 *
 * @ingroup reference_frame
 *
 * @param model The model for which we want to compute the COM
 * @param q The current joint positions
 * @param orientation The desired orientation of the center of mass frame
 * @param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 *
 * @return a frame point that represents the position of the origin of the center of mass frame expressed in world frame
 */
RobotDynamics::Math::FramePoint updateCenterOfMassFrame(Model& model, const Math::VectorNd& q, const Math::Quaternion& orientation, bool update_kinematics = true);

/** @brief Updates the center of mass frame of the robot be at location p_com, aligned with world frame
 *
 * @ingroup reference_frame
 *
 * @param model The model for which we want to compute the COM
 * @param p_com The com position in world frame
 */
void updateCenterOfMassFrame(Model& model, const Math::Vector3d& p_com);

/** @brief Updates the center of mass frame of the robot be at location p_com and to have orientation defined by the yaw/pitch/roll angles in euler_ypr argument.
 * The euler_ypr argument is optional and if not provided the center of mass reference frame will be aligned with world.
 *
 * @ingroup reference_frame
 *
 * @param model The model for which we want to compute the COM
 * @param p_com The com position in world frame
 * @param euler_ypr The desired orientation of the center of mass frame
 */
void updateCenterOfMassFrame(Model& model, const Math::Vector3d& p_com, const Math::Vector3d& euler_ypr);

/** @brief Updates the center of mass frame of the robot be at location p_com and to have orientation defined by the orientation argument.
 * The quaternion argument is optional and if not provided the center of mass reference frame will be aligned with world.
 *
 * @ingroup reference_frame
 *
 * @param model The model for which we want to compute the COM
 * @param p_com The com position in world frame
 * @param orientation The desired orientation of the center of mass frame
 */
void updateCenterOfMassFrame(Model& model, const Math::Vector3d& p_com, const Math::Quaternion& orientation);

/** @brief Computes the Center of Mass (COM) velocity in world frame.
 *
 * @param model The model for which we want to compute the COM velocity
 * @param q The current joint positions
 * @param qdot The current joint velocities
 * @param com_vel (modified) translational velocity of the Center of Mass of the model in world frame
 * @param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 */
void calcCenterOfMassVelocity(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::Vector3d& com_vel, bool update_kinematics = true);

/** @brief Computes the Center of Mass (COM) velocity in world frame.
 *
 * @param model The model for which we want to compute the COM velocity
 * @param q The current joint positions
 * @param qdot The current joint velocities
 * @param com_vel (modified) Frame vector where the result vector and frame will be stored
 * @param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 */
void calcCenterOfMassVelocity(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::FrameVector& com_vel, bool update_kinematics = true);

/** @brief Computes the Center of Mass (COM) velocity in world frame.
 *
 * @param model The model for which we want to compute the COM velocity
 * @param com_vel (modified) Vector3d where the result vector and frame will be stored
 */
void calcCenterOfMassVelocity(Model& model, Math::Vector3d& com_vel);

/** @brief Computes the Center of Mass (COM) velocity in world frame.
 *
 * @param model The model for which we want to compute the COM velocity
 * @param com_vel (modified) Frame vector where the result vector and frame will be stored
 */
void calcCenterOfMassVelocity(Model& model, Math::FrameVector& com_vel);

/** @brief Computes the potential energy of the full model.
 *
 * @param model
 * @param q
 * @param update_kinematics (optional) Defaults to true
 *
 * @return Potential energy
 */
double calcPotentialEnergy(Model& model, const Math::VectorNd& q, bool update_kinematics = true);

/** @brief Computes the kinetic energy of the full model.
 *
 * @param model
 * @param q
 * @param qdot
 * @param update_kinematics (optional) Defaults to true
 *
 * @return Kinetic energy
 */
double calcKineticEnergy(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, bool update_kinematics = true);

/** @brief Computes the gravitational wrench experienced on the robots center of mass
 *
 * @param model
 * @param q
 * @param update_kinematics (optional) Defaults to true
 *
 * @note The reference frame the resulting force is expressed in is the center of mass reference frame owned by the RobotDynamics::Model object. This means that
 * you must first have the position/orientation of that reference frame updated prior to calling this function otherwise the frame it's expressed in is likely to not
 * be what you want.
 */
Math::SpatialForce calcGravityWrenchOnCenterOfMass(Model& model, const Math::VectorNd& q, bool update_kinematics = true);

/**
 * @brief Computes the matrix \f$J_{com}\f$ such that \f$v_{com} = J_{com}  \dot{q} \f$
 *
 * @param model The model for which the COM jacobian will be computed for
 * @param q The current joint positions
 * @param jCom A 3 x model.qdot_size matrix where the jacobian will be stored.
 * @param update_kinematics If true, kinematic variables will be computed. Default = true.
 */
void calcCenterOfMassJacobian(Model& model, const Math::VectorNd& q, Math::MatrixNd& jCom, bool update_kinematics = true);

/**
 * @brief Calculates the center of mass of a subtree starting with the body with ID bodyId and scales it by the total
 **mass of the subtree.
 *
 * @param model The model used for the calculation
 * @param bodyId The ID of the first body in the desired subtree
 * @param q The current joint positions
 * @param updateKinematics If true, kinematic variables will be computed. Default = true
 */
Math::Vector3d calcSubtreeCenterOfMassScaledByMass(Model& model, const unsigned int bodyId, const Math::VectorNd& q, bool updateKinematics = true);

/**
 * @brief Calculates the total mass of the subtree beginning with body bodyId and traversing outwards from there
 *
 * @param model The model used for the calculation
 * @param bodyId The ID of the first body in the desired subtree
 *
 * @return Mass of subtree
 */
double calcSubtreeMass(Model& model, const unsigned int bodyId);

/**
 * @brief Calculates the centroidal momentum matrix, \f$ A(q) \f$ for a model. The centroidal momentum
 * matrix is a \f$ 6 \times N \f$ matrix such that the 6dof centroidal momentum vector is computed by,
 * \f[
 *  h = A(q) \dot{q}
 * \f]
 *
 * @note It is crucial that the \f$ A \f$ matrix be all zeros when this function is called, as the elements will
 * be added to. To be sure, call A.setZero() before calling this function.
 *
 * @param model RDL model
 * @param q Vector of joint positions
 * @param A 6 by N matrix where the result will be stored
 * @param update_kinematics If true, calculates kinematic parameters
 *
 * @note this function will compute the center of mass. If you already have this quantity it will be more efficient to use the function
 * that takes this as an argument so it doesn't recompute them
 */
void calcCentroidalMomentumMatrix(Model& model, const Math::VectorNd& q, Math::MatrixNd& A, bool update_kinematics = true);

/**
 * @brief Calculates the centroidal momentum matrix, \f$ A(q) \f$ for a model. The centroidal momentum
 * matrix is a \f$ 6 \times N \f$ matrix such that the 6dof centroidal momentum vector is computed by,
 * \f[
 *  h = A(q) \dot{q}
 * \f]
 *
 * @note It is crucial that the \f$ A \f$ matrix be all zeros when this function is called, as the elements will
 * be added to. To be sure, call A.setZero() before calling this function.
 *
 * @param model RDL model
 * @param q Vector of joint positions
 * @param A 6 by N matrix where the result will be stored
 * @param com Position of the center of mass in world coordinates
 * @param update_kinematics If true, calculates kinematic parameters
 */
void calcCentroidalMomentumMatrix(Model& model, const Math::VectorNd& q, Math::MatrixNd& A, Math::Vector3d com, bool update_kinematics = true);

/**
 * @brief Calculates the time derivative of the centroidal momentum matrix, i.e. the matrix computed by
 * RobotDynamics::Utils::calcCentroidalMomentumMatrix and stores it in the Adot argument
 *
 * @param model rdl model
 * @param q vector of position
 * @param qdot vector of velocities
 * @param Adot storage for centroidal momentum matrix dot
 * @param com position of the center of mass in world coordinates
 * @param com_velocity velocity of the center of mass in world coordinates
 * @param update_kinematics
 */
void calcCentroidalMomentumMatrixDot(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::MatrixNd& Adot, Math::Vector3d com,
                                     Math::Vector3d com_velocity, bool update_kinematics = true);

/**
 * @brief Calculates the time derivative of the centroidal momentum matrix, i.e. the matrix computed by
 * RobotDynamics::Utils::calcCentroidalMomentumMatrix and stores it in the Adot argument
 *
 * @param model
 * @param q
 * @param qdot
 * @param Adot
 * @param update_kinematics
 *
 * @note this function will compute the center of mass and its velocity. If you already have these quantities it will be more efficient to use the function
 * that takes these as arguments so it doesn't recompute them
 */
void calcCentroidalMomentumMatrixDot(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::MatrixNd& Adot, bool update_kinematics = true);

/**
 * @brief Calculates the centroidal momentum matrix and its time derivative, but in a more efficient way than calling the functions to compute them separately. If
 * you need both matrices it will be more efficient to use this function
 *
 * @param model rdl model
 * @param q vector of position
 * @param qdot vector of velocities
 * @param A storage for centroidal momentum matrix
 * @param Adot storage for centroidal momentum matrix dot
 * @param com position of the center of mass in world coordinates
 * @param com_velocity velocity of the center of mass in world coordinates
 * @param update_kinematics if true, kinematic variables will be computed. Defaults to true
 *
 */
void calcCentroidalMomentumMatrixAndMatrixDot(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::MatrixNd& A, Math::MatrixNd& Adot,
                                              Math::Vector3d com, Math::Vector3d com_velocity, bool update_kinematics = true);

/**
 * @brief Calculates the centroidal momentum matrix and its time derivative, but in a more efficient way than calling the functions to compute them separately. If
 * you need both matrices it will be more efficient to use this function.
 *
 * @param model rdl model
 * @param q vector of position
 * @param qdot vector of velocities
 * @param A storage for centroidal momentum matrix
 * @param Adot storage for centroidal momentum matrix dot
 * @param update_kinematics if true, kinematic variables will be computed. Defaults to true
 *
 * @note this function will compute the center of mass and its velocity. If you already have these quantities it will be more efficient to use the function
 * that takes these as arguments so it doesn't recompute them
 *
 */
void calcCentroidalMomentumMatrixAndMatrixDot(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::MatrixNd& A, Math::MatrixNd& Adot,
                                              bool update_kinematics = true);

}  // namespace Utils
}  // namespace RobotDynamics

#endif  // ifndef __RDL_UTILS_H__
