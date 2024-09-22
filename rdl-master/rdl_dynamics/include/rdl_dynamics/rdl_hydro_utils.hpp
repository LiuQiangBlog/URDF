//
// RDL - Robot Dynamics Library
// Modifications Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
//
// Licensed under the zlib license. See LICENSE for more details.
//

#ifndef __RDL_HYDRO_UTILS_H__
#define __RDL_HYDRO_UTILS_H__

/**
 * @file rdl_hydro_utils.h
 */

#include <string>

#include "rdl_dynamics/FramePoint.hpp"
#include "rdl_dynamics/Model.hpp"
#include "rdl_dynamics/rdl_mathutils.hpp"

namespace RobotDynamics
{
/** \page utils_page Hydrodynamic Utility Functions
 *
 * All utility functions are specified in the \ref utils_group "Utilities Module".
 *
 * \defgroup utils_group Hydro Utilities
 * @{
 *
 * Utility functions are those not necessarily required for kinematics/dynamics, but
 * provide utility by giving model information, calculating useful quantities such as
 * the position/velocity of the center of mass, etc.
 *
 */

struct Model;

/** \brief Namespace that contains optional helper functions */
namespace HydroUtils
{
/** @brief Computes the center of buoyancy and updates the center of buoyancy reference frame of the model
 *
 * @ingroup reference_frame
 *
 * @param model The model for which we want to compute the COB
 * @param q The current joint positions
 * @param euler_ypr The desired orientation of the center of buoyancy frame. Defaults to [0, 0, 0] in which case it would be aligned with world frame
 * @param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 *
 * @return a frame point that represents the position of the origin of the center of buoyancy frame expressed in world frame
 */
RobotDynamics::Math::FramePoint updateCenterOfBuoyancyFrame(Model& model, const Math::VectorNd& q, bool update_kinematics = true);

/** @brief Computes the center of buoyancy and updates the center of buoyancy reference frame of the model
 *
 * @ingroup reference_frame
 *
 * @param model The model for which we want to compute the COB
 * @param q The current joint positions
 * @param orientation The desired orientation of the center of buoyancy frame
 * @param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 *
 * @return a frame point that represents the position of the origin of the center of buoyancy frame expressed in world frame
 */
RobotDynamics::Math::FramePoint updateCenterOfBuoyancyFrame(Model& model, const Math::VectorNd& q, const Math::Quaternion& orientation, bool update_kinematics = true);

/** @brief Computes the center of buoyancy and updates the center of buoyancy reference frame of the model
 *
 * @ingroup reference_frame
 *
 * @param model The model for which we want to compute the COB
 * @param q The current joint positions
 * @param euler_ypr The desired orientation of the center of buoyancy frame
 * @param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 *
 * @return a frame point that represents the position of the origin of the center of buoyancy frame expressed in world frame
 */
RobotDynamics::Math::FramePoint updateCenterOfBuoyancyFrame(Model& model, const Math::VectorNd& q, const Math::Vector3d& euler_ypr, bool update_kinematics = true);

/** @brief Updates the center of buoyancy frame of the robot be at location p_com and aligned with world frame
 *
 * @ingroup reference_frame
 *
 * @param model The model for which we want to compute the COB
 * @param p_com The cob position in world frame
 */
void updateCenterOfBuoyancyFrame(Model& model, const Math::Vector3d& p_cob);

/** @brief Updates the center of buoyancy frame of the robot be at location p_com and aligned with world frame
 *
 * @ingroup reference_frame
 *
 * @param model The model for which we want to compute the COB
 * @param p_com The cob position in world frame
 * @param orientation Orientation of the com frame
 */
void updateCenterOfBuoyancyFrame(Model& model, const Math::Vector3d& p_cob, const Math::Quaternion& orientation);

/** @brief Updates the center of buoyancy frame of the robot be at location p_com and to have orientation defined by the yaw/pitch/roll angles in euler_ypr argument.
 * The euler_ypr argument is optional and if not provided the center of buoyancy reference frame will be aligned with world.
 *
 * @ingroup reference_frame
 *
 * @param model The model for which we want to compute the COB
 * @param p_com The cob position in world frame
 * @param euler_ypr The desired orientation of the center of buoyancy frame
 */
void updateCenterOfBuoyancyFrame(Model& model, const Math::Vector3d& p_cob, const Math::Vector3d& euler_ypr);

/** @brief Computes the Center of Buoyancy (COB) position.
 *
 * @param model The model for which we want to compute the COM
 * @param q The current joint positions
 * @param cob (output) location of the Center of Buoyancy of the model in world frame
 * @param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 */
void calcCenterOfBuoyancy(Model& model, const Math::VectorNd& q, Math::FramePoint& com, bool update_kinematics = true);

void calcCenterOfMassAndCenterOfBuoyancy(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, Math::Vector3d& com, Math::Vector3d& cob,
                                         Math::Vector3d& com_velocity, bool update_kinematics = true);

/** @brief Computes the wrench due to buoyancy experienced at the robots center of mass
 *
 * @param model
 * @param q
 * @param update_kinematics (optional) Defaults to true
 *
 * @note The reference frame the resulting force is expressed in is the center of mass reference frame owned by the RobotDynamics::Model object. This means that
 * you must first have the position/orientation of that reference frame updated prior to calling this function otherwise the frame it's expressed in is likely to not
 * be what you want.
 */
Math::SpatialForce calcBuoyancyWrenchOnCenterOfMass(Model& model, const Math::VectorNd& q, bool update_kinematics = true);

/** @brief Computes the wrench due to drag experienced at the robots center of mass
 *
 * @param model
 * @param q
 * @param qdot
 * @param update_kinematics (optional) Defaults to true
 *
 * @note The reference frame the resulting force is expressed in is the center of mass reference frame owned by the RobotDynamics::Model object. This means that
 * you must first have the position/orientation of that reference frame updated prior to calling this function otherwise the frame it's expressed in is likely to not
 * be what you want.
 */
Math::SpatialForce calcDragWrenchOnCenterOfMass(Model& model, const Math::VectorNd& q, const Math::VectorNd& qdot, bool update_kinematics = true);

/**
 * @brief Calculates the total volume of the model
 *
 * @param model The model used for the calculation
 * @param bodyId The ID of the first body in the desired subtree
 *
 * @return volume of subtree
 */
double calcSubtreeVolume(Model& model, const unsigned int bodyId);

/**
 * @brief Calculate the weight of a kinematic subtree in fluid with density RobotDynamics::Model::fluid_density
 *
 * @param model RDL Model
 * @param bodyId The ID of the first body in the desired subtree
 *
 * @return Weight in fluid
 */
double calcSubtreeWeightInFluid(Model& model, const unsigned int bodyId);

}  // namespace HydroUtils
}  // namespace RobotDynamics

/**
 * @}
 */

#endif  // ifndef __RDL_HYDRO_UTILS_H__
