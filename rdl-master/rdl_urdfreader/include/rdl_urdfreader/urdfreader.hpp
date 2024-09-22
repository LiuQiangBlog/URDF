// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_URDFREADER_H__
#define __RDL_URDFREADER_H__

#include <rclcpp/rclcpp.hpp>

#include <rdl_dynamics/Model.hpp>

/**
 * @page urdfreader_page Urdf Reader
 * All functions related to urdfreader are specified in the \ref
 * urdfreader_group "Urdf Reader Module".
 *
 * \defgroup urdfreader_group Urdf Reader
 * @{
 *
 * The urdfreader has been updated to include parsing of hydrodynamics parameters from an
 * urdf to produce a model with the appropriate hydrodynamic parameters for calculating
 * the dynamics equations including hydrodynamic effects.
 *
 * For each body that you want to have hydrodynamic effects, you will need to place the
 * following block in the links <link> block,
 *
 * @code{.xml}
 * <hydro>
        <drag>
            <linear_damping>-1. -2. -3. -4. -5. -6.</linear_damping>
            <quadratic_damping>-1. -2. -3. -4. -5. -6.</quadratic_damping>
        </drag>
        <buoyancy>
            <volume>0.1</volume>
            <origin xyz= "0.1101 0.121 -0.013"/>
        </buoyancy>
        <added_mass>
            1. 0 0 0 0 0
            0 183. 0 0 0 0
            0 0 184. 0 0 0
            0 0 0 2. 0 0
            0 0 0 0 44. 0
            0 0 0 0 0 45.
        </added_mass>
    </hydro>
 * @endcode
 *
 * There are essentially 3 blocks inside the hydro tags where all of the
 * information is gathered. Drag is an effect that is assumed to be quadratic
 * in the spatial velocity of each body, i.e. of the form a*v + b*v^2. To model this
 * we require 6 coefficients for the linear term and 6 coefficients for the quadratic
 * term for each body, contained within the <linear_damping> and <quadratic_damping> tags.
 *
 * For modelling the effects of added_mass, you will need to include each body's added
 * mass matrix in the <added_mass></added_mass> tag. Note that the added mass matrix is a
 * 6x6 inertia type matrix that should be positive definite.
 *
 * Buoyancy is the simplest of the three hydro effects modelled and simply requires a
 * volume(in cubic meters) and the x, y, and z coordinates of the center of buoyancy
 * for the body.
 *
 * @note The drag coefficients need to be negative!
 *
 * @note Any of these effects can be ignored by removing the respective block from the hydro
 * block.
 */

namespace RobotDynamics
{
namespace Urdf
{
namespace hydro
{
/**
 * @struct BodyHydroData
 * @brief Data struct for holding the hydro parameters for each body as they are
 * parsed out of the URDF
 */
struct BodyHydroData
{
    BodyHydroData() : hasHydro(false), volume(0.), centerOfBuoyancy(0., 0., 0.), linearDrag(6), quadraticDrag(6), dragCoefficient(0.)
    {
        addedMassMatrix.setZero();
        for (unsigned int i = 0; i < 6; i++)
        {
            linearDrag[i] = 0;
            quadraticDrag[i] = 0;
        }
    }

    bool hasHydro;                                      /**< True if the hydro block exists in a body's link block */
    double volume;                                      /**< Body volume in cubic meters */
    Math::Vector3d centerOfBuoyancy;                    /**< Coordinates of the center of buoyancy in body frame*/
    RobotDynamics::Math::SpatialMatrix addedMassMatrix; /**< For added mass inertial effects */
    std::vector<double> linearDrag;                     /**< 6 directional elements for the linear drag term */
    std::vector<double> quadraticDrag;                  /**< 6 directional elements for the quadratic drag term */
    Math::Vector3d ellipsoidMajorAxisDirection;
    double dragCoefficient; /**< Coefficient of drag for the body */
};

/**
 * @brief Parses an urdf and stores hydrodynamic data for each body in the data argument
 * @param model_xml_string Urdf contents
 * @param data Map of structs to hold the hydro data for each body
 * @param verbose True for a whole bunch of debug data to be printed to console
 * @return True if success, false otherwise
 */
bool parseHydrodynamicsParameters(const char* model_xml_string, std::map<std::string, hydro::BodyHydroData>& data, bool verbose = false);
}  // namespace hydro

/**
 * @brief Read urdf from file path
 * @param filename
 * @param model
 * @param verbose if true will print model information
 * @return True on success, false otherwise
 *
 * @note This function will deduce whether or not the robot has a floating base by checking the name of
 * the root link. If the name of the root link is "world", then it will have a floating base.
 */
bool urdfReadFromFile(const std::string& filename, ModelPtr model, JointType floating_base_rotation_joint_type = JointTypeSpherical, bool verbose = false);

/**
 * @brief Read urdf from string contents
 * @param model_xml_string
 * @param model
 * @param verbose if true will print model information
 * @return True on success, false otherwise
 *
 * @note This function will deduce whether or not the robot has a floating base by checking the name of
 * the root link. If the name of the root link is "world", then it will have a floating base.
 *
 */
bool urdfReadFromString(const std::string& model_xml_string, ModelPtr model, JointType floating_base_rotation_joint_type = JointTypeSpherical, bool verbose = false);

/**
 * @brief This will build a map of joint name to body name.
 * @param model_xml_string Urdf file contents
 * @param jointBodyMap Modified
 * @return
 *
 * @warning This will NOT give any information about a floating body/joint. The floating body will be
 * ignored since it's not moved by a joint called out in the urdf. Only joints/bodies in 'joint'/'link'
 * tags will be used to populate the map.
 */
bool parseJointBodyNameMapFromString(const std::string& model_xml_string, std::map<std::string, std::string>& jointBodyMap);

/**
 * @brief This will build vectors of joint name and body name pairs
 * @param model_xml_string Urdf file contents
 * @param joint_names Modified
 * @param body_names Modified
 * @return True if succuss, false otherwise
 *
 * @warning This will NOT give any information about a floating body/joint. The floating body will be
 * ignored since it's not moved by a joint called out in the urdf. Only joints/bodies in 'joint'/'link'
 * tags will be used to populate the map. Non fixed joint/body pairs will be ignored
 */
bool parseJointAndBodyNamesFromString(const std::string& model_xml_string, std::vector<std::string>& joint_names, std::vector<std::string>& body_names);

/**
 * @brief This will build a map of joint name to body name.
 * @param filename Filepath
 * @param jointBodyMap Modified
 * @return
 *
 * @warning This will NOT give any information about a floating body/joint. The floating body will be
 * ignored since it's not moved by a joint called out in the urdf. Only joints/bodies in 'joint'/'link'
 * tags will be used to populate the map.
 */
bool parseJointBodyNameMapFromFile(const std::string& filename, std::map<std::string, std::string>& jointBodyMap);

/**
 * @brief This will build a vector of joint indices in the same order as the list of joints
 * @param model RDL Model
 * @param body_names vector of body names. Order is important here. Recommended this comes from one of the parse joint and body names functions to ensure correct ordering
 * @param q_indices Modified. Vector of q indices populated
 * @return true on success, false otherwise
 *
 * @warning This will NOT give any information about a floating body/joint
 */
bool parseJointAndQIndex(const RobotDynamics::Model& model, const std::vector<std::string>& body_names, std::vector<unsigned int>& q_indices);

/**
 * @brief This will build a vector of joint indices in the same order as the list of joints
 * @param model_xml_string Urdf string
 * @param q_indices Modified. Vector of q indices populated
 * @return true on success, false otherwise
 *
 * @warning This will NOT give any information about a floating body/joint
 */
bool parseJointAndQIndex(const std::string& model_xml_string, std::vector<unsigned int>& q_indices);
}  // namespace Urdf
}  // namespace RobotDynamics

/* ___RDL_URDFREADER_H__ */
#endif  // ifndef __RDL_URDFREADER_H__
