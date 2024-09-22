/*
    Copyright (c) Tobias Löw
    Written by Tobias Löw <https://tobiloew.ch>

    This file is part of orwell.

    gafro is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3 as
    published by the Free Software Foundation.

    gafro is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with gafro. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <orwell/ReferenceController.hpp>
#include <orwell/torque/TorqueController.hpp>

namespace orwell
{

    /**
     * @brief defines an impedance control law in joint space
     * @details takes a joint position as control reference
     *
     * @tparam dof number of degrees of freedom of the controlled system
     */
    template <int dof>
    class JointImpedanceController : public TorqueReferenceController<dof, Eigen::Vector<double, dof>>
    {
      public:
        /**
         * @brief loads configuration parameters
         * @details
         *
         * @param interface interfaces the configuration server
         * @param name parameter namespace
         */
        JointImpedanceController(const sackmesser::Interface::Ptr &interface, const std::string &name);

        /**
         * @brief
         * @details
         */
        virtual ~JointImpedanceController();

        /**
         * @brief computes the control command for the impedance control law in joint space
         * @details
         * @return joint torque command
         */
        Eigen::Vector<double, dof> computeCommand();

      private:
        /**
         * @brief limit the joint position error between the current and the reference position
         * @details
         *
         * @param position_difference difference between the current and the reference position
         * @return limited position difference
         */
        Eigen::Vector<double, dof> limit(const Eigen::Vector<double, dof> &position_difference);

      private:
        /**
         * @brief configuration parameters
         * @details
         */
        struct Configuration : public sackmesser::Configuration
        {
            /**
             * @brief load configuration parameters
             * @return true if all parameters were found
             */
            bool load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server);

            /// stiffness applied to the position error in the impedance control law
            /// loaded from configuration file
            Eigen::Matrix<double, dof, dof> stiffness_;

            /// damping applied to the velocity error in the impedance control law
            /// loaded from configuration file
            Eigen::Matrix<double, dof, dof> damping_;

            /// limit of the difference between the current and the reference position
            /// loaded from configuration file
            Eigen::Vector<double, dof> position_difference_limit_;
        } config_;
    };

}  // namespace orwell

#include <orwell/torque/JointImpedanceController.hxx>

REGISTER_CLASS(orwell::TorqueController<7>, orwell::JointImpedanceController<7>, "joint_impedance_controller")