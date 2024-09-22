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

#include <orwell/torque/TorqueController.hpp>

namespace orwell
{

    /**
     * @brief base class for an impedance controller derived from TorqueController
     * @details
     *
     * @tparam dof number of degrees of freedom of the controlled system
     * @tparam Reference reference target
     */
    template <int dof, class Reference>
    class CartesianImpedanceController : public TorqueReferenceController<dof, Reference>
    {
      public:
        /**
         * @brief loads configuration parameters
         * @details
         *
         * @param interface interfaces the configuration server
         * @param name parameter namespace
         */
        CartesianImpedanceController(const sackmesser::Interface::Ptr &interface, const std::string &name);

        /**
         * @brief
         * @details
         */
        virtual ~CartesianImpedanceController();

        /**
         * @brief implements the impedance control law
         * @details derived classes should call setPositionError, setVelocityError and setAccelerationError during computeStateError
         * @return joint torque control command
         */
        typename RobotState<dof>::Vector computeCommand();

      protected:
        /**
         * @brief set the value of position_error_
         * @details derived classes should call this function during computeStateError
         *
         * @param position_error the position error for the impedance control law
         */
        void setPositionError(const Eigen::Vector<double, 6> &position_error);

        /**
         * @brief set the value of velocity_error_
         * @details derived classes should call this function during computeStateError
         *
         * @param velocity_error the velocity error for the impedance control law
         */
        void setVelocityError(const Eigen::Vector<double, 6> &velocity_error);

        /**
         * @brief set the value of acceleration_error_
         * @details derived classes should call this function during computeStateError
         *
         * @param acceleration_error the acceleration error for the impedance control law
         */
        void setAccelerationError(const Eigen::Vector<double, 6> &acceleration_error);

        /**
         * @brief limit the end-effector wrench to the given wrench_limit_
         * @details
         *
         * @param wrench wrench computed by getDesiredWrench
         * @return wrench within the limits
         */
        Eigen::Vector<double, 6> limitWrench(const Eigen::Vector<double, 6> &wrench);

        /**
         * @brief computes the state error for the impedance control law
         * @details derived classes should call setPositionError, setVelocityError, setAccelerationError
         */
        virtual void computeStateError() = 0;

      private:
        /**
         * @brief computes the desired wrench from the state error
         * @details
         * @return desired wrench
         */
        Eigen::Vector<double, 6> getDesiredWrench();

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
            Eigen::Matrix<double, 6, 6> stiffness_;

            /// damping applied to the velocity error in the impedance control law
            /// loaded from configuration file
            Eigen::Matrix<double, 6, 6> damping_;

            /// inertia applied to the acceleration error in the impedance control law
            /// loaded from configuration file
            Eigen::Matrix<double, 6, 6> inertia_;

            /// end-effector wrench limits
            /// loaded from configuration file
            Eigen::Vector<double, 6> wrench_limits_;
        } config_;

        /// position error for the impedance control law, initialized as zero
        Eigen::Vector<double, 6> position_error_;

        /// velocity error for the impedance control law, initialized as zero
        Eigen::Vector<double, 6> velocity_error_;

        /// acceleration error for the impedance control law, initialized as zero
        Eigen::Vector<double, 6> acceleration_error_;
    };

}  // namespace orwell

#include <orwell/torque/CartesianImpedanceController.hxx>