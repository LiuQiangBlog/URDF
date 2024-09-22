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

#include <memory>
//
#include <orwell/RobotState.hpp>

namespace orwell
{

    /**
     * @brief base class for a RobotModel to interface different kinematics/dynamics libraries
     * @details
     *
     * @tparam dof number of degrees of freedom of the controlled system
     */
    template <int dof>
    class RobotModel
    {
      public:
        /**
         * @brief
         * @details
         */
        RobotModel() = default;

        /**
         * @brief
         * @details
         */
        virtual ~RobotModel() = default;

        /// typedef
        using Vector = typename RobotState<dof>::Vector;

        /**
         * @brief precompute robot specific quantities for the given RobotState
         * @details will be called by Controller::getControlCommand
         *
         * @param robot_state current RobotState
         */
        virtual void computeCartesianState(const orwell::RobotState<dof> &robot_state) = 0;

        /**
         * @brief interface the computation of the forward dynamics for the used kinematics/dynamics library
         * @details
         *
         * @param position robot joint position
         * @param velocity robot joint velocity
         * @param acceleration robot joint acceleration
         * @param gravity gravity value
         * @return robot joint torque
         */
        virtual Vector computeForwardDynamics(const Vector &position, const Vector &velocity, const Vector &acceleration, const double &gravity) = 0;

        /**
         * @brief access the Jacobian Matrix
         * @details since the RobotState is not passed to this function, it is expected that it is being computed during the call to
         * computeCartesianState
         * @return Jacobian martix
         */
        virtual const Eigen::Matrix<double, 6, dof> &getJacobian() const = 0;

      public:
        /// typedef
        using Ptr = std::shared_ptr<RobotModel>;
    };

}  // namespace orwell

#include <orwell/RobotModel.hxx>