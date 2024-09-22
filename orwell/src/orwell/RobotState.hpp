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

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace orwell
{

    /**
     * @brief describes the RobotState
     * @details
     *
     * @tparam dof number of degrees of freedom of the controlled system
     */
    template <int dof>
    class RobotState
    {
      public:
        /// typedef
        using Vector = Eigen::Vector<double, dof>;

        /**
         * @brief describes the current robot pose
         * @details
         *
         */
        struct Pose
        {
            /// current robot position
            Eigen::Vector3d position;

            /// current robot orientation
            Eigen::Quaterniond orientation;
        };

        /**
         * @brief
         * @details
         */
        RobotState();

        /**
         * @brief
         * @details
         */
        virtual ~RobotState();

        /**
         * @brief set the joint position
         * @details
         *
         * @param position joint position
         */
        void setPosition(const Vector &position);

        /**
         * @brief set the joint velocity
         * @details
         *
         * @param velocity joint velocity
         */
        void setVelocity(const Vector &velocity);

        /**
         * @brief set the joint torque
         * @details
         *
         * @param torque joint torque
         */
        void setTorque(const Vector &torque);

        /**
         * @brief set the robot pose
         * @details
         *
         * @param pose robot pose
         */
        void setPose(const Pose &pose);

        /**
         * @brief get the joint position
         * @details
         * @return joint position
         */
        const Vector &getPosition() const;

        /**
         * @brief get the joint velocity
         * @details
         * @return joint velocity
         */
        const Vector &getVelocity() const;

        /**
         * @brief get the joint torque
         * @details
         * @return joint torque
         */
        const Vector &getTorque() const;

        /**
         * @brief get the robot pose
         * @details
         * @return robot pose
         */
        const Pose &getPose() const;

      private:
        ///
        Vector position_;

        ///
        Vector velocity_;

        ///
        Vector torque_;

        Pose pose_;
    };
}  // namespace orwell

#include <orwell/RobotState.hxx>