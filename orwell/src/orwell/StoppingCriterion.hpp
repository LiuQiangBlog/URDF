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

namespace orwell
{

    template <int dof>
    class RobotState;

    /**
     * @brief base class for a stopping criterion for a Controller
     * @details
     *
     * @tparam dof number of degrees of freedom of the controlled system
     */
    template <int dof>
    class StoppingCriterion
    {
      public:
        /**
         * @brief
         * @details
         */
        StoppingCriterion() = default;

        /**
         * @brief
         * @details
         */
        virtual ~StoppingCriterion() = default;

        /**
         * @brief implement this function in derived classes to check whether the stopping criterion has been reached
         * @details will be called during Controller::isFinished
         *
         * @param robot_state current RobotState
         * @return true if the stopping criterion has been reached
         */
        virtual bool isFinished(const RobotState<dof> &robot_state) const = 0;

      protected:
      private:
    };

}  // namespace orwell