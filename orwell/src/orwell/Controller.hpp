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
//
#include <sackmesser/FactoryClass.hpp>
#include <sackmesser/Interface.hpp>
//
#include <orwell/RobotModel.hpp>
#include <orwell/RobotState.hpp>
#include <orwell/StoppingCriterion.hpp>

namespace orwell
{

    /**
     * @brief base class for controller implementation
     * @details
     *
     * @tparam dof number of degrees of freedom of the controlled system
     */
    template <int dof>
    class Controller : public std::enable_shared_from_this<Controller<dof>>
    {
      public:
        /**
         * @brief loads configuration parameters
         * @details
         *
         * @param interface interfaces the configuration server
         * @param name parameter namespace
         */
        Controller(const sackmesser::Interface::Ptr &interface, const std::string &name);

        /**
         * @brief
         */
        virtual ~Controller() = default;

        /**
         * @brief general function to obtain control commands
         * @details
         *
         * @param robot_state current RobotState
         * @return control command as torque, position or velocity
         */
        typename RobotState<dof>::Vector getControlCommand(const RobotState<dof> &robot_state);

        /**
         * @brief set the RobotModel
         * @details since the RobotModel is implemented as a base class, this function allows the usage of different kinematics/dynamics libraries
         *
         * @param robot_model RobotModel the model of the robot to compute kinematics/dynamics
         */
        void setRobotModel(const typename RobotModel<dof>::Ptr &robot_model);

        /**
         * @brief (optional) StoppingCriterion
         * @details can be checked via the function isFinished
         *
         * @param stopping_criterion StoppingCriterion
         */
        void setStoppingCriterion(const std::shared_ptr<StoppingCriterion<dof>> &stopping_criterion);

        /**
         * @brief check if the StoppingCriterion is fulfilled
         * @details
         *
         * @param robot_state current RobotState
         * @return true if the StoppingCriterion is fulfilled
         */
        bool isFinished(const RobotState<dof> &robot_state) const;

        /**
         * @brief cast Controller to a derived class
         * @details expects the derived class to declare a typedef called Ptr
         *
         * @tparam Derived derived class type
         * @return shared pointer to derived class
         */
        template <class Derived>
        typename Derived::Ptr cast();

        /**
         * @brief proved access to the RobotState to derived classes
         * @details
         * @return RobotState
         */
        const RobotState<dof> &getRobotState() const;

      protected:
        /**
         * @brief will be invoked when calling getControlCommand
         * @details actual computation of the control command depending on the derived class
         * @return control command
         */
        virtual typename RobotState<dof>::Vector computeCommand() = 0;

        /**
         * @brief proved access to the RobotModel to derived classes
         * @details
         * @return RobotModel
         */
        typename RobotModel<dof>::Ptr getRobotModel();

      private:
        /**
         * @brief limit the control command
         * @details uses command_limits_
         *
         * @param RobotState::Vector control command
         * @return control command that is within the given limits
         */
        typename RobotState<dof>::Vector limitCommand(const typename RobotState<dof>::Vector &command);

      private:
        /// RobotModel
        typename RobotModel<dof>::Ptr robot_model_;

        /// RobotState
        RobotState<dof> robot_state_;

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

            /// command limits
            typename RobotState<dof>::Vector command_limits_;
        } config_;

        /// StoppingCriterion
        std::shared_ptr<StoppingCriterion<dof>> stopping_criterion_;

      private:
        // declare ParallelController as friend class such that it can access protected method computeCommand
        template <int, template <int> class>
        friend class ParallelController;
    };

}  // namespace orwell

#include <orwell/Controller.hxx>