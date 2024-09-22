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
#include <orwell/position/PositionController.hpp>
#include <orwell/torque/TorqueController.hpp>
#include <orwell/velocity/VelocityController.hpp>

namespace orwell
{

    /**
     * @brief manages available controllers
     * @details
     *
     * @tparam dof numbers of degrees of freedom of the controlled system
     */
    template <int dof>
    class ControllerManager : private TorqueController<dof>::SharedGroup,
                              private VelocityController<dof>::SharedGroup,
                              private PositionController<dof>::SharedGroup
    {
      public:
        /**
         * @brief loads configuration parameters
         * @details
         *
         * @param interface interfaces the configuration server
         * @param robot_model RobotModel of the controlled system
         * @param name parameter namespace
         */
        ControllerManager(const sackmesser::Interface::Ptr &interface, const typename RobotModel<dof>::Ptr &robot_model,
                          const std::string &name = "controller_manager");

        /**
         * @brief
         */
        virtual ~ControllerManager();

        /**
         * @brief obtain a specific TorqueController
         * @details
         *
         * @param name name of the TorqueController
         * @return TorqueController
         */
        typename TorqueController<dof>::Ptr getTorqueController(const std::string &name);

        /**
         * @brief obtain a specific VelocityController
         * @details
         *
         * @param name name of the VelocityController
         * @return VelocityController
         */
        typename VelocityController<dof>::Ptr getVelocityController(const std::string &name);

        /**
         * @brief obtain a specific PositionController
         * @details
         *
         * @param name name of the PositionController
         * @return PositionController
         */
        typename PositionController<dof>::Ptr getPositionController(const std::string &name);

      protected:
      private:
        /// RobotModel
        typename RobotModel<dof>::Ptr robot_model_;
    };

}  // namespace orwell

#include <orwell/ControllerManager.hxx>