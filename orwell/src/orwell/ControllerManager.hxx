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

#include <sackmesser/Configurations.hpp>
//
#include <orwell/ControllerManager.hpp>
//
#include <orwell/ParallelController.hpp>

namespace orwell
{

    template <int dof>
    ControllerManager<dof>::ControllerManager(const sackmesser::Interface::Ptr &interface, const typename RobotModel<dof>::Ptr &robot_model,
                                              const std::string &name)
      : TorqueController<dof>::SharedGroup(interface, name + "/torque_controller"),      //
        VelocityController<dof>::SharedGroup(interface, name + "/velocity_controller"),  //
        PositionController<dof>::SharedGroup(interface, name + "/position_controller"),  //
        robot_model_(robot_model)
    {
        TorqueController<dof>::SharedGroup::forEach(
          [this](typename TorqueController<dof>::Ptr controller) { controller->setRobotModel(robot_model_); });

        VelocityController<dof>::SharedGroup::forEach(
          [this](typename VelocityController<dof>::Ptr controller) { controller->setRobotModel(robot_model_); });
    }

    template <int dof>
    ControllerManager<dof>::~ControllerManager() = default;

    template <int dof>
    typename TorqueController<dof>::Ptr ControllerManager<dof>::getTorqueController(const std::string &name)
    {
        return TorqueController<dof>::SharedGroup::get(name);
    }

    template <int dof>
    typename VelocityController<dof>::Ptr ControllerManager<dof>::getVelocityController(const std::string &name)
    {
        return VelocityController<dof>::SharedGroup::get(name);
    }

    template <int dof>
    typename PositionController<dof>::Ptr ControllerManager<dof>::getPositionController(const std::string &name)
    {
        return PositionController<dof>::SharedGroup::get(name);
    }

}  // namespace orwell