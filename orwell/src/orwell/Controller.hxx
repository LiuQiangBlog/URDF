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
#include <orwell/Controller.hpp>

namespace orwell
{

    template <int dof>
    Controller<dof>::Controller(const sackmesser::Interface::Ptr &interface, const std::string &name)
    {
        config_ = interface->getConfigurations()->load<Configuration>(name);
    }

    template <int dof>
    bool Controller<dof>::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        std::array<double, 7> command_limits;

        if (!server->loadParameter<7>(ns + "/command_limits", &command_limits))
        {
            return false;
        }

        command_limits_ = Eigen::Vector<double, 7>(command_limits.data());

        return true;
    }

    template <int dof>
    typename RobotModel<dof>::Ptr Controller<dof>::getRobotModel()
    {
        return robot_model_;
    }

    template <int dof>
    void Controller<dof>::setRobotModel(const typename RobotModel<dof>::Ptr &robot_model)
    {
        robot_model_ = robot_model;
    }

    template <int dof>
    void Controller<dof>::setStoppingCriterion(const std::shared_ptr<StoppingCriterion<dof>> &stopping_criterion)
    {
        stopping_criterion_ = stopping_criterion;
    }

    template <int dof>
    bool Controller<dof>::isFinished(const RobotState<dof> &robot_state) const
    {
        if (!stopping_criterion_)
        {
            return false;
        }

        return stopping_criterion_->isFinished(robot_state);
    }

    template <int dof>
    template <class Derived>
    typename Derived::Ptr Controller<dof>::cast()
    {
        return std::dynamic_pointer_cast<Derived>(this->shared_from_this());
    }

    template <int dof>
    typename RobotState<dof>::Vector Controller<dof>::getControlCommand(const RobotState<dof> &robot_state)
    {
        robot_state_ = robot_state;

        this->getRobotModel()->computeCartesianState(robot_state);

        return limitCommand(computeCommand());
    }

    template <int dof>
    const RobotState<dof> &Controller<dof>::getRobotState() const
    {
        return robot_state_;
    }

    template <int dof>
    typename RobotState<dof>::Vector Controller<dof>::limitCommand(const typename RobotState<dof>::Vector &command)
    {
        return command.array().min(config_.command_limits_.array()).max(-config_.command_limits_.array());
    }

}  // namespace orwell