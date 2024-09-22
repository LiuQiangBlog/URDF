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

#include <orwell/torque/JointImpedanceController.hpp>

namespace orwell
{

    template <int dof>
    JointImpedanceController<dof>::JointImpedanceController(const sackmesser::Interface::Ptr &interface, const std::string &name)
      : TorqueReferenceController<dof, Eigen::Vector<double, dof>>(interface, name)
    {
        config_ = interface->getConfigurations()->load<Configuration>(name);
    }

    template <int dof>
    bool JointImpedanceController<dof>::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        std::array<double, dof> stiffness, damping, position_difference_limit;

        if (!(server->loadParameter<dof>(ns + "/stiffness", &stiffness)  //
              && server->loadParameter<dof>(ns + "/damping", &damping)   //
              && server->loadParameter<dof>(ns + "/position_difference_limit", &position_difference_limit)))
        {
            return false;
        }

        stiffness_ = Eigen::DiagonalMatrix<double, dof, dof>(Eigen::Vector<double, dof>(stiffness.data()));
        damping_ = Eigen::DiagonalMatrix<double, dof, dof>(Eigen::Vector<double, dof>(damping.data()));
        position_difference_limit_ = Eigen::Vector<double, dof>(position_difference_limit.data());

        return true;
    }

    template <int dof>
    JointImpedanceController<dof>::~JointImpedanceController() = default;

    template <int dof>
    Eigen::Vector<double, dof> JointImpedanceController<dof>::computeCommand()
    {
        Eigen::Vector<double, dof> torque = Eigen::Vector<double, dof>::Zero();

        Eigen::Vector<double, dof> position_error = limit(this->getReference() - this->getRobotState().getPosition());

        torque = config_.stiffness_ * position_error + config_.damping_ * (-this->getRobotState().getVelocity());

        return torque;
    }

    template <int dof>
    Eigen::Vector<double, dof> JointImpedanceController<dof>::limit(const Eigen::Vector<double, dof> &position_difference)
    {
        return position_difference.array().min(config_.position_difference_limit_.array()).max(-config_.position_difference_limit_.array());
    }

}  // namespace orwell