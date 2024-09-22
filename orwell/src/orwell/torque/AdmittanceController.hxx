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
#include <orwell/torque/AdmittanceController.hpp>

namespace orwell
{

    template <int dof, class Reference>
    AdmittanceController<dof, Reference>::AdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name)
      : TorqueReferenceController<dof, Reference>(interface, name)
    {}

    template <int dof, class Reference>
    AdmittanceController<dof, Reference>::~AdmittanceController() = default;

    template <int dof, class Reference>
    typename RobotState<dof>::Vector AdmittanceController<dof, Reference>::computeCommand()
    {
        const typename RobotState<dof>::Vector &position = this->getRobotState().getPosition();
        const typename RobotState<dof>::Vector &velocity = this->getRobotState().getVelocity();
        typename RobotState<dof>::Vector acceleration = computeDesiredJointAcceleration();

        return this->getRobotModel()->computeForwardDynamics(position, velocity, acceleration, 0.0);
    }

}  // namespace orwell