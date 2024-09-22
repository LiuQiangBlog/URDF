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

#include <orwell/torque/CompliantController.hpp>

namespace orwell
{

    template <int dof>
    CompliantController<dof>::CompliantController(const sackmesser::Interface::Ptr &interface, const std::string &)
      : TorqueController<dof>(interface, "controller_manager/torque_controller/compliant_controller/")
    {}

    template <int dof>
    CompliantController<dof>::~CompliantController() = default;

    template <int dof>
    Eigen::Vector<double, dof> CompliantController<dof>::computeCommand()
    {
        return Eigen::Vector<double, dof>::Zero();
    }

}  // namespace orwell

REGISTER_CLASS(orwell::TorqueController<7>, orwell::CompliantController<7>, "compliant_controller")