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

#include <orwell/ParallelController.hpp>

namespace orwell
{

    template <int dof, template <int> class DerivedController>
    ParallelController<dof, DerivedController>::ParallelController(const sackmesser::Interface::Ptr &interface, const std::string &name)
      : TorqueController<dof>(interface, name), TorqueController<dof>::SharedGroup(interface, name + "/controller")
    {
        this->forEach([this](typename TorqueController<dof>::Ptr controller) { controllers_.push_back(controller); });
    }

    template <int dof, template <int> class DerivedController>
    ParallelController<dof, DerivedController>::~ParallelController() = default;

    template <int dof, template <int> class DerivedController>
    Eigen::Vector<double, dof> ParallelController<dof, DerivedController>::computeCommand()
    {
        Eigen::Vector<double, dof> torque = Eigen::Vector<double, dof>::Zero();

        for (const auto &controller : controllers_)
        {
            torque += controller->computeCommand();
        }

        return torque;
    }

}  // namespace orwell