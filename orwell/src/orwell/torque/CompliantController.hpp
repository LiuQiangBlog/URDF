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

#include <orwell/torque/TorqueController.hpp>

namespace orwell
{

    /**
     * @brief fully compliant controller for kinesthetic control
     * @details
     *
     * @tparam dof number of degrees of freedom of the controlled system
     */
    template <int dof>
    class CompliantController : public TorqueController<dof>
    {
      public:
        /**
         * @brief loads configuration parameters
         * @details
         *
         * @param interface interfaces the configuration server
         * @param name parameter namespace
         */
        CompliantController(const sackmesser::Interface::Ptr &interface, const std::string &name);

        /**
         * @brief
         * @details
         */
        virtual ~CompliantController();

        /**
         * @brief computes the control command, essentially only considers gravity compensation
         * @details
         * @return joint torque command
         */
        Eigen::Vector<double, dof> computeCommand();

      private:
    };

}  // namespace orwell

#include <orwell/torque/CompliantController.hxx>