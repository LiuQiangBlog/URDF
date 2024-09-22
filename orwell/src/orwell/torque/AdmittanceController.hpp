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
     * @brief base class for an admittance controller derived from TorqueController
     * @details
     *
     * @tparam dof number of degrees of freedom of the controlled system
     * @tparam Reference reference target
     */
    template <int dof, class Reference>
    class AdmittanceController : public TorqueReferenceController<dof, Reference>
    {
      public:
        /**
         * @brief loads configuration parameters
         * @details
         *
         * @param interface interfaces the configuration server
         * @param name parameter namespace
         */
        AdmittanceController(const sackmesser::Interface::Ptr &interface, const std::string &name);

        /**
         * @brief
         * @details
         */
        virtual ~AdmittanceController();

        /**
         * @brief computes the joint torque using forward dynamics
         * @details
         * @return joint torque command
         */
        typename RobotState<dof>::Vector computeCommand();

      protected:
        /**
         * @brief admittance control law
         * @details
         * @return desired joint acceleration
         */
        virtual typename RobotState<dof>::Vector computeDesiredJointAcceleration() = 0;
    };

}  // namespace orwell

#include <orwell/torque/AdmittanceController.hxx>