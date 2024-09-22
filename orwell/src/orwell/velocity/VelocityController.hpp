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

#include <atomic>
#include <memory>
//
#include <orwell/Controller.hpp>
#include <orwell/ReferenceController.hpp>
#include <orwell/RobotState.hpp>

namespace orwell
{
    /**
     * @brief Controller using velocity commands
     * @details
     *
     * @tparam dof number of degrees of freedom of the controlled system
     */
    template <int dof>
    class VelocityController : public Controller<dof>,
                               public sackmesser::FactoryClass<VelocityController<dof>, const sackmesser::Interface::Ptr &, const std::string &>
    {
      public:
        /**
         * @brief loads configuration parameters
         * @details
         *
         * @param interface interfaces the configuration server
         * @param name parameter namespace
         */
        VelocityController(const sackmesser::Interface::Ptr &interface, const std::string &name);

        /**
         * @brief
         * @details
         */
        virtual ~VelocityController() = default;

        /**
         * @brief will be invoked when calling ControllergetControlCommand
         * @details actual computation of the control command depending on the derived class
         * @return control command
         */
        virtual typename RobotState<dof>::Vector computeCommand() = 0;

        /// typedef
        using Ptr = std::shared_ptr<VelocityController>;

      private:
    };

    template <int dof, class Reference>
    using VelocityReferenceController = ReferenceController<Reference, VelocityController<dof>>;

}  // namespace orwell

#include <orwell/velocity/VelocityController.hxx>