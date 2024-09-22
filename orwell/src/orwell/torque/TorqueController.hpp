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
#include <sackmesser/Configurations.hpp>
//
#include <orwell/Controller.hpp>
#include <orwell/ReferenceController.hpp>
#include <orwell/RobotState.hpp>

namespace orwell
{
    /**
     * @brief Controller using torque commands
     * @details
     *
     * @tparam dof number of degrees of freedom of the controlled system
     */
    template <int dof>
    class TorqueController : public Controller<dof>,
                             public sackmesser::FactoryClass<TorqueController<dof>, const sackmesser::Interface::Ptr &, const std::string &>
    {
      public:
        /**
         * @brief loads configuration parameters
         * @details
         *
         * @param interface interfaces the configuration server
         * @param name parameter namespace
         */
        TorqueController(const sackmesser::Interface::Ptr &interface, const std::string &name);

        /**
         * @brief
         * @details
         */
        virtual ~TorqueController() = default;

      public:
        /// typedef
        using Ptr = std::shared_ptr<TorqueController>;

      protected:
    };

    template <int dof, class Reference>
    using TorqueReferenceController = ReferenceController<Reference, TorqueController<dof>>;

}  // namespace orwell

#include <orwell/torque/TorqueController.hxx>