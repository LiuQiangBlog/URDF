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

#include <orwell/Controller.hpp>

namespace orwell
{

    template <int dof>
    class ControllerManager;

    template <int dof>
    class TorqueController;

    template <int dof>
    class VelocityController;

    /**
     * @brief invokes several controllers in parallel and combines their control command
     * @details
     *
     * @tparam dof number of degrees of freedom of the controlled system
     * @tparam DerivedController type of the controller, i.e. TorqueController, PositionController or VelocityController
     */
    template <int dof, template <int> class DerivedController>
    class ParallelController : public DerivedController<dof>, private DerivedController<dof>::SharedGroup
    {
      public:
        /**
         * @brief
         * @details loads configuration parameters
         *
         * @param interface interfaces the configuration server
         * @param name parameter namespace
         */
        ParallelController(const sackmesser::Interface::Ptr &interface, const std::string &name);

        /**
         * @brief
         */
        virtual ~ParallelController();

        /**
         * @brief computation of the control command depending
         * @details invokes computeCommand for all contained controllers
         * @return control command
         */
        Eigen::Vector<double, dof> computeCommand();

      private:
        /// controllers that are active in parallel
        std::vector<std::shared_ptr<DerivedController<dof>>> controllers_;
    };

    template <int dof>
    using ParallelTorqueController = ParallelController<7, TorqueController>;

    template <int dof>
    using ParallelVelocityController = ParallelController<7, VelocityController>;

}  // namespace orwell

#include <orwell/ParallelController.hxx>
//
#include <orwell/torque/TorqueController.hpp>
#include <orwell/velocity/VelocityController.hpp>

REGISTER_CLASS(orwell::TorqueController<7>, orwell::ParallelTorqueController<7>, "parallel_controller")
// REGISTER_CLASS(orwell::VelocityController<7>, orwell::ParallelVelocityController<7>, "parallel_controller")