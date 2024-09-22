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
#include <orwell/torque/CartesianImpedanceController.hpp>

namespace orwell
{

    template <int dof, class Reference>
    CartesianImpedanceController<dof, Reference>::CartesianImpedanceController(const sackmesser::Interface::Ptr &interface, const std::string &name)
      : TorqueReferenceController<dof, Reference>(interface, name)
    {
        config_ = interface->getConfigurations()->load<Configuration>(name);

        position_error_ = Eigen::Vector<double, 6>::Zero();
        velocity_error_ = Eigen::Vector<double, 6>::Zero();
        acceleration_error_ = Eigen::Vector<double, 6>::Zero();
    }

    template <int dof, class Reference>
    bool CartesianImpedanceController<dof, Reference>::Configuration::load(const std::string &ns,
                                                                           const std::shared_ptr<sackmesser::Configurations> &server)
    {
        std::array<double, 6> stiffness, damping, inertia, wrench_limits;

        if (!(server->loadParameter<6>(ns + "/stiffness", &stiffness)  //
              && server->loadParameter<6>(ns + "/damping", &damping)   //
              && server->loadParameter<6>(ns + "/inertia", &inertia)   //
              && server->loadParameter<6>(ns + "/wrench_limits", &wrench_limits)))
        {
            return false;
        }

        stiffness_ = Eigen::DiagonalMatrix<double, 6, 6>(Eigen::Vector<double, 6>(stiffness.data()));
        damping_ = Eigen::DiagonalMatrix<double, 6, 6>(Eigen::Vector<double, 6>(damping.data()));
        inertia_ = Eigen::DiagonalMatrix<double, 6, 6>(Eigen::Vector<double, 6>(inertia.data()));
        wrench_limits_ = Eigen::Vector<double, 6>(wrench_limits.data());

        return true;
    }

    template <int dof, class Reference>
    CartesianImpedanceController<dof, Reference>::~CartesianImpedanceController() = default;

    template <int dof, class Reference>
    typename RobotState<dof>::Vector CartesianImpedanceController<dof, Reference>::computeCommand()
    {
        computeStateError();

        return this->getRobotModel()->getJacobian().transpose() * limitWrench(this->getDesiredWrench());
    }

    template <int dof, class Reference>
    Eigen::Vector<double, 6> CartesianImpedanceController<dof, Reference>::getDesiredWrench()
    {
        return config_.stiffness_ * position_error_ +  //
               config_.damping_ * velocity_error_ +    //
               config_.inertia_ * acceleration_error_;
    }

    template <int dof, class Reference>
    void CartesianImpedanceController<dof, Reference>::setPositionError(const Eigen::Vector<double, 6> &position_error)
    {
        position_error_ = position_error;
    }

    template <int dof, class Reference>
    void CartesianImpedanceController<dof, Reference>::setVelocityError(const Eigen::Vector<double, 6> &velocity_error)
    {
        velocity_error_ = velocity_error;
    }

    template <int dof, class Reference>
    void CartesianImpedanceController<dof, Reference>::setAccelerationError(const Eigen::Vector<double, 6> &acceleration_error)
    {
        acceleration_error_ = acceleration_error;
    }

    template <int dof, class Reference>
    Eigen::Vector<double, 6> CartesianImpedanceController<dof, Reference>::limitWrench(const Eigen::Vector<double, 6> &wrench)
    {
        return wrench.array().min(config_.wrench_limits_.array()).max(-config_.wrench_limits_.array());
    }

}  // namespace orwell