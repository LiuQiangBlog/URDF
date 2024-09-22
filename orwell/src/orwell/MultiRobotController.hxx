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
#include <orwell/MultiRobotController.hpp>

namespace orwell
{

    template <int n, int dof, template <int> class DerivedController>
    bool MultiRobotController<n, dof, DerivedController>::Configuration::load(const std::string &ns,
                                                                              const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "/control_frequency", &control_frequency, false);
    }

    template <int n, int dof, template <int> class DerivedController>
    MultiRobotController<n, dof, DerivedController>::MultiRobotController(const sackmesser::Interface::Ptr &interface, const std::string &name,
                                                                          const RobotModel<n * dof>::Ptr &robot_model)
      : robot_model_(robot_model)
    {
        config_ = interface->getConfigurations()->load<Configuration>(name);

        for (int i = 0; i < n; ++i)
        {
            controllers_[i] = std::make_shared<SingleController>(this, i, interface, name);

            controllers_[i]->setRobotModel(std::make_shared<RobotModelInterface>(i, robot_model));

            commands_[i] = RobotState<dof>::Vector::Zero();
        }

        compute_thread_active_ = false;
    }

    template <int n, int dof, template <int> class DerivedController>
    MultiRobotController<n, dof, DerivedController>::~MultiRobotController() = default;

    template <int n, int dof, template <int> class DerivedController>
    std::shared_ptr<typename MultiRobotController<n, dof, DerivedController>::SingleController>
      MultiRobotController<n, dof, DerivedController>::getController(int index)
    {
        assert(index < dof);

        return controllers_[index];
    }

    template <int n, int dof, template <int> class DerivedController>
    void MultiRobotController<n, dof, DerivedController>::setCommand(int index, const typename RobotState<dof>::Vector &command)
    {
        assert(index < dof);

        std::scoped_lock<std::mutex> lock(compute_mutex_);

        commands_[index] = command;
    }

    template <int n, int dof, template <int> class DerivedController>
    const typename RobotState<dof>::Vector &MultiRobotController<n, dof, DerivedController>::getCommand(int index) const
    {
        assert(index < dof);

        std::scoped_lock<std::mutex> lock(compute_mutex_);

        return commands_.at(index);
    }

    template <int n, int dof, template <int> class DerivedController>
    void MultiRobotController<n, dof, DerivedController>::start()
    {
        compute_thread_active_ = true;

        compute_thread_ = std::thread([this]() {
            while (compute_thread_active_.load())
            {
                RobotState<n * dof> state;

                Eigen::Vector<double, n * dof> positions;
                Eigen::Vector<double, n * dof> velocities;
                Eigen::Vector<double, n * dof> torques;

                for (int i = 0; i < n; ++i)
                {
                    positions.middleRows(i * dof, dof) = this->getController(i)->getRobotState().getPosition();
                    velocities.middleRows(i * dof, dof) = this->getController(i)->getRobotState().getVelocity();
                    torques.middleRows(i * dof, dof) = this->getController(i)->getRobotState().getTorque();
                }

                state.setPosition(positions);
                state.setVelocity(velocities);
                state.setTorque(torques);

                typename RobotState<n * dof>::Vector command = computeCommand(state);

                for (int i = 0; i < n; ++i)
                {
                    setCommand(i, command.middleRows(i * dof, dof));
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1.0 / config_.control_frequency)));
            }
        });
    }

    template <int n, int dof, template <int> class DerivedController>
    void MultiRobotController<n, dof, DerivedController>::stop()
    {
        compute_thread_active_.store(false);

        while (!compute_thread_.joinable())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }

        compute_thread_.join();
    }

    template <int n, int dof, template <int> class DerivedController>
    RobotModel<n * dof>::Ptr MultiRobotController<n, dof, DerivedController>::getRobotModel() const
    {
        return robot_model_;
    }

    template <int n, int dof, template <int> class DerivedController>
    MultiRobotController<n, dof, DerivedController>::SingleController::SingleController(const MultiRobotController *multi_robot_controller, int index,
                                                                                        const sackmesser::Interface::Ptr &interface,
                                                                                        const std::string &name)
      : DerivedController<dof>(interface, name), multi_robot_controller_(multi_robot_controller), index_(index)
    {}

    template <int n, int dof, template <int> class DerivedController>
    typename RobotState<dof>::Vector MultiRobotController<n, dof, DerivedController>::SingleController::computeCommand()
    {
        return multi_robot_controller_->getCommand(index_);
    }

    template <int n, int dof, template <int> class DerivedController>
    MultiRobotController<n, dof, DerivedController>::RobotModelInterface::RobotModelInterface(const int &index,
                                                                                              const RobotModel<n * dof>::Ptr &robot_model)
      : index_(index), robot_model_(robot_model)
    {}

    template <int n, int dof, template <int> class DerivedController>
    void MultiRobotController<n, dof, DerivedController>::RobotModelInterface::computeCartesianState(const orwell::RobotState<dof> & /*robot_state*/)
    {}

    template <int n, int dof, template <int> class DerivedController>
    typename RobotState<dof>::Vector MultiRobotController<n, dof, DerivedController>::RobotModelInterface::computeForwardDynamics(
      const typename RobotState<dof>::Vector & /*position*/, const typename RobotState<dof>::Vector & /*velocity*/,
      const typename RobotState<dof>::Vector & /*acceleration*/, const double & /*gravity*/)
    {
        return RobotState<dof>::Vector::Zero();
    }

    template <int n, int dof, template <int> class DerivedController>
    const Eigen::Matrix<double, 6, dof> &MultiRobotController<n, dof, DerivedController>::RobotModelInterface::getJacobian() const
    {}

}  // namespace orwell