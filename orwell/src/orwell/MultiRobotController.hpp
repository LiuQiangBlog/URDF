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

#include <array>
#include <mutex>
#include <thread>
//
#include <sackmesser/Configuration.hpp>
#include <sackmesser/Interface.hpp>
//
#include <orwell/RobotModel.hpp>
#include <orwell/RobotState.hpp>

namespace orwell
{

    template <int n, int dof, template <int> class DerivedController>
    class MultiRobotController
    {
      public:
        MultiRobotController(const sackmesser::Interface::Ptr &interface, const std::string &name, const RobotModel<n * dof>::Ptr &robot_model);

        virtual ~MultiRobotController();

      private:
        class SingleController : public DerivedController<dof>
        {
          public:
            SingleController(const MultiRobotController *multi_robot_controller, int index, const sackmesser::Interface::Ptr &interface,
                             const std::string &name);

          protected:
            typename RobotState<dof>::Vector computeCommand();

          private:
            int index_;

            const MultiRobotController *multi_robot_controller_;
        };

        class RobotModelInterface : public RobotModel<dof>
        {
          public:
            RobotModelInterface(const int &index, const RobotModel<n * dof>::Ptr &robot_model);

            void computeCartesianState(const orwell::RobotState<dof> &robot_state);

            typename RobotState<dof>::Vector computeForwardDynamics(const typename RobotState<dof>::Vector &position,
                                                                    const typename RobotState<dof>::Vector &velocity,
                                                                    const typename RobotState<dof>::Vector &acceleration, const double &gravity);

            const Eigen::Matrix<double, 6, dof> &getJacobian() const;

          private:
            int index_;

            RobotModel<n * dof>::Ptr robot_model_;
        };

      public:
        std::shared_ptr<SingleController> getController(int index);

        const typename RobotState<dof>::Vector &getCommand(int index) const;

      protected:
        void setCommand(int index, const typename RobotState<dof>::Vector &command);

      private:
        virtual typename RobotState<n * dof>::Vector computeCommand(const RobotState<n * dof> &state) = 0;

      public:
        void start();

        void stop();

        RobotModel<n * dof>::Ptr getRobotModel() const;

      protected:
      private:
        class Configuration : public sackmesser::Configuration
        {
          public:
            /**
             * @brief load configuration parameters
             * @return true if all parameters were found
             */
            bool load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server);

            /// command limits
            double control_frequency;
        } config_;

      private:
        RobotModel<n * dof>::Ptr robot_model_;

        std::array<std::shared_ptr<SingleController>, n> controllers_;

        std::array<typename RobotState<dof>::Vector, n> commands_;

        std::thread compute_thread_;

        mutable std::mutex compute_mutex_;

        std::atomic<bool> compute_thread_active_;
    };

}  // namespace orwell

#include <orwell/MultiRobotController.hxx>