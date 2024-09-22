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

#include <orwell/RobotState.hpp>

namespace orwell
{

    template <int dof>
    RobotState<dof>::RobotState() = default;

    template <int dof>
    RobotState<dof>::~RobotState() = default;

    template <int dof>
    void RobotState<dof>::setPosition(const Vector &position)
    {
        position_ = position;
    }

    template <int dof>
    void RobotState<dof>::setVelocity(const Vector &velocity)
    {
        velocity_ = velocity;
    }

    template <int dof>
    void RobotState<dof>::setTorque(const Vector &torque)
    {
        torque_ = torque;
    }

    template <int dof>
    void RobotState<dof>::setPose(const Pose &pose)
    {
        pose_ = pose;
    }

    template <int dof>
    const typename RobotState<dof>::Vector &RobotState<dof>::getPosition() const
    {
        return position_;
    }

    template <int dof>
    const typename RobotState<dof>::Vector &RobotState<dof>::getVelocity() const
    {
        return velocity_;
    }

    template <int dof>
    const typename RobotState<dof>::Vector &RobotState<dof>::getTorque() const
    {
        return torque_;
    }

    template <int dof>
    const typename RobotState<dof>::Pose &RobotState<dof>::getPose() const
    {
        return pose_;
    }

}  // namespace orwell