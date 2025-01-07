//
// Created by LiuQiang on 2025/1/7.
//
#include "Trajectory.h"
#include "Logging.h"

int main()
{
    Eigen::VectorXd waypoint(4);
    std::vector<Eigen::VectorXd> waypoints;

    waypoint << 1424.0, 984.999694824219, 2126.0, 0.0;
    waypoints.push_back(waypoint);
    waypoint << 1423.0, 985.000244140625, 2126.0, 0.0;
    waypoints.push_back(waypoint);

    Eigen::VectorXd max_velocities(4);
    max_velocities << 1.3, 0.67, 0.67, 0.5;
    Eigen::VectorXd max_accelerations(4);
    max_accelerations << 0.00249, 0.00249, 0.00249, 0.00249;

    auto path_maybe = Path::create(waypoints, 100.0);
    if (path_maybe.has_value())
    {
        LOG_INFO << "create path success.";
    }
    auto trajectory_maybe = Trajectory::create(*path_maybe, max_velocities, max_accelerations, 10.0);
    if (trajectory_maybe.has_value())
    {
        LOG_INFO << "create trajectory success.";
    }
    const Trajectory& trajectory = trajectory_maybe.value();
    LOG_INFO << trajectory.getDuration();

    // Test start matches
    LOG_INFO << trajectory.getPosition(0.0)[0];
    LOG_INFO << trajectory.getPosition(0.0)[1];
    LOG_INFO << trajectory.getPosition(0.0)[2];
    LOG_INFO << trajectory.getPosition(0.0)[3];

    // Test end matches
    LOG_INFO << trajectory.getPosition(trajectory.getDuration())[0];
    LOG_INFO << trajectory.getPosition(trajectory.getDuration())[1];
    LOG_INFO << trajectory.getPosition(trajectory.getDuration())[2];
    LOG_INFO << trajectory.getPosition(trajectory.getDuration())[3];

    // Start at rest and end at rest
    const double traj_duration = trajectory.getDuration();
    LOG_INFO << trajectory.getVelocity(0.0)[0];
    LOG_INFO << trajectory.getVelocity(traj_duration)[0];
    return 0;
}