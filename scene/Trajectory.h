//
// Created by LiuQiang on 2025/1/5.
//

#ifndef SCENE_TRAJECTORY_H
#define SCENE_TRAJECTORY_H
#include <Eigen/Core>
#include <list>
#include <optional>

// The intermediate waypoints of the input path need to be blended so that the entire path is diffentiable.
// This constant defines the maximum deviation allowed at those intermediate waypoints, in radians for revolute joints,
// or meters for prismatic joints.
constexpr double DEFAULT_PATH_TOLERANCE = 0.1;

enum LimitType
{
    VELOCITY,
    ACCELERATION
};

const std::unordered_map<LimitType, std::string> LIMIT_TYPES = {{VELOCITY, "velocity"}, {ACCELERATION, "acceleration"}};
class PathSegment
{
public:
    PathSegment(double length = 0.0) : length_(length) {}
    virtual ~PathSegment() // is required for destructing derived classes
    {
    }
    double getLength() const
    {
        return length_;
    }
    virtual Eigen::VectorXd getConfig(double s) const = 0;
    virtual Eigen::VectorXd getTangent(double s) const = 0;
    virtual Eigen::VectorXd getCurvature(double s) const = 0;
    virtual std::list<double> getSwitchingPoints() const = 0;
    virtual PathSegment *clone() const = 0;

    double position_;

protected:
    double length_;
};

class Path
{
public:
    // Create a Path from a vector of waypoints and a maximum deviation to tolerate at the intermediate waypoints.
    // The algorithm needs max_deviation to be greater than zero so that the path is differentiable.
    static std::optional<Path> create(const std::vector<Eigen::VectorXd> &waypoint,
                                      double max_deviation = DEFAULT_PATH_TOLERANCE);

    // Copy constructor.
    Path(const Path &path);

    double getLength() const;
    Eigen::VectorXd getConfig(double s) const;
    Eigen::VectorXd getTangent(double s) const;
    Eigen::VectorXd getCurvature(double s) const;

    /** @brief Get the next switching point.
     *  @param[in] s Arc length traveled so far
     *  @param[out] discontinuity True if this switching point is a discontinuity
     *  @return arc length to the switching point
     **/
    double getNextSwitchingPoint(double s, bool &discontinuity) const;

    /// @brief Return a list of all switching points as a pair (arc length to switching point, discontinuity)
    std::list<std::pair<double, bool>> getSwitchingPoints() const;

private:
    // Default constructor private to prevent misuse. Use `create` instead to create a Path object.
    Path() = default;

    PathSegment *getPathSegment(double &s) const;

    double length_ = 0.0;
    std::list<std::pair<double, bool>> switching_points_;
    std::list<std::unique_ptr<PathSegment>> path_segments_;
};

class Trajectory
{
public:
    /// @brief Generates a time-optimal trajectory.
    /// @returns std::nullopt if the trajectory couldn't be parameterized.
    static std::optional<Trajectory> create(const Path &path,
                                            const Eigen::VectorXd &max_velocity,
                                            const Eigen::VectorXd &max_acceleration,
                                            double time_step = 0.001);

    /// @brief Returns the optimal duration of the trajectory
    double getDuration() const;

    /** @brief Return the position/configuration vector for a given point in time
     */
    Eigen::VectorXd getPosition(double time) const;
    /** @brief Return the velocity vector for a given point in time */
    Eigen::VectorXd getVelocity(double time) const;
    /** @brief Return the acceleration vector for a given point in time */
    Eigen::VectorXd getAcceleration(double time) const;

private:
    Trajectory(const Path &path,
               const Eigen::VectorXd &max_velocity,
               const Eigen::VectorXd &max_acceleration,
               double time_step);

    struct TrajectoryStep
    {
        TrajectoryStep() {}
        TrajectoryStep(double path_pos, double path_vel) : path_pos_(path_pos), path_vel_(path_vel) {}
        double path_pos_;
        double path_vel_;
        double time_;
    };

    bool getNextSwitchingPoint(double path_pos,
                               TrajectoryStep &next_switching_point,
                               double &before_acceleration,
                               double &after_acceleration);
    bool getNextAccelerationSwitchingPoint(double path_pos,
                                           TrajectoryStep &next_switching_point,
                                           double &before_acceleration,
                                           double &after_acceleration);
    bool getNextVelocitySwitchingPoint(double path_pos,
                                       TrajectoryStep &next_switching_point,
                                       double &before_acceleration,
                                       double &after_acceleration);
    bool integrateForward(std::list<TrajectoryStep> &trajectory, double acceleration);
    void integrateBackward(std::list<TrajectoryStep> &start_trajectory,
                           double path_pos,
                           double path_vel,
                           double acceleration);
    double getMinMaxPathAcceleration(double path_position, double path_velocity, bool max);
    double getMinMaxPhaseSlope(double path_position, double path_velocity, bool max);
    double getAccelerationMaxPathVelocity(double path_pos) const;
    double getVelocityMaxPathVelocity(double path_pos) const;
    double getAccelerationMaxPathVelocityDeriv(double path_pos);
    double getVelocityMaxPathVelocityDeriv(double path_pos);

    std::list<TrajectoryStep>::const_iterator getTrajectorySegment(double time) const;

    Path path_;
    Eigen::VectorXd max_velocity_;
    Eigen::VectorXd max_acceleration_;
    unsigned int joint_num_ = 0.0;
    bool valid_ = true;
    std::list<TrajectoryStep> trajectory_;
    std::list<TrajectoryStep> end_trajectory_; // non-empty only if the trajectory generation failed.

    double time_step_ = 0.0;

    mutable double cached_time_ = std::numeric_limits<double>::max();
    mutable std::list<TrajectoryStep>::const_iterator cached_trajectory_segment_;
};

#endif // SCENE_TRAJECTORY_H
