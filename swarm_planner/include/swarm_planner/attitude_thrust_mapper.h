#ifndef SWARM_PLANNER_ATTITUDE_THRUST_MAPPER_H_
#define SWARM_PLANNER_ATTITUDE_THRUST_MAPPER_H_

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace swarm_planner {

struct AttitudeThrustResult
{
    Eigen::Quaterniond quaternion{Eigen::Quaterniond::Identity()};
    double normalized_thrust{0.0};  // PX4 convention: negative = upward
    bool valid{false};
};

inline AttitudeThrustResult mapAccelerationToAttitudeThrust(
    const Eigen::Vector3d& desired_acceleration,
    double yaw_desired,
    double mass,
    double gravity,
    double hover_thrust,
    const Eigen::Quaterniond& fallback_quaternion = Eigen::Quaterniond::Identity())
{
    AttitudeThrustResult result;
    result.quaternion = fallback_quaternion;
    result.quaternion.normalize();

    if (!std::isfinite(mass) || mass <= 0.0 ||
        !std::isfinite(gravity) || gravity <= 0.0 ||
        !std::isfinite(hover_thrust) || hover_thrust <= 0.0)
    {
        return result;
    }

    const Eigen::Vector3d A =
        -mass * gravity * Eigen::Vector3d::UnitZ() + mass * desired_acceleration;
    const double a_norm = A.norm();
    if (!std::isfinite(a_norm) || a_norm < 1e-6)
    {
        return result;
    }

    const Eigen::Vector3d b3c = -A / a_norm;

    Eigen::Vector3d b1ref(std::cos(yaw_desired), std::sin(yaw_desired), 0.0);
    if (!std::isfinite(b1ref.x()) || !std::isfinite(b1ref.y()) || !std::isfinite(b1ref.z()))
    {
        b1ref = Eigen::Vector3d::UnitX();
    }

    Eigen::Vector3d b1_proj = b1ref - b3c * (b3c.dot(b1ref));
    double proj_norm = b1_proj.norm();
    if (proj_norm < 1e-6)
    {
        const Eigen::Vector3d arbitrary =
            std::abs(b3c.dot(Eigen::Vector3d::UnitX())) < 0.9
                ? Eigen::Vector3d::UnitX()
                : Eigen::Vector3d::UnitY();
        b1_proj = arbitrary - b3c * (b3c.dot(arbitrary));
        proj_norm = std::max(b1_proj.norm(), 1e-6);
    }

    const Eigen::Vector3d b1c = b1_proj / proj_norm;
    const Eigen::Vector3d b2c = b3c.cross(b1c);

    Eigen::Matrix3d Rd;
    Rd.col(0) = b1c;
    Rd.col(1) = b2c;
    Rd.col(2) = b3c;

    result.quaternion = Eigen::Quaterniond(Rd);
    result.quaternion.normalize();
    result.normalized_thrust = -(a_norm / (mass * gravity)) * hover_thrust;
    result.valid = true;
    return result;
}

}  // namespace swarm_planner

#endif  // SWARM_PLANNER_ATTITUDE_THRUST_MAPPER_H_
