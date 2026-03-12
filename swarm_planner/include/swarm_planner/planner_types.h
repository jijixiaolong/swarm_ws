#ifndef SWARM_PLANNER_PLANNER_TYPES_H_
#define SWARM_PLANNER_PLANNER_TYPES_H_

#include <array>
#include <cstddef>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace swarm_planner {

using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;
using Quaternion = Eigen::Quaterniond;

constexpr std::size_t kNumUavs = 3;

template <typename T>
struct TimedData
{
    T value{};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};

    void update(const T& v, const rclcpp::Time& now)
    {
        value = v;
        stamp = now;
    }

    bool fresh(const rclcpp::Time& now, double timeout_s) const
    {
        return (now - stamp).seconds() <= timeout_s;
    }
};

struct SwarmPeerKinematics
{
    Vector3 position{Vector3::Zero()};
    Vector3 velocity{Vector3::Zero()};
};

struct SwarmPayloadState
{
    Vector3 position{Vector3::Zero()};
    Vector3 velocity{Vector3::Zero()};
};

struct SwarmCmdSnapshot
{
    std::array<SwarmPeerKinematics, kNumUavs> peers{};
    SwarmPayloadState payload{};
};

}  // namespace swarm_planner

#endif  // SWARM_PLANNER_PLANNER_TYPES_H_
