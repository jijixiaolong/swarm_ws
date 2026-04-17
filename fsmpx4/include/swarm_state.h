#ifndef SWARM_STATE_H_
#define SWARM_STATE_H_

#include <array>

#include <rclcpp/rclcpp.hpp>

#include "swarm_planner/planner_types.h"

namespace fsmpx4
{

struct AgentState
{
    swarm_planner::Vector3    position{swarm_planner::Vector3::Zero()};
    swarm_planner::Vector3    velocity{swarm_planner::Vector3::Zero()};
    swarm_planner::Quaternion attitude{swarm_planner::Quaternion::Identity()};
    rclcpp::Time              stamp{0, 0, RCL_ROS_TIME};

    void updatePosition(const swarm_planner::Vector3& pos, const rclcpp::Time& now)
    {
        position = pos;
        stamp = now;
    }

    void updateVelocity(const swarm_planner::Vector3& vel, const rclcpp::Time& now)
    {
        velocity = vel;
        stamp = now;
    }

    void updateAttitude(const swarm_planner::Quaternion& att, const rclcpp::Time& now)
    {
        attitude = att;
        stamp = now;
    }

    void update(
        const swarm_planner::Vector3& pos,
        const swarm_planner::Vector3& vel,
        const swarm_planner::Quaternion& att,
        const rclcpp::Time& now)
    {
        position = pos;
        velocity = vel;
        attitude = att;
        stamp = now;
    }

    bool fresh(const rclcpp::Time& now, double timeout_s) const
    {
        return (now - stamp).seconds() <= timeout_s;
    }
};

struct SwarmState
{
    std::array<AgentState, swarm_planner::kNumUavs> uavs;
    AgentState payload;

    bool allFresh(const rclcpp::Time& now, double timeout_s) const
    {
        for (const auto& uav : uavs)
        {
            if (!uav.fresh(now, timeout_s)) return false;
        }
        return payload.fresh(now, timeout_s);
    }
};

}  // namespace fsmpx4

#endif  // SWARM_STATE_H_
