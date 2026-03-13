#ifndef SWARM_PLANNER_SWARM_PLANNER_NODE_UTILS_H_
#define SWARM_PLANNER_SWARM_PLANNER_NODE_UTILS_H_

#include <array>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "swarm_planner/msg/swarm_planner_debug.hpp"
#include "swarm_planner/planner_core.h"
#include "swarm_planner/planner_types.h"

namespace swarm_planner {
namespace node_utils {

inline constexpr std::array<const char*, kNumUavs> kPeerNamespaces{"/px4_1", "/px4_2", "/px4_3"};

inline rclcpp::QoS makePx4Qos()
{
    return rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile);
}

inline rclcpp::QoS makeReliableQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::Volatile);
}

inline geometry_msgs::msg::Vector3 toVector3Msg(const Vector3& v)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
}

inline geometry_msgs::msg::Vector3Stamped makeVector3StampedMsg(
    const rclcpp::Time& stamp,
    const Vector3& v)
{
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "ned";
    msg.vector = toVector3Msg(v);
    return msg;
}

inline swarm_planner::msg::SwarmPlannerDebug makeDebugMsg(
    const rclcpp::Time& stamp,
    const control::SwarmPlannerCore::DebugState& debug)
{
    swarm_planner::msg::SwarmPlannerDebug msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "ned";
    msg.self_index = debug.self_index;
    msg.mass = debug.mass;
    msg.dt_input = debug.dt_input;
    msg.dt_used = debug.dt_used;
    msg.dt_valid_for_update = debug.dt_valid_for_update;
    msg.structure_locked = debug.structure_locked;
    msg.used_cfo = debug.used_cfo;
    msg.valid = debug.valid;
    msg.payload_position_ned = toVector3Msg(debug.payload_position_ned);
    msg.payload_velocity_ned = toVector3Msg(debug.payload_velocity_ned);
    msg.payload_target_ned = toVector3Msg(debug.payload_target_ned);
    msg.previous_thrust_vector = toVector3Msg(debug.previous_thrust_vector);

    for (size_t i = 0; i < kNumUavs; ++i)
    {
        msg.uav_positions_ned[i] = toVector3Msg(debug.uav_positions_ned[i]);
        msg.uav_velocities_ned[i] = toVector3Msg(debug.uav_velocities_ned[i]);
        msg.beta[i] = debug.beta[i];
    }

    for (size_t i = 0; i < debug.virtual_positions_ned.size(); ++i)
    {
        msg.virtual_positions_ned[i] = toVector3Msg(debug.virtual_positions_ned[i]);
        msg.virtual_velocities_ned[i] = toVector3Msg(debug.virtual_velocities_ned[i]);
    }

    for (size_t i = 0; i < debug.rest_lengths.size(); ++i)
    {
        msg.rest_lengths[i] = debug.rest_lengths[i];
    }

    msg.passive_force = toVector3Msg(debug.passive_force);
    msg.tracking_input = toVector3Msg(debug.tracking_input);
    msg.virtual_acceleration = toVector3Msg(debug.virtual_acceleration);
    msg.mapped_acceleration = toVector3Msg(debug.mapped_acceleration);
    msg.cfo_acceleration = toVector3Msg(debug.cfo_acceleration);
    msg.desired_acceleration = toVector3Msg(debug.desired_acceleration);
    return msg;
}

}  // namespace node_utils
}  // namespace swarm_planner

#endif  // SWARM_PLANNER_SWARM_PLANNER_NODE_UTILS_H_
