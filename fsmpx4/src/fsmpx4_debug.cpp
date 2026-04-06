#include "fsmpx4.h"
#include "fsmpx4_utils.h"

namespace fsmpx4
{
namespace
{

geometry_msgs::msg::Vector3 toFsmpx4Vector3Msg(const types::Vector3& v)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
}

geometry_msgs::msg::Quaternion toQuaternionMsg(const Eigen::Quaterniond& q)
{
    geometry_msgs::msg::Quaternion msg;
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    return msg;
}

}  // namespace

void FSMPX4::publishDebugMessage(const rclcpp::Time& stamp)
{
    if (debug_pub_->get_subscription_count() == 0 &&
        debug_pub_->get_intra_process_subscription_count() == 0)
    {
        return;
    }

    fsmpx4::msg::FSMDebug msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = get_fully_qualified_name();
    msg.fsm_state = toDebugState(state_);
    msg.fsm_state_name = std::string(toString(state_));

    const auto& s = *current_state_;
    msg.uav_position = toFsmpx4Vector3Msg(s.position);
    msg.uav_velocity = toFsmpx4Vector3Msg(s.velocity);
    msg.uav_angular_velocity = toFsmpx4Vector3Msg(s.angular_velocity);
    msg.uav_hover_thrust = s.hover_thrust;
    {
        Eigen::Quaterniond q(s.rotation);
        q.normalize();
        msg.uav_attitude = toQuaternionMsg(q);
    }

    msg.cmd_position = toFsmpx4Vector3Msg(cmd_.position);
    msg.cmd_velocity = toFsmpx4Vector3Msg(cmd_.velocity);
    msg.cmd_acceleration = toFsmpx4Vector3Msg(cmd_.acceleration);
    msg.cmd_b1d = toFsmpx4Vector3Msg(cmd_.b1d);
    msg.cmd_yaw = cmd_.yaw_desired;

    msg.control_thrust_vector = toFsmpx4Vector3Msg(output_.thrust_vector);
    msg.control_thrust = output_.thrust;
    msg.control_moment = toFsmpx4Vector3Msg(output_.moment);
    msg.control_acceleration = toFsmpx4Vector3Msg(output_.A);

    const Eigen::Quaterniond q = output_.qd.normalized();
    msg.cmd_attitude = toQuaternionMsg(q);
    msg.control_attitude = toQuaternionMsg(q);
    msg.control_valid = output_.valid;

    debug_pub_->publish(std::move(msg));
}

void FSMPX4::publishSwarmDebugMessage(
    const rclcpp::Time& stamp,
    const swarm_planner::control::SwarmPlannerCore::DebugState& debug)
{
    if (swarm_debug_pub_->get_subscription_count() == 0 &&
        swarm_debug_pub_->get_intra_process_subscription_count() == 0)
    {
        return;
    }

    swarm_planner::msg::SwarmPlannerDebug msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "ned";
    msg.self_index = debug.self_index;
    msg.mass = debug.mass;
    msg.structure_locked = debug.structure_locked;
    msg.valid = debug.valid;
    msg.cmd_phase_name = std::string(toString(cmd_ctx_.phase));
    msg.payload_position_ned = fsmpx4::toVector3Msg(debug.payload_position_ned);
    msg.payload_velocity_ned = fsmpx4::toVector3Msg(debug.payload_velocity_ned);
    msg.payload_target_ned = fsmpx4::toVector3Msg(debug.payload_target_ned);

    for (size_t i = 0; i < swarm_planner::kNumUavs; ++i)
    {
        msg.uav_positions_ned[i] = fsmpx4::toVector3Msg(debug.uav_positions_ned[i]);
        msg.uav_velocities_ned[i] = fsmpx4::toVector3Msg(debug.uav_velocities_ned[i]);
        msg.beta[i] = debug.beta[i];
    }

    for (size_t i = 0; i < debug.virtual_positions_ned.size(); ++i)
    {
        msg.virtual_positions_ned[i] = fsmpx4::toVector3Msg(debug.virtual_positions_ned[i]);
        msg.virtual_velocities_ned[i] = fsmpx4::toVector3Msg(debug.virtual_velocities_ned[i]);
    }

    for (size_t i = 0; i < debug.rest_lengths.size(); ++i)
    {
    msg.rest_lengths[i] = debug.rest_lengths[i];
    }

    msg.passive_force = fsmpx4::toVector3Msg(debug.passive_force);
    msg.tracking_input = fsmpx4::toVector3Msg(debug.tracking_input);
    msg.virtual_acceleration = fsmpx4::toVector3Msg(debug.virtual_acceleration);
    msg.mapped_acceleration = fsmpx4::toVector3Msg(debug.mapped_acceleration);
    msg.desired_acceleration = fsmpx4::toVector3Msg(debug.desired_acceleration);
    swarm_debug_pub_->publish(std::move(msg));
}

void FSMPX4::logLoadedParams() const
{
    RCLCPP_INFO(
        get_logger(),
        "频率: %.0fHz 悬停: %.3f",
        params_.basic.ctrl_freq_max,
        params_.thr_map.hover_percentage);
    RCLCPP_INFO(
        get_logger(),
        "原点: lat=%.8f lon=%.8f alt=%.2fm",
        params_.position.gps_origin_latitude_deg,
        params_.position.gps_origin_longitude_deg,
        params_.position.gps_origin_altitude_m);
    RCLCPP_INFO(
        get_logger(),
        "起飞: h=%.1fm v=%.1fm/s idle=%.1fs timeout=%.0fs",
        params_.takeoff.target_height_m,
        params_.takeoff.ascent_velocity_m_s,
        params_.takeoff.idle_duration_s,
        params_.takeoff.timeout_s);
    RCLCPP_INFO(
        get_logger(),
        "编队: self=%d mass=%.2f target=(%.2f,%.2f,%.2f)",
        params_.swarm.self_index,
        params_.swarm.mass,
        params_.swarm.target_ned.x(),
        params_.swarm.target_ned.y(),
        params_.swarm.target_ned.z());
    RCLCPP_INFO(
        get_logger(),
        "载荷: global=%s local=%s",
        params_.swarm.payload_global_position_topic.c_str(),
        params_.swarm.payload_local_position_topic.c_str());
    RCLCPP_INFO(
        get_logger(),
        "增益: Kp=(%.2f,%.2f,%.2f) Kv=(%.2f,%.2f,%.2f)",
        params_.gains.Kp_x,
        params_.gains.Kp_y,
        params_.gains.Kp_z,
        params_.gains.Kv_x,
        params_.gains.Kv_y,
        params_.gains.Kv_z);
    RCLCPP_INFO(
        get_logger(),
        "推力: [%.3f, %.3f] mass=%.2fkg",
        params_.limits.min_thrust,
        params_.limits.max_thrust,
        params_.physical.mass);
}

}  // namespace fsmpx4
