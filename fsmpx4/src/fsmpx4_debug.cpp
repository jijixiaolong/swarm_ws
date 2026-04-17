#include "fsmpx4.h"
#include "fsm_utils.h"

namespace fsmpx4
{

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
    msg.uav_position.x = s.position.x();
    msg.uav_position.y = s.position.y();
    msg.uav_position.z = s.position.z();
    msg.uav_velocity.x = s.velocity.x();
    msg.uav_velocity.y = s.velocity.y();
    msg.uav_velocity.z = s.velocity.z();
    msg.uav_angular_velocity.x = s.angular_velocity.x();
    msg.uav_angular_velocity.y = s.angular_velocity.y();
    msg.uav_angular_velocity.z = s.angular_velocity.z();
    msg.uav_hover_thrust = s.hover_thrust;
    {
        Eigen::Quaterniond q(s.rotation);
        q.normalize();
        msg.uav_attitude.w = q.w();
        msg.uav_attitude.x = q.x();
        msg.uav_attitude.y = q.y();
        msg.uav_attitude.z = q.z();
    }

    msg.cmd_position.x = cmd_.position.x();
    msg.cmd_position.y = cmd_.position.y();
    msg.cmd_position.z = cmd_.position.z();
    msg.cmd_velocity.x = cmd_.velocity.x();
    msg.cmd_velocity.y = cmd_.velocity.y();
    msg.cmd_velocity.z = cmd_.velocity.z();
    msg.cmd_acceleration.x = cmd_.acceleration.x();
    msg.cmd_acceleration.y = cmd_.acceleration.y();
    msg.cmd_acceleration.z = cmd_.acceleration.z();
    msg.cmd_b1d.x = cmd_.b1d.x();
    msg.cmd_b1d.y = cmd_.b1d.y();
    msg.cmd_b1d.z = cmd_.b1d.z();
    msg.cmd_yaw = cmd_.yaw_desired;

    msg.control_thrust_vector.x = output_.thrust_vector.x();
    msg.control_thrust_vector.y = output_.thrust_vector.y();
    msg.control_thrust_vector.z = output_.thrust_vector.z();
    msg.control_thrust = output_.thrust;
    msg.control_acceleration.x = output_.A.x();
    msg.control_acceleration.y = output_.A.y();
    msg.control_acceleration.z = output_.A.z();

    const Eigen::Quaterniond q = output_.qd.normalized();
    msg.control_attitude.w = q.w();
    msg.control_attitude.x = q.x();
    msg.control_attitude.y = q.y();
    msg.control_attitude.z = q.z();
    msg.control_valid = output_.valid;

    debug_pub_->publish(std::move(msg));
}

void FSMPX4::publishSwarmDebugMessage(
    const rclcpp::Time& stamp,
    const swarm_planner::control::SwarmPlannerCore::DebugState& debug)
{
    // 本函数在 executeState() 中调用，早于 process() 末尾的计数器递增。
    // 因此需要按"本轮执行完之后是否会触发 debug 发布"来判断，确保与 FSMDebug 同周期记录。
    const bool publish_this_cycle =
        (debug_publish_every_n_ <= 1) || ((debug_publish_counter_ + 1) >= debug_publish_every_n_);
    if (!publish_this_cycle)
    {
        return;
    }
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
    msg.payload_position_ned = toVector3Msg(debug.payload_position_ned);
    msg.payload_velocity_ned = toVector3Msg(debug.payload_velocity_ned);
    msg.payload_target_ned = toVector3Msg(debug.payload_target_ned);
    msg.desired_payload_velocity = toVector3Msg(debug.desired_payload_velocity);
    msg.prev_thrust_ned = toVector3Msg(debug.prev_thrust_ned);

    for (size_t i = 0; i < swarm_planner::kNumUavs; ++i)
    {
        msg.uav_positions_ned[i] = toVector3Msg(debug.uav_positions_ned[i]);
        msg.uav_velocities_ned[i] = toVector3Msg(debug.uav_velocities_ned[i]);
        msg.beta[i] = debug.beta[i];
        const swarm_planner::Quaternion& q = swarm_state_.uavs[i].attitude;
        msg.uav_attitudes[i].w = q.w();
        msg.uav_attitudes[i].x = q.x();
        msg.uav_attitudes[i].y = q.y();
        msg.uav_attitudes[i].z = q.z();
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

    msg.spring_force = toVector3Msg(debug.spring_force);
    msg.damping_force = toVector3Msg(debug.damping_force);
    msg.friction_force = toVector3Msg(debug.friction_force);
    msg.passive_force = toVector3Msg(debug.passive_force);
    msg.tracking_p_term = toVector3Msg(debug.tracking_p_term);
    msg.tracking_i_term = toVector3Msg(debug.tracking_i_term);
    msg.tracking_d_term = toVector3Msg(debug.tracking_d_term);
    msg.tracking_input = toVector3Msg(debug.tracking_input);
    msg.virtual_acceleration = toVector3Msg(debug.virtual_acceleration);
    msg.mapped_acceleration = toVector3Msg(debug.mapped_acceleration);
    msg.azimuth_stabilization_acceleration =
        toVector3Msg(debug.azimuth_stabilization_acceleration);
    msg.separation_acceleration = toVector3Msg(debug.separation_acceleration);
    msg.base_acceleration = toVector3Msg(debug.base_acceleration);
    msg.rope_direction_ned = toVector3Msg(debug.rope_direction_ned);
    msg.estimated_rope_force_ned = toVector3Msg(debug.estimated_rope_force_ned);
    msg.rope_compensation_acceleration = toVector3Msg(debug.rope_compensation_acceleration);
    msg.unclipped_desired_acceleration =
        toVector3Msg(debug.unclipped_desired_acceleration);
    msg.desired_acceleration = toVector3Msg(debug.desired_acceleration);
    msg.dt_used = debug.dt_used;
    msg.measured_rope_axis_velocity = debug.measured_rope_axis_velocity;
    msg.observer_input_rope_axis_acceleration = debug.observer_input_rope_axis_acceleration;
    msg.rope_axis_velocity_hat = debug.rope_axis_velocity_hat;
    msg.rope_axis_disturbance_hat = debug.rope_axis_disturbance_hat;
    msg.rope_axis_observer_error = debug.rope_axis_observer_error;
    msg.estimated_rope_tension_n = debug.estimated_rope_tension_n;
    msg.prev_thrust_valid = debug.prev_thrust_valid;
    msg.rope_observer_valid = debug.rope_observer_valid;
    swarm_debug_pub_->publish(std::move(msg));
}

void FSMPX4::logLoadedParams() const
{
    RCLCPP_INFO(get_logger(), "频率: %.0fHz 悬停: %.3f", params_.basic.ctrl_freq_max, params_.thr_map.hover_percentage);
    RCLCPP_INFO(get_logger(), "原点: lat=%.8f lon=%.8f alt=%.2fm",
                params_.position.gps_origin_latitude_deg,
                params_.position.gps_origin_longitude_deg,
                params_.position.gps_origin_altitude_m);
    RCLCPP_INFO(get_logger(), "起飞: h=%.1fm v=%.1fm/s idle=%.1fs timeout=%.0fs",
                params_.takeoff.target_height_m, params_.takeoff.ascent_velocity_m_s,
                params_.takeoff.idle_duration_s, params_.takeoff.timeout_s);
    RCLCPP_INFO(get_logger(), "编队: self=%d mass=%.2f target=(%.2f,%.2f,%.2f)",
                params_.swarm.self_index, params_.swarm.mass,
                params_.swarm.target_ned.x(), params_.swarm.target_ned.y(), params_.swarm.target_ned.z());
    RCLCPP_INFO(get_logger(), "载荷: global=%s local=%s",
                params_.swarm.payload_global_position_topic.c_str(),
                params_.swarm.payload_local_position_topic.c_str());
    RCLCPP_INFO(get_logger(), "增益: Kp=(%.2f,%.2f,%.2f) Kv=(%.2f,%.2f,%.2f)",
                params_.gains.Kp_x, params_.gains.Kp_y, params_.gains.Kp_z,
                params_.gains.Kv_x, params_.gains.Kv_y, params_.gains.Kv_z);
    RCLCPP_INFO(get_logger(), "推力: [%.3f, %.3f] mass=%.2fkg", params_.limits.min_thrust, params_.limits.max_thrust, params_.physical.mass);
}

}  // namespace fsmpx4
