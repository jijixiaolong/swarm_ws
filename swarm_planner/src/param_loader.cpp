#include "swarm_planner/param_loader.h"

namespace swarm_planner {
namespace param_loader {
namespace {

std::string resolve_output_topic(rclcpp::Node& node, const std::string& px4_ns)
{
    const std::string configured_output_topic =
        node.declare_parameter<std::string>("output_topic", "");
    if (!configured_output_topic.empty())
    {
        return configured_output_topic;
    }
    return px4_ns.empty() ? "/planner/desired_acceleration"
                          : px4_ns + "/planner/desired_acceleration";
}

}  // namespace

PlannerNodeParams load_from_node(rclcpp::Node& node)
{
    PlannerNodeParams params;

    params.px4_ns = node.declare_parameter<std::string>("px4_ns", params.px4_ns);
    params.self_index = node.declare_parameter<int>("self_index", params.self_index);
    params.mass = node.declare_parameter<double>("mass", params.mass);
    params.gravity = node.declare_parameter<double>("gravity", params.gravity);
    params.compute_hz = node.declare_parameter<double>("compute_hz", params.compute_hz);
    params.target_ned.x() = node.declare_parameter<double>("target_x_m", params.target_ned.x());
    params.target_ned.y() = node.declare_parameter<double>("target_y_m", params.target_ned.y());
    params.target_ned.z() = node.declare_parameter<double>("target_z_m", params.target_ned.z());

    params.core.gravity = params.gravity;
    params.core.h_u_m = node.declare_parameter<double>("h_u_m", params.core.h_u_m);
    params.core.spring_k = node.declare_parameter<double>("spring_k", params.core.spring_k);
    params.core.damping_c1 = node.declare_parameter<double>("damping_c1", params.core.damping_c1);
    params.core.friction_c2 =
        node.declare_parameter<double>("friction_c2", params.core.friction_c2);
    params.core.vel_pid_kp = node.declare_parameter<double>("vel_pid_kp", params.core.vel_pid_kp);
    params.core.vel_pid_ki = node.declare_parameter<double>("vel_pid_ki", params.core.vel_pid_ki);
    params.core.vel_pid_kd = node.declare_parameter<double>("vel_pid_kd", params.core.vel_pid_kd);
    params.core.payload_kp = node.declare_parameter<double>("payload_kp", params.core.payload_kp);
    params.core.acc_norm_limit_m_s2 =
        node.declare_parameter<double>("acc_norm_limit_m_s2", params.core.acc_norm_limit_m_s2);
    params.core.dt_min_s = node.declare_parameter<double>("dt_min_s", params.core.dt_min_s);
    params.core.dt_max_s = node.declare_parameter<double>("dt_max_s", params.core.dt_max_s);
    params.core.integral_limit =
        node.declare_parameter<double>("integral_limit", params.core.integral_limit);
    params.core.epsilon = node.declare_parameter<double>("epsilon", params.core.epsilon);
    params.core.rest_lengths_override = node.declare_parameter<std::vector<double>>(
        "structure_reference.rest_lengths", params.core.rest_lengths_override);
    params.core.cfo.enable = node.declare_parameter<bool>("cfo.enable", params.core.cfo.enable);
    params.core.cfo.l1 = node.declare_parameter<double>("cfo.l1", params.core.cfo.l1);
    params.core.cfo.l2 = node.declare_parameter<double>("cfo.l2", params.core.cfo.l2);
    params.core.cfo.phi = node.declare_parameter<double>("cfo.phi", params.core.cfo.phi);
    params.core.cfo.fmax_n =
        node.declare_parameter<double>("cfo.fmax_n", params.core.cfo.fmax_n);
    params.core.cfo.l_min_m =
        node.declare_parameter<double>("cfo.l_min_m", params.core.cfo.l_min_m);

    params.aggregator.swarm_origin_latitude_deg = node.declare_parameter<double>(
        "swarm_origin.latitude_deg", params.aggregator.swarm_origin_latitude_deg);
    params.aggregator.swarm_origin_longitude_deg = node.declare_parameter<double>(
        "swarm_origin.longitude_deg", params.aggregator.swarm_origin_longitude_deg);
    params.aggregator.swarm_origin_altitude_m = node.declare_parameter<double>(
        "swarm_origin.altitude_m", params.aggregator.swarm_origin_altitude_m);
    params.aggregator.peer_timeout_s =
        node.declare_parameter<double>("peer_timeout_s", params.aggregator.peer_timeout_s);
    params.aggregator.payload_timeout_s =
        node.declare_parameter<double>("payload_timeout_s", params.aggregator.payload_timeout_s);
    params.aggregator.payload_vel_lpf_alpha = node.declare_parameter<double>(
        "payload_vel_lpf_alpha", params.aggregator.payload_vel_lpf_alpha);
    params.aggregator.payload_vel_max_m_s = node.declare_parameter<double>(
        "payload_vel_max_m_s", params.aggregator.payload_vel_max_m_s);

    params.payload_navsat_topic =
        node.declare_parameter<std::string>("payload_navsat_topic", params.payload_navsat_topic);
    params.output_topic = resolve_output_topic(node, params.px4_ns);

    return params;
}

}  // namespace param_loader
}  // namespace swarm_planner
