#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <exception>

#include "param_loader.h"

namespace fsmpx4 {
namespace param_loader {

namespace {

std::vector<std::string> defaultUavNamespaces()
{
    return {"/px4_1", "/px4_2", "/px4_3"};
}

std::string normalizeNamespace(std::string ns)
{
    if (!ns.empty() && ns.front() != '/')
    {
        ns.insert(ns.begin(), '/');
    }
    return ns;
}

double clampMin(const double value, const double lower_bound)
{
    return std::max(value, lower_bound);
}

}  // namespace

FSMParams::FSMParams()
{
    gains.Kp_x = 2.5;
    gains.Kp_y = 2.5;
    gains.Kp_z = 4.0;
    gains.Kv_x = 3.0;
    gains.Kv_y = 3.0;
    gains.Kv_z = 4.5;
    gains.Kvi_x = 0.02;
    gains.Kvi_y = 0.02;
    gains.Kvi_z = 0.1;

    physical.mass = 2.0;
    physical.gravity = 9.81;

    limits.max_thrust = -0.1;
    limits.min_thrust = -0.9;

    thr_map.hover_percentage = 0.67;

    basic.ctrl_freq_max = 200.0;
    basic.use_integral = false;
    basic.use_fmu_manual_topic = false;
    basic.px4_ns = "";

    position.gps_origin_latitude_deg = 0.0;
    position.gps_origin_longitude_deg = 0.0;
    position.gps_origin_altitude_m = 0.0;

    takeoff.enabled = true;
    takeoff.idle_duration_s = 1.0;
    takeoff.target_height_m = 1.0;
    takeoff.ascent_velocity_m_s = 0.6;
    takeoff.timeout_s = 10.0;

    land.detected_ready_timeout_s = 2.0;
    land.command_resend_interval_s = 0.2;
    land.disarm_command_resend_interval_s = 0.2;
    land.disarm_command_hold_s = 1.0;

    vehicle_command.target_system_id = 1;
    vehicle_command.target_component_id = 1;
    vehicle_command.source_system_id = 1;
    vehicle_command.source_component_id = 1;

    swarm.uav_namespaces = defaultUavNamespaces();
    swarm.self_index = 0;
    swarm.mass = 2.0;
    swarm.target_ned = swarm_planner::Vector3(0.0, 0.0, -1.0);
    swarm.payload_global_position_topic = "/px4_4/fmu/out/vehicle_global_position";
    swarm.payload_local_position_topic = "/px4_4/fmu/out/vehicle_local_position";
    swarm.data_timeout_s = 0.5;
    swarm.rope_length_m = 1.0;
    swarm.rope_taut_fraction = 0.85;
}

bool FSMParams::load_from_ros_node(const std::shared_ptr<rclcpp::Node>& node)
{
    return load_params_from_node(*node, *this);
}

bool FSMParams::load_from_node(rclcpp::Node& node)
{
    return load_params_from_node(node, *this);
}

bool load_params_from_node(const std::shared_ptr<rclcpp::Node>& node, FSMParams& params)
{
    return load_params_from_node(*node, params);
}

bool load_params_from_node(rclcpp::Node& node, FSMParams& params)
{
    try {
        params.basic.ctrl_freq_max = clampMin(
            node.declare_parameter<double>("ctrl_freq_max", params.basic.ctrl_freq_max), 1.0);
        params.basic.use_integral = node.declare_parameter<bool>("control.use_integral", params.basic.use_integral);
        params.basic.use_fmu_manual_topic = node.declare_parameter<bool>("manual_control_topic", params.basic.use_fmu_manual_topic);
        params.basic.px4_ns =
            normalizeNamespace(node.declare_parameter<std::string>("px4_ns", params.basic.px4_ns));

        params.position.gps_origin_latitude_deg = node.declare_parameter<double>(
            "position.gps_origin_latitude_deg", params.position.gps_origin_latitude_deg);
        params.position.gps_origin_longitude_deg = node.declare_parameter<double>(
            "position.gps_origin_longitude_deg", params.position.gps_origin_longitude_deg);
        params.position.gps_origin_altitude_m = node.declare_parameter<double>(
            "position.gps_origin_altitude_m", params.position.gps_origin_altitude_m);

        params.takeoff.enabled = node.declare_parameter<bool>(
            "takeoff.enabled", params.takeoff.enabled);
        params.takeoff.idle_duration_s = clampMin(
            node.declare_parameter<double>("takeoff.idle_duration_s", params.takeoff.idle_duration_s), 0.0);
        params.takeoff.target_height_m = clampMin(
            node.declare_parameter<double>("takeoff.target_height_m", params.takeoff.target_height_m), 0.1);
        params.takeoff.ascent_velocity_m_s = clampMin(
            node.declare_parameter<double>("takeoff.ascent_velocity_m_s", params.takeoff.ascent_velocity_m_s), 0.05);
        params.takeoff.timeout_s = clampMin(
            node.declare_parameter<double>("takeoff.timeout_s", params.takeoff.timeout_s), 1.0);

        params.land.detected_ready_timeout_s = clampMin(
            node.declare_parameter<double>(
                "land.detected_ready_timeout_s", params.land.detected_ready_timeout_s),
            0.05);
        params.land.command_resend_interval_s = clampMin(
            node.declare_parameter<double>(
                "land.command_resend_interval_s", params.land.command_resend_interval_s),
            0.01);
        params.land.disarm_command_resend_interval_s = clampMin(
            node.declare_parameter<double>(
                "land.disarm_command_resend_interval_s",
                params.land.disarm_command_resend_interval_s),
            0.01);
        params.land.disarm_command_hold_s = clampMin(
            node.declare_parameter<double>(
                "land.disarm_command_hold_s", params.land.disarm_command_hold_s),
            0.0);

        params.vehicle_command.target_system_id = std::clamp(
            static_cast<int>(node.declare_parameter<int>(
                "vehicle_command.target_system_id", params.vehicle_command.target_system_id)),
            0,
            255);
        params.vehicle_command.target_component_id = std::clamp(
            static_cast<int>(node.declare_parameter<int>(
                "vehicle_command.target_component_id", params.vehicle_command.target_component_id)),
            0,
            255);
        params.vehicle_command.source_system_id = std::clamp(
            static_cast<int>(node.declare_parameter<int>(
                "vehicle_command.source_system_id", params.vehicle_command.source_system_id)),
            0,
            255);
        params.vehicle_command.source_component_id = std::clamp(
            static_cast<int>(node.declare_parameter<int>(
                "vehicle_command.source_component_id", params.vehicle_command.source_component_id)),
            0,
            255);

        params.thr_map.hover_percentage = std::clamp(
            node.declare_parameter<double>("thr_map.hover_percentage", params.thr_map.hover_percentage),
            0.0,
            1.0);

        params.gains.Kp_x = node.declare_parameter<double>("control.Kp_x", params.gains.Kp_x);
        params.gains.Kp_y = node.declare_parameter<double>("control.Kp_y", params.gains.Kp_y);
        params.gains.Kp_z = node.declare_parameter<double>("control.Kp_z", params.gains.Kp_z);
        params.gains.Kv_x = node.declare_parameter<double>("control.Kv_x", params.gains.Kv_x);
        params.gains.Kv_y = node.declare_parameter<double>("control.Kv_y", params.gains.Kv_y);
        params.gains.Kv_z = node.declare_parameter<double>("control.Kv_z", params.gains.Kv_z);
        params.gains.Kvi_x = node.declare_parameter<double>("control.Kvi_x", params.gains.Kvi_x);
        params.gains.Kvi_y = node.declare_parameter<double>("control.Kvi_y", params.gains.Kvi_y);
        params.gains.Kvi_z = node.declare_parameter<double>("control.Kvi_z", params.gains.Kvi_z);

        params.physical.mass = clampMin(
            node.declare_parameter<double>("control.mass", params.physical.mass), 1e-6);
        params.physical.gravity = clampMin(
            node.declare_parameter<double>("control.gravity", params.physical.gravity), 0.0);

        params.limits.max_thrust = node.declare_parameter<double>("control.max_thrust", params.limits.max_thrust);
        params.limits.min_thrust = node.declare_parameter<double>("control.min_thrust", params.limits.min_thrust);

        const auto declared_uav_namespaces = node.declare_parameter<std::vector<std::string>>(
            "swarm.uav_namespaces", params.swarm.uav_namespaces);
        if (declared_uav_namespaces.size() == swarm_planner::kNumUavs)
        {
            params.swarm.uav_namespaces = declared_uav_namespaces;
            for (auto& ns : params.swarm.uav_namespaces)
            {
                ns = normalizeNamespace(ns);
            }
        }
        else
        {
            RCLCPP_WARN(
                node.get_logger(),
                "swarm.uav_namespaces size=%zu, expected=%zu; falling back to defaults",
                declared_uav_namespaces.size(),
                swarm_planner::kNumUavs);
            params.swarm.uav_namespaces = defaultUavNamespaces();
        }

        params.swarm.self_index = std::clamp(
            static_cast<int>(node.declare_parameter<int>("swarm.self_index", params.swarm.self_index)),
            0,
            static_cast<int>(swarm_planner::kNumUavs) - 1);
        params.swarm.mass = clampMin(
            node.declare_parameter<double>("swarm.mass", params.swarm.mass), 1e-6);
        params.swarm.target_ned.x() = node.declare_parameter<double>(
            "swarm.target_x_m", params.swarm.target_ned.x());
        params.swarm.target_ned.y() = node.declare_parameter<double>(
            "swarm.target_y_m", params.swarm.target_ned.y());
        params.swarm.target_ned.z() = node.declare_parameter<double>(
            "swarm.target_z_m", params.swarm.target_ned.z());
        params.swarm.payload_global_position_topic = normalizeNamespace(
            node.declare_parameter<std::string>(
                "swarm.payload_global_position_topic",
                params.swarm.payload_global_position_topic));
        params.swarm.payload_local_position_topic = normalizeNamespace(
            node.declare_parameter<std::string>(
                "swarm.payload_local_position_topic",
                params.swarm.payload_local_position_topic));
        params.swarm.data_timeout_s = clampMin(
            node.declare_parameter<double>("swarm.data_timeout_s", params.swarm.data_timeout_s),
            0.01);
        params.swarm.rope_length_m = clampMin(
            node.declare_parameter<double>("swarm.rope_length_m", params.swarm.rope_length_m),
            0.1);
        params.swarm.rope_taut_fraction = std::clamp(
            node.declare_parameter<double>("swarm.rope_taut_fraction", params.swarm.rope_taut_fraction),
            0.5, 1.0);

        params.swarm.core.gravity = clampMin(
            node.declare_parameter<double>("swarm.gravity", params.swarm.core.gravity), 0.0);
        params.swarm.core.h_u_m = clampMin(
            node.declare_parameter<double>("swarm.h_u_m", params.swarm.core.h_u_m), 1e-6);
        params.swarm.core.spring_k = clampMin(
            node.declare_parameter<double>("swarm.spring_k", params.swarm.core.spring_k), 0.0);
        params.swarm.core.damping_c1 = clampMin(
            node.declare_parameter<double>("swarm.damping_c1", params.swarm.core.damping_c1), 0.0);
        params.swarm.core.friction_c2 = clampMin(
            node.declare_parameter<double>("swarm.friction_c2", params.swarm.core.friction_c2), 0.0);
        params.swarm.core.vel_pid_kp = clampMin(
            node.declare_parameter<double>("swarm.vel_pid_kp", params.swarm.core.vel_pid_kp), 0.0);
        params.swarm.core.vel_pid_ki = clampMin(
            node.declare_parameter<double>("swarm.vel_pid_ki", params.swarm.core.vel_pid_ki), 0.0);
        params.swarm.core.vel_pid_kd = clampMin(
            node.declare_parameter<double>("swarm.vel_pid_kd", params.swarm.core.vel_pid_kd), 0.0);
        params.swarm.core.payload_kp = clampMin(
            node.declare_parameter<double>("swarm.payload_kp", params.swarm.core.payload_kp), 0.0);
        params.swarm.core.payload_ki = clampMin(
            node.declare_parameter<double>("swarm.payload_ki", params.swarm.core.payload_ki), 0.0);
        params.swarm.core.payload_mass = clampMin(
            node.declare_parameter<double>("swarm.payload_mass", params.swarm.core.payload_mass), 0.0);
        params.swarm.core.acc_norm_limit_m_s2 = clampMin(
            node.declare_parameter<double>(
                "swarm.acc_norm_limit_m_s2", params.swarm.core.acc_norm_limit_m_s2),
            0.0);
        params.swarm.core.integral_limit = clampMin(
            node.declare_parameter<double>("swarm.integral_limit", params.swarm.core.integral_limit),
            0.0);
        params.swarm.core.payload_integral_limit = clampMin(
            node.declare_parameter<double>(
                "swarm.payload_integral_limit", params.swarm.core.payload_integral_limit),
            0.0);
        params.swarm.core.payload_error_limit_xy_m = clampMin(
            node.declare_parameter<double>(
                "swarm.payload_error_limit_xy_m", params.swarm.core.payload_error_limit_xy_m),
            0.0);
        params.swarm.core.payload_error_limit_z_m = clampMin(
            node.declare_parameter<double>(
                "swarm.payload_error_limit_z_m", params.swarm.core.payload_error_limit_z_m),
            0.0);
        params.swarm.core.rest_lengths_override = node.declare_parameter<std::vector<double>>(
            "swarm.structure_reference.rest_lengths", params.swarm.core.rest_lengths_override);

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node.get_logger(), "参数加载失败: %s", e.what());
        return false;
    }
}

} // namespace param_loader
} // namespace fsmpx4
