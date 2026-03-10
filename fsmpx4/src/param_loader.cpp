#include <rclcpp/rclcpp.hpp>
#include <exception>
#include "param_loader.h"

namespace fsmpx4 {
namespace param_loader {

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

    command.timeout_s = 0.5;
    command.acceleration_topic = "";

    vehicle_command.target_system_id = 1;
    vehicle_command.target_component_id = 1;
    vehicle_command.source_system_id = 1;
    vehicle_command.source_component_id = 1;
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
        params.basic.ctrl_freq_max = node.declare_parameter<double>("ctrl_freq_max", params.basic.ctrl_freq_max);
        params.basic.use_integral = node.declare_parameter<bool>("control.use_integral", params.basic.use_integral);
        params.basic.use_fmu_manual_topic = node.declare_parameter<bool>("manual_control_topic", params.basic.use_fmu_manual_topic);
        params.basic.px4_ns = node.declare_parameter<std::string>("px4_ns", params.basic.px4_ns);

        params.position.gps_origin_latitude_deg = node.declare_parameter<double>(
            "position.gps_origin_latitude_deg", params.position.gps_origin_latitude_deg);
        params.position.gps_origin_longitude_deg = node.declare_parameter<double>(
            "position.gps_origin_longitude_deg", params.position.gps_origin_longitude_deg);
        params.position.gps_origin_altitude_m = node.declare_parameter<double>(
            "position.gps_origin_altitude_m", params.position.gps_origin_altitude_m);

        params.takeoff.enabled = node.declare_parameter<bool>(
            "takeoff.enabled", params.takeoff.enabled);
        params.takeoff.idle_duration_s = node.declare_parameter<double>(
            "takeoff.idle_duration_s", params.takeoff.idle_duration_s);
        params.takeoff.target_height_m = node.declare_parameter<double>(
            "takeoff.target_height_m", params.takeoff.target_height_m);
        params.takeoff.ascent_velocity_m_s = node.declare_parameter<double>(
            "takeoff.ascent_velocity_m_s", params.takeoff.ascent_velocity_m_s);
        params.takeoff.timeout_s = node.declare_parameter<double>(
            "takeoff.timeout_s", params.takeoff.timeout_s);

        params.land.detected_ready_timeout_s = node.declare_parameter<double>(
            "land.detected_ready_timeout_s", params.land.detected_ready_timeout_s);
        params.land.command_resend_interval_s = node.declare_parameter<double>(
            "land.command_resend_interval_s", params.land.command_resend_interval_s);
        params.land.disarm_command_resend_interval_s = node.declare_parameter<double>(
            "land.disarm_command_resend_interval_s", params.land.disarm_command_resend_interval_s);
        params.land.disarm_command_hold_s = node.declare_parameter<double>(
            "land.disarm_command_hold_s", params.land.disarm_command_hold_s);

        params.command.timeout_s = node.declare_parameter<double>(
            "command.timeout_s", params.command.timeout_s);
        params.command.acceleration_topic = node.declare_parameter<std::string>(
            "command.acceleration_topic", params.command.acceleration_topic);

        params.vehicle_command.target_system_id = node.declare_parameter<int>(
            "vehicle_command.target_system_id", params.vehicle_command.target_system_id);
        params.vehicle_command.target_component_id = node.declare_parameter<int>(
            "vehicle_command.target_component_id", params.vehicle_command.target_component_id);
        params.vehicle_command.source_system_id = node.declare_parameter<int>(
            "vehicle_command.source_system_id", params.vehicle_command.source_system_id);
        params.vehicle_command.source_component_id = node.declare_parameter<int>(
            "vehicle_command.source_component_id", params.vehicle_command.source_component_id);

        params.thr_map.hover_percentage = node.declare_parameter<double>("thr_map.hover_percentage", params.thr_map.hover_percentage);

        params.gains.Kp_x = node.declare_parameter<double>("control.Kp_x", params.gains.Kp_x);
        params.gains.Kp_y = node.declare_parameter<double>("control.Kp_y", params.gains.Kp_y);
        params.gains.Kp_z = node.declare_parameter<double>("control.Kp_z", params.gains.Kp_z);
        params.gains.Kv_x = node.declare_parameter<double>("control.Kv_x", params.gains.Kv_x);
        params.gains.Kv_y = node.declare_parameter<double>("control.Kv_y", params.gains.Kv_y);
        params.gains.Kv_z = node.declare_parameter<double>("control.Kv_z", params.gains.Kv_z);
        params.gains.Kvi_x = node.declare_parameter<double>("control.Kvi_x", params.gains.Kvi_x);
        params.gains.Kvi_y = node.declare_parameter<double>("control.Kvi_y", params.gains.Kvi_y);
        params.gains.Kvi_z = node.declare_parameter<double>("control.Kvi_z", params.gains.Kvi_z);

        params.physical.mass = node.declare_parameter<double>("control.mass", params.physical.mass);
        params.physical.gravity = node.declare_parameter<double>("control.gravity", params.physical.gravity);

        params.limits.max_thrust = node.declare_parameter<double>("control.max_thrust", params.limits.max_thrust);
        params.limits.min_thrust = node.declare_parameter<double>("control.min_thrust", params.limits.min_thrust);

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node.get_logger(), "参数加载失败: %s", e.what());
        return false;
    }
}

} // namespace param_loader
} // namespace fsmpx4
