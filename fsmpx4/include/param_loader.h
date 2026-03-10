#ifndef PARAM_LOADER_H_
#define PARAM_LOADER_H_

#include <memory>
#include <string>

namespace rclcpp {
class Node;
}

namespace fsmpx4 {
namespace param_loader {

struct FSMParams
{
    struct Gains {
        double Kp_x;
        double Kp_y;
        double Kp_z;
        double Kv_x;
        double Kv_y;
        double Kv_z;
        double Kvi_x;
        double Kvi_y;
        double Kvi_z;
    } gains;

    struct Physical {
        double mass;
        double gravity;
    } physical;

    struct Limits {
        double max_thrust;
        double min_thrust;
    } limits;

    struct ThrustMapping {
        double hover_percentage;
    } thr_map;

    struct Basic {
        double ctrl_freq_max;
        bool use_integral;
        bool use_fmu_manual_topic;
        std::string px4_ns;
    } basic;

    struct Position {
        double gps_origin_latitude_deg;
        double gps_origin_longitude_deg;
        double gps_origin_altitude_m;
    } position;

    struct Takeoff {
        bool enabled;
        double idle_duration_s;
        double target_height_m;
        double ascent_velocity_m_s;
        double timeout_s;
    } takeoff;

    struct Land {
        double detected_ready_timeout_s;
        double command_resend_interval_s;
        double disarm_command_resend_interval_s;
        double disarm_command_hold_s;
    } land;

    struct Command {
        double timeout_s;
        std::string acceleration_topic;
    } command;

    struct VehicleCommand {
        int target_system_id;
        int target_component_id;
        int source_system_id;
        int source_component_id;
    } vehicle_command;

    FSMParams();

    bool load_from_ros_node(const std::shared_ptr<rclcpp::Node>& node);
    bool load_from_node(rclcpp::Node& node);
};

bool load_params_from_node(const std::shared_ptr<rclcpp::Node>& node, FSMParams& params);
bool load_params_from_node(rclcpp::Node& node, FSMParams& params);

} // namespace param_loader
} // namespace fsmpx4

#endif  // PARAM_LOADER_H_
