#include "swarm_planner/planner_utils.h"

#include <algorithm>
#include <cmath>

namespace swarm_planner {
namespace {

std::string resolveOutputTopic(rclcpp::Node& node, const std::string& px4_ns)
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

namespace geo {

Vector3 lla_to_ned(
    const double lat_deg,
    const double lon_deg,
    const double alt_m,
    const GpsOrigin& origin)
{
    const double lat_rad = lat_deg * kDegToRad;
    const double lon_rad = lon_deg * kDegToRad;
    return {
        (lat_rad - origin.lat_rad) * kEarthRadiusM,
        (lon_rad - origin.lon_rad) * kEarthRadiusM * std::cos(origin.lat_rad),
        origin.alt_m - alt_m};
}

}  // namespace geo

void SwarmStateActions::configure(const SwarmStateActionsConfig& cfg)
{
    cfg_ = cfg;
    swarm_origin_.set(
        cfg_.swarm_origin_latitude_deg,
        cfg_.swarm_origin_longitude_deg,
        cfg_.swarm_origin_altitude_m);

    for (auto& peer_position : peer_positions_)
    {
        peer_position = TimedData<Vector3>{};
    }

    for (auto& peer_velocity : peer_velocities_)
    {
        peer_velocity = TimedData<Vector3>{};
    }

    payload_position_ = TimedData<Vector3>{};
    payload_velocity_.setZero();
    payload_has_prev_ = false;
}

void SwarmStateActions::handlePeerGlobalPosition(
    const size_t idx,
    const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    if (idx >= kNumUavs || !msg->lat_lon_valid || !msg->alt_valid)
    {
        return;
    }

    peer_positions_[idx].update(
        geo::lla_to_ned(msg->lat, msg->lon, msg->alt, swarm_origin_),
        rclcpp::Clock(RCL_ROS_TIME).now());
}

void SwarmStateActions::handlePeerLocalPosition(
    const size_t idx,
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    if (idx >= kNumUavs || !msg->v_xy_valid || !msg->v_z_valid)
    {
        return;
    }

    peer_velocities_[idx].update(
        Vector3(msg->vx, msg->vy, msg->vz),
        rclcpp::Clock(RCL_ROS_TIME).now());
}

void SwarmStateActions::handlePayloadNavSat(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    if (!std::isfinite(msg->latitude) ||
        !std::isfinite(msg->longitude) ||
        !std::isfinite(msg->altitude))
    {
        return;
    }

    updatePayloadGlobalPosition(
        msg->latitude,
        msg->longitude,
        msg->altitude,
        rclcpp::Clock(RCL_ROS_TIME).now());
}

bool SwarmStateActions::getSnapshot(
    const rclcpp::Time& now,
    SwarmCmdSnapshot& snapshot,
    std::string& reason) const
{
    const double peer_timeout = std::max(0.01, cfg_.peer_timeout_s);
    for (size_t i = 0; i < kNumUavs; ++i)
    {
        if (!peer_positions_[i].fresh(now, peer_timeout))
        {
            reason = "peer position timeout idx=" + std::to_string(i);
            return false;
        }

        if (!peer_velocities_[i].fresh(now, peer_timeout))
        {
            reason = "peer velocity timeout idx=" + std::to_string(i);
            return false;
        }
    }

    if (!payload_position_.fresh(now, std::max(0.01, cfg_.payload_timeout_s)))
    {
        reason = "payload position timeout";
        return false;
    }

    for (size_t i = 0; i < kNumUavs; ++i)
    {
        snapshot.peers[i].position = peer_positions_[i].value;
        snapshot.peers[i].velocity = peer_velocities_[i].value;
    }

    snapshot.payload.position = payload_position_.value;
    snapshot.payload.velocity = payload_velocity_;
    reason.clear();
    return true;
}

void SwarmStateActions::updatePayloadPosition(
    const Vector3& pos,
    const rclcpp::Time& stamp)
{
    const double alpha = std::clamp(cfg_.payload_vel_lpf_alpha, 0.0, 1.0);
    const double vel_max = std::max(0.0, cfg_.payload_vel_max_m_s);

    if (payload_has_prev_)
    {
        const double dt = (stamp - payload_position_.stamp).seconds();
        if (dt > 1e-4)
        {
            Vector3 velocity_estimate =
                alpha * payload_velocity_ +
                (1.0 - alpha) * (pos - payload_position_.value) / dt;
            const double velocity_norm = velocity_estimate.norm();
            if (vel_max > 0.0 &&
                velocity_norm > vel_max)
            {
                velocity_estimate *= vel_max / velocity_norm;
            }
            payload_velocity_ = velocity_estimate;
        }
    }
    else
    {
        payload_velocity_.setZero();
        payload_has_prev_ = true;
    }

    payload_position_.update(pos, stamp);
}

void SwarmStateActions::updatePayloadGlobalPosition(
    const double latitude_deg,
    const double longitude_deg,
    const double altitude_m,
    const rclcpp::Time& stamp)
{
    updatePayloadPosition(
        geo::lla_to_ned(latitude_deg, longitude_deg, altitude_m, swarm_origin_),
        stamp);
}

PlannerNodeParams loadPlannerNodeParams(rclcpp::Node& node)
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

    params.state_actions.swarm_origin_latitude_deg = node.declare_parameter<double>(
        "swarm_origin.latitude_deg", params.state_actions.swarm_origin_latitude_deg);
    params.state_actions.swarm_origin_longitude_deg = node.declare_parameter<double>(
        "swarm_origin.longitude_deg", params.state_actions.swarm_origin_longitude_deg);
    params.state_actions.swarm_origin_altitude_m = node.declare_parameter<double>(
        "swarm_origin.altitude_m", params.state_actions.swarm_origin_altitude_m);
    params.state_actions.peer_timeout_s =
        node.declare_parameter<double>("peer_timeout_s", params.state_actions.peer_timeout_s);
    params.state_actions.payload_timeout_s =
        node.declare_parameter<double>("payload_timeout_s", params.state_actions.payload_timeout_s);
    params.state_actions.payload_vel_lpf_alpha = node.declare_parameter<double>(
        "payload_vel_lpf_alpha", params.state_actions.payload_vel_lpf_alpha);
    params.state_actions.payload_vel_max_m_s = node.declare_parameter<double>(
        "payload_vel_max_m_s", params.state_actions.payload_vel_max_m_s);

    params.payload_navsat_topic =
        node.declare_parameter<std::string>("payload_navsat_topic", params.payload_navsat_topic);
    params.output_topic = resolveOutputTopic(node, params.px4_ns);

    return params;
}

namespace control {
namespace planner_utils {

Vector3 computeDesiredPayloadVelocity(
    const SwarmPlannerCore::Input& input,
    const SwarmPlannerCore::Config& cfg)
{
    return -cfg.payload_kp * (input.payload_position_ned - input.payload_target_ned);
}

bool loadRestLengthMatrix(
    const std::vector<double>& rest_lengths_override,
    RestLengthMatrix& rest_lengths)
{
    for (int row = 0; row < SwarmPlannerCore::kNumNodes; ++row)
    {
        for (int col = 0; col < SwarmPlannerCore::kNumNodes; ++col)
        {
            const double length =
                rest_lengths_override[row * SwarmPlannerCore::kNumNodes + col];
            if (!std::isfinite(length) || length < 0.0)
            {
                return false;
            }

            rest_lengths(row, col) = length;
        }
    }

    return true;
}

bool validateRestLengthDiagonal(RestLengthMatrix& rest_lengths)
{
    for (int node = 0; node < SwarmPlannerCore::kNumNodes; ++node)
    {
        if (rest_lengths(node, node) != 0.0)
        {
            return false;
        }

        rest_lengths(node, node) = 0.0;
    }

    return true;
}

bool validateRestLengthSymmetry(const RestLengthMatrix& rest_lengths)
{
    for (int row = 0; row < SwarmPlannerCore::kNumNodes; ++row)
    {
        for (int col = row + 1; col < SwarmPlannerCore::kNumNodes; ++col)
        {
            if (rest_lengths(row, col) != rest_lengths(col, row))
            {
                return false;
            }
        }
    }

    return true;
}

Vector3 computeVelocityErrorDerivative(
    const Vector3& velocity_error,
    const Vector3& previous_velocity_error,
    const double dt,
    const bool previous_velocity_error_valid)
{
    if (dt <= 0.0 || !previous_velocity_error_valid)
    {
        return Vector3::Zero();
    }

    return (velocity_error - previous_velocity_error) / dt;
}

void clampIntegral(Vector3& velocity_integral, const double integral_limit)
{
    if (integral_limit <= 0.0)
    {
        return;
    }

    for (int axis = 0; axis < 3; ++axis)
    {
        velocity_integral(axis) = std::clamp(
            velocity_integral(axis), -integral_limit, integral_limit);
    }
}

bool cfoReadyForUpdate(
    const SwarmPlannerCore::Config& cfg,
    const double cable_length,
    const double dt)
{
    return cfg.cfo.enable && dt > 0.0 &&
           cable_length >= std::max(cfg.cfo.l_min_m, 0.0);
}

void flattenRestLengths(
    const RestLengthMatrix& rest_lengths,
    std::array<double, SwarmPlannerCore::kNumNodes * SwarmPlannerCore::kNumNodes>& flat_rest_lengths)
{
    for (int row = 0; row < SwarmPlannerCore::kNumNodes; ++row)
    {
        for (int col = 0; col < SwarmPlannerCore::kNumNodes; ++col)
        {
            flat_rest_lengths[row * SwarmPlannerCore::kNumNodes + col] =
                rest_lengths(row, col);
        }
    }
}

}  // namespace planner_utils
}  // namespace control
}  // namespace swarm_planner
