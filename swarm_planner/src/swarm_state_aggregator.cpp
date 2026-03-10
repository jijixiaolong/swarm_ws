#include "swarm_planner/swarm_state_aggregator.h"

#include <algorithm>
#include <cmath>

namespace swarm_planner {

void SwarmStateAggregator::configure(const Config& cfg)
{
    cfg_ = cfg;
    swarm_origin_.set(cfg_.swarm_origin_latitude_deg,
                      cfg_.swarm_origin_longitude_deg,
                      cfg_.swarm_origin_altitude_m);
    RCLCPP_INFO(rclcpp::get_logger("SwarmStateAggregator"),
                "Swarm origin configured from parameters: lat=%.8f lon=%.8f alt=%.3f",
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

void SwarmStateAggregator::handlePeerGlobalPosition(
    size_t idx,
    const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    if (idx >= kNumUavs || !msg->lat_lon_valid || !msg->alt_valid)
        return;

    peer_positions_[idx].update(
        geo::lla_to_ned(msg->lat, msg->lon, msg->alt, swarm_origin_),
        rclcpp::Clock(RCL_ROS_TIME).now());
}

void SwarmStateAggregator::handlePeerLocalPosition(
    size_t idx,
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    if (idx >= kNumUavs || !msg->v_xy_valid || !msg->v_z_valid)
        return;

    peer_velocities_[idx].update(
        Vector3(msg->vx, msg->vy, msg->vz),
        rclcpp::Clock(RCL_ROS_TIME).now());
}

void SwarmStateAggregator::handlePayloadNavSat(
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

bool SwarmStateAggregator::getSnapshot(
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
void SwarmStateAggregator::updatePayloadPosition(const Vector3& pos, const rclcpp::Time& stamp)
{
    const double alpha = std::clamp(cfg_.payload_vel_lpf_alpha, 0.0, 1.0);
    const double vel_max = std::max(0.0, cfg_.payload_vel_max_m_s);

    if (payload_has_prev_)
    {
        const double dt = (stamp - payload_position_.stamp).seconds();
        if (dt > 1e-4)
        {
            Vector3 v_est = alpha * payload_velocity_ +
                            (1.0 - alpha) * (pos - payload_position_.value) / dt;
            const double vn = v_est.norm();
            if (vel_max > 0.0 && vn > vel_max && vn > 1e-6)
            {
                v_est *= vel_max / vn;
            }
            payload_velocity_ = v_est;
        }
    } else
    {
        payload_velocity_.setZero();
        payload_has_prev_ = true;
    }

    payload_position_.update(pos, stamp);
}

void SwarmStateAggregator::updatePayloadGlobalPosition(
    double latitude_deg,
    double longitude_deg,
    double altitude_m,
    const rclcpp::Time& stamp)
{
    updatePayloadPosition(
        geo::lla_to_ned(latitude_deg, longitude_deg, altitude_m, swarm_origin_),
        stamp);
}

}  // namespace swarm_planner
