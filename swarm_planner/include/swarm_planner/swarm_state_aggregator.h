#ifndef SWARM_PLANNER_STATE_AGGREGATOR_H_
#define SWARM_PLANNER_STATE_AGGREGATOR_H_

#include <string>

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "geo_utils.h"
#include "types.h"

namespace swarm_planner {

class SwarmStateAggregator
{
public:
    struct Config
    {
        double swarm_origin_latitude_deg{0.0};
        double swarm_origin_longitude_deg{0.0};
        double swarm_origin_altitude_m{0.0};
        double peer_timeout_s{0.5};
        double payload_timeout_s{0.5};
        double payload_vel_lpf_alpha{0.7};
        double payload_vel_max_m_s{5.0};
    };

    SwarmStateAggregator() = default;

    void configure(const Config& cfg);
    void handlePeerGlobalPosition(
        size_t idx,
        const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void handlePeerLocalPosition(
        size_t idx,
        const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void handlePayloadNavSat(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    bool getSnapshot(
        const rclcpp::Time& now,
        SwarmCmdSnapshot& snapshot,
        std::string& reason) const;

private:
    void updatePayloadGlobalPosition(
        double latitude_deg,
        double longitude_deg,
        double altitude_m,
        const rclcpp::Time& stamp);
    void updatePayloadPosition(const Vector3& pos, const rclcpp::Time& stamp);

    Config cfg_{};
    geo::GpsOrigin swarm_origin_;
    std::array<TimedData<Vector3>, kNumUavs> peer_positions_{};
    std::array<TimedData<Vector3>, kNumUavs> peer_velocities_{};
    TimedData<Vector3> payload_position_{};
    Vector3 payload_velocity_{Vector3::Zero()};
    bool payload_has_prev_{false};
};

}  // namespace swarm_planner

#endif  // SWARM_PLANNER_STATE_AGGREGATOR_H_
