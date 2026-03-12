#ifndef SWARM_PLANNER_PLANNER_UTILS_H_
#define SWARM_PLANNER_PLANNER_UTILS_H_

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "planner_core.h"

namespace swarm_planner {

namespace geo {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kEarthRadiusM = 6378137.0;

struct GpsOrigin
{
    double lat_rad{0.0};
    double lon_rad{0.0};
    double alt_m{0.0};

    void set(double lat_deg, double lon_deg, double alt_msl)
    {
        lat_rad = lat_deg * kDegToRad;
        lon_rad = lon_deg * kDegToRad;
        alt_m = alt_msl;
    }
};

Vector3 lla_to_ned(double lat_deg, double lon_deg, double alt_m, const GpsOrigin& origin);

}  // namespace geo

struct SwarmStateActionsConfig
{
    double swarm_origin_latitude_deg{0.0};
    double swarm_origin_longitude_deg{0.0};
    double swarm_origin_altitude_m{0.0};
    double peer_timeout_s{0.5};
    double payload_timeout_s{0.5};
    double payload_vel_lpf_alpha{0.7};
    double payload_vel_max_m_s{5.0};
};

class SwarmStateActions
{
public:
    void configure(const SwarmStateActionsConfig& cfg);
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

    SwarmStateActionsConfig cfg_{};
    geo::GpsOrigin swarm_origin_{};
    std::array<TimedData<Vector3>, kNumUavs> peer_positions_{};
    std::array<TimedData<Vector3>, kNumUavs> peer_velocities_{};
    TimedData<Vector3> payload_position_{};
    Vector3 payload_velocity_{Vector3::Zero()};
    bool payload_has_prev_{false};
};

struct PlannerNodeParams
{
    std::string px4_ns{};
    int self_index{-1};
    double mass{2.0};
    double gravity{9.81};
    double compute_hz{200.0};
    Vector3 target_ned{Vector3(0.0, 0.0, -2.0)};
    std::string output_topic{};
    std::string payload_navsat_topic{"/payload/navsat"};
    control::SwarmPlannerCore::Config core{};
    SwarmStateActionsConfig state_actions{};
};

PlannerNodeParams loadPlannerNodeParams(rclcpp::Node& node);

namespace control {
namespace planner_utils {

 inline constexpr double kFixedComputeDtS = 1.0 / 200.0;

using RestLengthMatrix =
    Eigen::Matrix<double, SwarmPlannerCore::kNumNodes, SwarmPlannerCore::kNumNodes>;

Vector3 computeDesiredPayloadVelocity(
    const SwarmPlannerCore::Input& input,
    const SwarmPlannerCore::Config& cfg);
bool loadRestLengthMatrix(
    const std::vector<double>& rest_lengths_override,
    RestLengthMatrix& rest_lengths);
bool validateRestLengthDiagonal(RestLengthMatrix& rest_lengths);
bool validateRestLengthSymmetry(const RestLengthMatrix& rest_lengths);
Vector3 computeVelocityErrorDerivative(
    const Vector3& velocity_error,
    const Vector3& previous_velocity_error,
    double dt,
    bool previous_velocity_error_valid);
void clampIntegral(Vector3& velocity_integral, double integral_limit);
bool cfoReadyForUpdate(
    const SwarmPlannerCore::Config& cfg,
    double cable_length,
    double dt);
void flattenRestLengths(
    const RestLengthMatrix& rest_lengths,
    std::array<double, SwarmPlannerCore::kNumNodes * SwarmPlannerCore::kNumNodes>& flat_rest_lengths);

}  // namespace planner_utils
}  // namespace control
}  // namespace swarm_planner

#endif  // SWARM_PLANNER_PLANNER_UTILS_H_
