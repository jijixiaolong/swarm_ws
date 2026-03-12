#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "swarm_planner/planner_types.h"
#include "swarm_planner/planner_utils.h"

namespace {

using swarm_planner::SwarmCmdSnapshot;
using swarm_planner::SwarmStateActions;
using swarm_planner::SwarmStateActionsConfig;
using swarm_planner::Vector3;

px4_msgs::msg::VehicleGlobalPosition::SharedPtr makeGlobal(
    const double lat,
    const double lon,
    const double alt)
{
    auto msg = std::make_shared<px4_msgs::msg::VehicleGlobalPosition>();
    msg->lat = lat;
    msg->lon = lon;
    msg->alt = static_cast<float>(alt);
    msg->lat_lon_valid = true;
    msg->alt_valid = true;
    return msg;
}

px4_msgs::msg::VehicleLocalPosition::SharedPtr makeLocal(
    const double vx,
    const double vy,
    const double vz)
{
    auto msg = std::make_shared<px4_msgs::msg::VehicleLocalPosition>();
    msg->vx = static_cast<float>(vx);
    msg->vy = static_cast<float>(vy);
    msg->vz = static_cast<float>(vz);
    msg->v_xy_valid = true;
    msg->v_z_valid = true;
    return msg;
}

sensor_msgs::msg::NavSatFix::SharedPtr makePayloadNavSat(
    const double latitude,
    const double longitude,
    const double altitude)
{
    auto msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
    msg->latitude = latitude;
    msg->longitude = longitude;
    msg->altitude = altitude;
    return msg;
}

void expectVectorNear(
    const Vector3& actual,
    const Vector3& expected,
    const double tol = 1e-6)
{
    EXPECT_NEAR(actual.x(), expected.x(), tol);
    EXPECT_NEAR(actual.y(), expected.y(), tol);
    EXPECT_NEAR(actual.z(), expected.z(), tol);
}

class RclcppFixture : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok())
        {
            int argc = 0;
            char** argv = nullptr;
            rclcpp::init(argc, argv);
        }
    }

    static void TearDownTestSuite()
    {
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
    }
};

TEST_F(RclcppFixture, FixedSwarmOriginProducesCommonNedFrame)
{
    SwarmStateActions state_actions;
    SwarmStateActionsConfig cfg;
    cfg.swarm_origin_latitude_deg = 30.0;
    cfg.swarm_origin_longitude_deg = 120.0;
    cfg.swarm_origin_altitude_m = 100.0;
    cfg.peer_timeout_s = 1.0;
    cfg.payload_timeout_s = 1.0;
    state_actions.configure(cfg);

    state_actions.handlePeerGlobalPosition(0, makeGlobal(30.00001, 120.00002, 103.0));
    state_actions.handlePeerGlobalPosition(1, makeGlobal(30.00002, 120.00001, 101.0));
    state_actions.handlePeerGlobalPosition(2, makeGlobal(29.99999, 120.00003, 99.0));
    state_actions.handlePeerLocalPosition(0, makeLocal(1.0, 2.0, 3.0));
    state_actions.handlePeerLocalPosition(1, makeLocal(0.0, 0.0, 0.0));
    state_actions.handlePeerLocalPosition(2, makeLocal(-1.0, -2.0, -3.0));
    state_actions.handlePayloadNavSat(makePayloadNavSat(30.00003, 120.00004, 97.0));

    SwarmCmdSnapshot snapshot;
    std::string reason;
    ASSERT_TRUE(state_actions.getSnapshot(rclcpp::Clock(RCL_ROS_TIME).now(), snapshot, reason))
        << reason;

    const swarm_planner::geo::GpsOrigin origin = [] {
        swarm_planner::geo::GpsOrigin value;
        value.set(30.0, 120.0, 100.0);
        return value;
    }();
    expectVectorNear(
        snapshot.peers[0].position,
        swarm_planner::geo::lla_to_ned(30.00001, 120.00002, 103.0, origin));
    expectVectorNear(snapshot.peers[0].velocity, Vector3(1.0, 2.0, 3.0));
    expectVectorNear(
        snapshot.payload.position,
        swarm_planner::geo::lla_to_ned(30.00003, 120.00004, 97.0, origin));
    expectVectorNear(snapshot.payload.velocity, Vector3::Zero());
}

TEST_F(RclcppFixture, PayloadNavSatUsesCommonGpsOrigin)
{
    SwarmStateActions state_actions;
    SwarmStateActionsConfig cfg;
    cfg.swarm_origin_latitude_deg = 47.397742;
    cfg.swarm_origin_longitude_deg = 8.545594;
    cfg.swarm_origin_altitude_m = 488.0;
    cfg.payload_timeout_s = 1.0;
    state_actions.configure(cfg);

    state_actions.handlePeerGlobalPosition(0, makeGlobal(47.397742, 8.545594, 488.0));
    state_actions.handlePeerGlobalPosition(1, makeGlobal(47.397742, 8.545594, 488.0));
    state_actions.handlePeerGlobalPosition(2, makeGlobal(47.397742, 8.545594, 488.0));
    state_actions.handlePeerLocalPosition(0, makeLocal(0.0, 0.0, 0.0));
    state_actions.handlePeerLocalPosition(1, makeLocal(0.0, 0.0, 0.0));
    state_actions.handlePeerLocalPosition(2, makeLocal(0.0, 0.0, 0.0));
    state_actions.handlePayloadNavSat(
        makePayloadNavSat(47.397742 + 1.0e-5, 8.545594 + 2.0e-5, 489.5));

    SwarmCmdSnapshot snapshot;
    std::string reason;
    ASSERT_TRUE(state_actions.getSnapshot(rclcpp::Clock(RCL_ROS_TIME).now(), snapshot, reason))
        << reason;

    swarm_planner::geo::GpsOrigin origin;
    origin.set(47.397742, 8.545594, 488.0);
    expectVectorNear(
        snapshot.payload.position,
        swarm_planner::geo::lla_to_ned(
            47.397742 + 1.0e-5,
            8.545594 + 2.0e-5,
            489.5,
            origin));
}

}  // namespace
