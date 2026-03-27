#include <cstdlib>
#include <filesystem>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include "fsmpx4.h"
#include "swarm_planner/geo_utils.h"
#include "swarm_planner/planner_core.h"

namespace {

using namespace std::chrono_literals;

constexpr double kOriginLatDeg = 30.0;
constexpr double kOriginLonDeg = 120.0;
constexpr double kOriginAltM = 10.0;
constexpr double kHoverThrust = 0.71;
constexpr double kPayloadLatDeg = kOriginLatDeg + 5.0e-6;
constexpr double kPayloadLonDeg = kOriginLonDeg + 5.0e-6;
constexpr double kPayloadAltM = kOriginAltM + 1.0;
constexpr double kPayloadVx = 0.2;
constexpr double kPayloadVy = -0.15;
constexpr double kPayloadVz = 0.05;

std::vector<double> restLengths()
{
    return {
        0.0, 0.98, 1.01, 0.55, 1.14,
        0.98, 0.0, 1.03, 0.60, 1.17,
        1.01, 1.03, 0.0, 0.59, 1.16,
        0.55, 0.60, 0.59, 0.0, 1.0,
        1.14, 1.17, 1.16, 1.0, 0.0};
}

rclcpp::QoS px4Qos()
{
    return rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile);
}

px4_msgs::msg::ManualControlSetpoint makeRc(double aux1, double aux2)
{
    px4_msgs::msg::ManualControlSetpoint msg{};
    msg.roll = 0.0f;
    msg.pitch = 0.0f;
    msg.yaw = 0.0f;
    msg.throttle = 0.0f;
    msg.aux1 = static_cast<float>(aux1);
    msg.aux2 = static_cast<float>(aux2);
    return msg;
}

px4_msgs::msg::VehicleAttitude makeAttitude()
{
    px4_msgs::msg::VehicleAttitude msg{};
    msg.q[0] = 1.0f;
    msg.q[1] = 0.0f;
    msg.q[2] = 0.0f;
    msg.q[3] = 0.0f;
    return msg;
}

px4_msgs::msg::VehicleGlobalPosition makeGlobalPosition(
    double lat,
    double lon,
    double alt)
{
    px4_msgs::msg::VehicleGlobalPosition msg{};
    msg.lat_lon_valid = true;
    msg.alt_valid = true;
    msg.lat = lat;
    msg.lon = lon;
    msg.alt = static_cast<float>(alt);
    return msg;
}

px4_msgs::msg::VehicleLocalPosition makeLocalPosition(
    double vx = 0.0,
    double vy = 0.0,
    double vz = 0.0,
    double ref_alt = kOriginAltM,
    double z = 0.0)
{
    px4_msgs::msg::VehicleLocalPosition msg{};
    msg.z_valid = true;
    msg.z_global = true;
    msg.ref_alt = static_cast<float>(ref_alt);
    msg.z = static_cast<float>(z);
    msg.v_xy_valid = true;
    msg.v_z_valid = true;
    msg.vx = static_cast<float>(vx);
    msg.vy = static_cast<float>(vy);
    msg.vz = static_cast<float>(vz);
    return msg;
}

swarm_planner::control::SwarmPlannerCore::Config makeSwarmConfig()
{
    swarm_planner::control::SwarmPlannerCore::Config cfg;
    cfg.gravity = 9.81;
    cfg.h_u_m = 1.0;
    cfg.spring_k = 3.0;
    cfg.damping_c1 = 1.2;
    cfg.friction_c2 = 0.3;
    cfg.vel_pid_kp = 1.2;
    cfg.vel_pid_ki = 0.3;
    cfg.vel_pid_kd = 0.1;
    cfg.payload_kp = 1.5;
    cfg.payload_ki = 0.2;
    cfg.acc_norm_limit_m_s2 = 6.0;
    cfg.integral_limit = 2.0;
    cfg.payload_integral_limit = 2.0;
    cfg.rest_lengths_override = restLengths();
    return cfg;
}

px4_msgs::msg::VehicleAttitudeSetpoint makeExpectedPlannerOutput()
{
    swarm_planner::geo::GpsOrigin origin;
    origin.set(kOriginLatDeg, kOriginLonDeg, kOriginAltM);

    swarm_planner::control::SwarmPlannerCore::Input input;
    input.uav_positions_ned[0] = swarm_planner::Vector3(0.0, 0.0, 0.0);
    input.uav_positions_ned[1] =
        swarm_planner::geo::lla_to_ned(kOriginLatDeg + 1.0e-5, kOriginLonDeg, kOriginAltM, origin);
    input.uav_positions_ned[2] =
        swarm_planner::geo::lla_to_ned(kOriginLatDeg, kOriginLonDeg + 1.0e-5, kOriginAltM, origin);
    input.uav_velocities_ned[0] = swarm_planner::Vector3::Zero();
    input.uav_velocities_ned[1] = swarm_planner::Vector3(0.1, 0.0, 0.0);
    input.uav_velocities_ned[2] = swarm_planner::Vector3(0.0, -0.1, 0.0);
    input.payload_position_ned = swarm_planner::geo::lla_to_ned(
        kPayloadLatDeg, kPayloadLonDeg, kPayloadAltM, origin);
    input.payload_velocity_ned = swarm_planner::Vector3(kPayloadVx, kPayloadVy, kPayloadVz);
    input.payload_target_ned = swarm_planner::Vector3(0.0, 0.0, -1.0);
    input.self_index = 0;
    input.mass = 2.0;

    swarm_planner::control::SwarmPlannerCore core;
    const auto cfg = makeSwarmConfig();
    if (!core.initialize(cfg))
    {
        throw std::runtime_error("swarm planner core init failed");
    }

    swarm_planner::control::SwarmPlannerCore::Output swarm_output;
    if (!core.compute(input, swarm_output) || !swarm_output.valid)
    {
        throw std::runtime_error("swarm planner compute failed");
    }

    fsmpx4::control::PositionAttitudeController::Config ctrl_cfg;
    ctrl_cfg.mass = 2.0;
    ctrl_cfg.gravity = 9.81;
    ctrl_cfg.hover_thrust_default = kHoverThrust;

    fsmpx4::control::PositionAttitudeController controller;
    if (!controller.initialize(ctrl_cfg))
    {
        throw std::runtime_error("controller init failed");
    }

    fsmpx4::types::UAVState state;
    state.rotation = fsmpx4::types::Matrix3::Identity();
    state.hover_thrust = kHoverThrust;

    const auto output = controller.computeFromDesiredAcceleration(
        state,
        fsmpx4::types::Vector3(
            swarm_output.desired_acceleration.x(),
            swarm_output.desired_acceleration.y(),
            swarm_output.desired_acceleration.z()),
        0.0,
        fsmpx4::types::Vector3::UnitX());

    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    msg.q_d[0] = static_cast<float>(output.qd.w());
    msg.q_d[1] = static_cast<float>(output.qd.x());
    msg.q_d[2] = static_cast<float>(output.qd.y());
    msg.q_d[3] = static_cast<float>(output.qd.z());
    msg.thrust_body[2] = static_cast<float>(output.thrust);
    return msg;
}

bool closeEnough(float a, float b, float tol = 1.0e-3f)
{
    return std::fabs(a - b) <= tol;
}

}  // namespace

class FSMPX4PlannerCmdTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        std::filesystem::create_directories("/tmp/fsmpx4_test_logs");
        std::filesystem::create_directories("/tmp/fsmpx4_test_home");
        setenv("ROS_LOG_DIR", "/tmp/fsmpx4_test_logs", 1);
        setenv("ROS_HOME", "/tmp/fsmpx4_test_home", 1);
        setenv("ROS_LOCALHOST_ONLY", "1", 1);
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
    }

    static void TearDownTestSuite()
    {
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
    }

    void SetUp() override
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            rclcpp::Parameter("ctrl_freq_max", 1.0),
            rclcpp::Parameter("takeoff.enabled", false),
            rclcpp::Parameter("thr_map.hover_percentage", kHoverThrust),
            rclcpp::Parameter("position.gps_origin_latitude_deg", kOriginLatDeg),
            rclcpp::Parameter("position.gps_origin_longitude_deg", kOriginLonDeg),
            rclcpp::Parameter("position.gps_origin_altitude_m", kOriginAltM),
            rclcpp::Parameter("swarm.uav_namespaces", std::vector<std::string>{"/px4_1", "/px4_2", "/px4_3"}),
            rclcpp::Parameter("swarm.self_index", 0),
            rclcpp::Parameter("swarm.mass", 2.0),
            rclcpp::Parameter("swarm.target_x_m", 0.0),
            rclcpp::Parameter("swarm.target_y_m", 0.0),
            rclcpp::Parameter("swarm.target_z_m", -1.0),
            rclcpp::Parameter(
                "swarm.payload_global_position_topic",
                "/px4_4/fmu/out/vehicle_global_position"),
            rclcpp::Parameter(
                "swarm.payload_local_position_topic",
                "/px4_4/fmu/out/vehicle_local_position"),
            rclcpp::Parameter("swarm.data_timeout_s", 0.05),
            rclcpp::Parameter("swarm.gravity", 9.81),
            rclcpp::Parameter("swarm.h_u_m", 1.0),
            rclcpp::Parameter("swarm.spring_k", 3.0),
            rclcpp::Parameter("swarm.damping_c1", 1.2),
            rclcpp::Parameter("swarm.friction_c2", 0.3),
            rclcpp::Parameter("swarm.vel_pid_kp", 1.2),
            rclcpp::Parameter("swarm.vel_pid_ki", 0.3),
            rclcpp::Parameter("swarm.vel_pid_kd", 0.1),
            rclcpp::Parameter("swarm.payload_kp", 1.5),
            rclcpp::Parameter("swarm.payload_ki", 0.2),
            rclcpp::Parameter("swarm.payload_mass", 1.0),
            rclcpp::Parameter("swarm.acc_norm_limit_m_s2", 6.0),
            rclcpp::Parameter("swarm.integral_limit", 2.0),
            rclcpp::Parameter("swarm.payload_integral_limit", 2.0),
            rclcpp::Parameter("swarm.structure_reference.rest_lengths", restLengths()),
        });

        fsm_ = std::make_shared<fsmpx4::FSMPX4>(options);
        io_node_ = std::make_shared<rclcpp::Node>("fsmpx4_planner_cmd_test_io");

        rc_pub_ = io_node_->create_publisher<px4_msgs::msg::ManualControlSetpoint>(
            "/rc/manual_control_setpoint", px4Qos());
        attitude_pub_ = io_node_->create_publisher<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", px4Qos());
        global_pos_pub_ = io_node_->create_publisher<px4_msgs::msg::VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position", px4Qos());
        local_pos_pub_ = io_node_->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", px4Qos());
        peer_global_pubs_[0] = io_node_->create_publisher<px4_msgs::msg::VehicleGlobalPosition>(
            "/px4_2/fmu/out/vehicle_global_position", px4Qos());
        peer_global_pubs_[1] = io_node_->create_publisher<px4_msgs::msg::VehicleGlobalPosition>(
            "/px4_3/fmu/out/vehicle_global_position", px4Qos());
        peer_local_pubs_[0] = io_node_->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
            "/px4_2/fmu/out/vehicle_local_position", px4Qos());
        peer_local_pubs_[1] = io_node_->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
            "/px4_3/fmu/out/vehicle_local_position", px4Qos());
        payload_global_pub_ = io_node_->create_publisher<px4_msgs::msg::VehicleGlobalPosition>(
            "/px4_4/fmu/out/vehicle_global_position", px4Qos());
        payload_local_pub_ = io_node_->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
            "/px4_4/fmu/out/vehicle_local_position", px4Qos());

        output_sub_ = io_node_->create_subscription<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/fmu/in/vehicle_attitude_setpoint", px4Qos(),
            [this](const px4_msgs::msg::VehicleAttitudeSetpoint::SharedPtr msg) {
                forwarded_.push_back(*msg);
            });
        debug_sub_ = io_node_->create_subscription<fsmpx4::msg::FSMDebug>(
            "/fsmpx4_fsm/debug", rclcpp::QoS(10).reliable(),
            [this](const fsmpx4::msg::FSMDebug::SharedPtr msg) {
                debug_messages_.push_back(*msg);
            });

        executor_.add_node(fsm_);
        executor_.add_node(io_node_);
        pump();
    }

    void TearDown() override
    {
        executor_.remove_node(io_node_);
        executor_.remove_node(fsm_);
        debug_sub_.reset();
        output_sub_.reset();
        payload_local_pub_.reset();
        payload_global_pub_.reset();
        peer_local_pubs_.fill(nullptr);
        peer_global_pubs_.fill(nullptr);
        local_pos_pub_.reset();
        global_pos_pub_.reset();
        attitude_pub_.reset();
        rc_pub_.reset();
        io_node_.reset();
        fsm_.reset();
        debug_messages_.clear();
        forwarded_.clear();
    }

    void pump(int rounds = 4)
    {
        for (int i = 0; i < rounds; ++i)
        {
            executor_.spin_some();
            std::this_thread::sleep_for(5ms);
        }
    }

    void publishSelfSensors()
    {
        attitude_pub_->publish(makeAttitude());
        global_pos_pub_->publish(makeGlobalPosition(kOriginLatDeg, kOriginLonDeg, kOriginAltM));
        local_pos_pub_->publish(makeLocalPosition());
    }

    void publishSwarmSensors()
    {
        peer_global_pubs_[0]->publish(
            makeGlobalPosition(kOriginLatDeg + 1.0e-5, kOriginLonDeg, kOriginAltM));
        peer_global_pubs_[1]->publish(
            makeGlobalPosition(kOriginLatDeg, kOriginLonDeg + 1.0e-5, kOriginAltM));
        peer_local_pubs_[0]->publish(makeLocalPosition(0.1, 0.0, 0.0));
        peer_local_pubs_[1]->publish(makeLocalPosition(0.0, -0.1, 0.0));
        payload_global_pub_->publish(makeGlobalPosition(kPayloadLatDeg, kPayloadLonDeg, kPayloadAltM));
        payload_local_pub_->publish(makeLocalPosition(kPayloadVx, kPayloadVy, kPayloadVz));
    }

    void publishRc(double aux1, double aux2)
    {
        rc_pub_->publish(makeRc(aux1, aux2));
    }

    void step()
    {
        pump();
        fsm_->process();
        pump();
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<fsmpx4::FSMPX4> fsm_;
    std::shared_ptr<rclcpp::Node> io_node_;

    rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint>::SharedPtr rc_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_pos_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_pub_;
    std::array<rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr, 2> peer_global_pubs_{};
    std::array<rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr, 2> peer_local_pubs_{};
    rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr payload_global_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr payload_local_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr output_sub_;
    rclcpp::Subscription<fsmpx4::msg::FSMDebug>::SharedPtr debug_sub_;
    std::vector<fsmpx4::msg::FSMDebug> debug_messages_;
    std::vector<px4_msgs::msg::VehicleAttitudeSetpoint> forwarded_;
};

TEST_F(FSMPX4PlannerCmdTest, SwarmStateGatesCommandModeAndTimeoutReturnsHover)
{
    publishSelfSensors();
    publishRc(1.0, -1.0);
    step();
    EXPECT_EQ(fsm_->currentState(), fsmpx4::FSMPX4::State::OFFBOARD_STABILIZED);

    publishSelfSensors();
    publishRc(1.0, 0.2);
    step();
    EXPECT_EQ(fsm_->currentState(), fsmpx4::FSMPX4::State::AUTO_HOVER);

    publishSelfSensors();
    publishRc(1.0, 1.0);
    step();
    EXPECT_EQ(fsm_->currentState(), fsmpx4::FSMPX4::State::AUTO_HOVER);

    const auto expected_output = makeExpectedPlannerOutput();
    publishSelfSensors();
    publishSwarmSensors();
    publishRc(1.0, 1.0);
    step();
    EXPECT_EQ(fsm_->currentState(), fsmpx4::FSMPX4::State::CMD_CTRL);
    ASSERT_FALSE(forwarded_.empty());
    const auto forwarded_cmd = forwarded_.back();
    EXPECT_TRUE(closeEnough(forwarded_cmd.q_d[0], expected_output.q_d[0]));
    EXPECT_TRUE(closeEnough(forwarded_cmd.q_d[1], expected_output.q_d[1]));
    EXPECT_TRUE(closeEnough(forwarded_cmd.q_d[2], expected_output.q_d[2]));
    EXPECT_TRUE(closeEnough(forwarded_cmd.q_d[3], expected_output.q_d[3]));
    EXPECT_TRUE(closeEnough(forwarded_cmd.thrust_body[2], expected_output.thrust_body[2]));

    std::this_thread::sleep_for(80ms);
    publishSelfSensors();
    publishRc(1.0, 1.0);
    step();
    EXPECT_EQ(fsm_->currentState(), fsmpx4::FSMPX4::State::AUTO_HOVER);
    ASSERT_FALSE(forwarded_.empty());
}

TEST_F(FSMPX4PlannerCmdTest, AutoHoverUsesFsmpx4ControllerEvenWithSwarmState)
{
    publishSelfSensors();
    publishRc(1.0, -1.0);
    step();
    EXPECT_EQ(fsm_->currentState(), fsmpx4::FSMPX4::State::OFFBOARD_STABILIZED);

    publishSelfSensors();
    publishRc(1.0, 0.2);
    step();
    EXPECT_EQ(fsm_->currentState(), fsmpx4::FSMPX4::State::AUTO_HOVER);
    ASSERT_FALSE(forwarded_.empty());

    const auto hover_cmd_before_swarm = forwarded_.back();
    EXPECT_TRUE(closeEnough(hover_cmd_before_swarm.q_d[0], 1.0f));
    EXPECT_TRUE(closeEnough(hover_cmd_before_swarm.q_d[1], 0.0f));
    EXPECT_TRUE(closeEnough(hover_cmd_before_swarm.q_d[2], 0.0f));
    EXPECT_TRUE(closeEnough(hover_cmd_before_swarm.q_d[3], 0.0f));
    EXPECT_TRUE(closeEnough(hover_cmd_before_swarm.thrust_body[2], static_cast<float>(-kHoverThrust)));

    publishSelfSensors();
    publishSwarmSensors();
    publishRc(1.0, 0.2);
    step();
    EXPECT_EQ(fsm_->currentState(), fsmpx4::FSMPX4::State::AUTO_HOVER);
    ASSERT_FALSE(forwarded_.empty());

    const auto hover_cmd_with_swarm = forwarded_.back();
    EXPECT_TRUE(closeEnough(hover_cmd_with_swarm.q_d[0], hover_cmd_before_swarm.q_d[0]));
    EXPECT_TRUE(closeEnough(hover_cmd_with_swarm.q_d[1], hover_cmd_before_swarm.q_d[1]));
    EXPECT_TRUE(closeEnough(hover_cmd_with_swarm.q_d[2], hover_cmd_before_swarm.q_d[2]));
    EXPECT_TRUE(closeEnough(hover_cmd_with_swarm.q_d[3], hover_cmd_before_swarm.q_d[3]));
    EXPECT_TRUE(closeEnough(
        hover_cmd_with_swarm.thrust_body[2],
        hover_cmd_before_swarm.thrust_body[2]));
}

TEST_F(FSMPX4PlannerCmdTest, DebugTopicCarriesFsmState)
{
    publishSelfSensors();
    publishRc(1.0, -1.0);
    step();

    ASSERT_FALSE(debug_messages_.empty());
    auto debug_msg = debug_messages_.back();
    EXPECT_EQ(debug_msg.fsm_state, fsmpx4::msg::FSMDebug::STATE_OFFBOARD_STABILIZED);
    EXPECT_EQ(debug_msg.fsm_state_name, "OFFBOARD_STABILIZED");

    publishSelfSensors();
    publishRc(1.0, 0.2);
    step();

    ASSERT_FALSE(debug_messages_.empty());
    debug_msg = debug_messages_.back();
    EXPECT_EQ(debug_msg.fsm_state, fsmpx4::msg::FSMDebug::STATE_AUTO_HOVER);
    EXPECT_EQ(debug_msg.fsm_state_name, "AUTO_HOVER");
}
