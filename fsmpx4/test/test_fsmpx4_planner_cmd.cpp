#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include "fsmpx4.h"

namespace {

using namespace std::chrono_literals;

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

px4_msgs::msg::VehicleGlobalPosition makeGlobalPosition()
{
    px4_msgs::msg::VehicleGlobalPosition msg{};
    msg.lat_lon_valid = true;
    msg.alt_valid = true;
    msg.lat = 30.0;
    msg.lon = 120.0;
    msg.alt = 10.0f;
    return msg;
}

px4_msgs::msg::VehicleLocalPosition makeLocalPosition()
{
    px4_msgs::msg::VehicleLocalPosition msg{};
    msg.z_valid = true;
    msg.z_global = true;
    msg.ref_alt = 10.0f;
    msg.z = 0.0f;
    msg.v_xy_valid = true;
    msg.v_z_valid = true;
    msg.vx = 0.0f;
    msg.vy = 0.0f;
    msg.vz = 0.0f;
    return msg;
}

geometry_msgs::msg::Vector3Stamped makePlannerAcceleration()
{
    geometry_msgs::msg::Vector3Stamped msg{};
    msg.header.frame_id = "ned";
    msg.vector.x = 1.2;
    msg.vector.y = -0.4;
    msg.vector.z = 0.3;
    return msg;
}

px4_msgs::msg::VehicleAttitudeSetpoint makeExpectedPlannerOutput(
    const geometry_msgs::msg::Vector3Stamped& planner_acceleration)
{
    constexpr double kHoverThrust = 0.71;

    fsmpx4::control::PositionAttitudeController::Config cfg;
    cfg.mass = 2.0;
    cfg.gravity = 9.81;
    cfg.hover_thrust_default = kHoverThrust;

    fsmpx4::control::PositionAttitudeController controller;
    if (!controller.initialize(cfg))
    {
        throw std::runtime_error("controller init failed");
    }

    fsmpx4::types::UAVState state;
    state.rotation = fsmpx4::types::Matrix3::Identity();
    state.hover_thrust = kHoverThrust;

    const auto output = controller.computeFromDesiredAcceleration(
        state,
        fsmpx4::types::Vector3(
            planner_acceleration.vector.x,
            planner_acceleration.vector.y,
            planner_acceleration.vector.z),
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
            rclcpp::Parameter("command.timeout_s", 0.05),
            rclcpp::Parameter("thr_map.hover_percentage", 0.71),
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
        planner_pub_ = io_node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/planner/desired_acceleration", rclcpp::QoS(10).reliable());

        output_sub_ = io_node_->create_subscription<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/fmu/in/vehicle_attitude_setpoint", px4Qos(),
            [this](const px4_msgs::msg::VehicleAttitudeSetpoint::SharedPtr msg) {
                forwarded_.push_back(*msg);
            });

        executor_.add_node(fsm_);
        executor_.add_node(io_node_);
        pump();
    }

    void TearDown() override
    {
        executor_.remove_node(io_node_);
        executor_.remove_node(fsm_);
        output_sub_.reset();
        planner_pub_.reset();
        local_pos_pub_.reset();
        global_pos_pub_.reset();
        attitude_pub_.reset();
        rc_pub_.reset();
        io_node_.reset();
        fsm_.reset();
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

    void publishSensors()
    {
        attitude_pub_->publish(makeAttitude());
        global_pos_pub_->publish(makeGlobalPosition());
        local_pos_pub_->publish(makeLocalPosition());
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
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr planner_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr output_sub_;
    std::vector<px4_msgs::msg::VehicleAttitudeSetpoint> forwarded_;
};

TEST_F(FSMPX4PlannerCmdTest, PlannerFreshnessGatesCommandModeAndTimeoutReturnsHover)
{
    publishSensors();
    publishRc(1.0, -1.0);
    step();
    EXPECT_EQ(fsm_->currentState(), fsmpx4::FSMPX4::State::OFFBOARD_STABILIZED);

    publishSensors();
    publishRc(1.0, 0.2);
    step();
    EXPECT_EQ(fsm_->currentState(), fsmpx4::FSMPX4::State::AUTO_HOVER);

    const auto planner_acceleration = makePlannerAcceleration();
    const auto expected_output = makeExpectedPlannerOutput(planner_acceleration);

    publishSensors();
    publishRc(1.0, 1.0);
    const auto count_before_cmd_request = forwarded_.size();
    step();
    EXPECT_EQ(fsm_->currentState(), fsmpx4::FSMPX4::State::AUTO_HOVER);
    ASSERT_GT(forwarded_.size(), count_before_cmd_request);
    const auto hover_output = forwarded_.back();
    EXPECT_FALSE(closeEnough(hover_output.thrust_body[2], expected_output.thrust_body[2]));

    planner_pub_->publish(planner_acceleration);
    pump();
    publishSensors();
    publishRc(1.0, 1.0);
    step();
    EXPECT_EQ(fsm_->currentState(), fsmpx4::FSMPX4::State::CMD_CTRL);
    ASSERT_FALSE(forwarded_.empty());
    const auto forwarded_planner = forwarded_.back();
    EXPECT_TRUE(closeEnough(forwarded_planner.q_d[0], expected_output.q_d[0]));
    EXPECT_TRUE(closeEnough(forwarded_planner.q_d[1], expected_output.q_d[1]));
    EXPECT_TRUE(closeEnough(forwarded_planner.q_d[2], expected_output.q_d[2]));
    EXPECT_TRUE(closeEnough(forwarded_planner.q_d[3], expected_output.q_d[3]));
    EXPECT_TRUE(closeEnough(forwarded_planner.thrust_body[2], expected_output.thrust_body[2]));

    std::this_thread::sleep_for(80ms);
    publishSensors();
    publishRc(1.0, 1.0);
    const auto count_before_timeout = forwarded_.size();
    step();
    EXPECT_EQ(fsm_->currentState(), fsmpx4::FSMPX4::State::AUTO_HOVER);
    EXPECT_EQ(forwarded_.size(), count_before_timeout);

    publishSensors();
    publishRc(1.0, 1.0);
    step();
    EXPECT_EQ(fsm_->currentState(), fsmpx4::FSMPX4::State::AUTO_HOVER);
    ASSERT_GT(forwarded_.size(), count_before_timeout);
    const auto recovered_hover = forwarded_.back();
    EXPECT_FALSE(closeEnough(recovered_hover.thrust_body[2], expected_output.thrust_body[2]));
}
