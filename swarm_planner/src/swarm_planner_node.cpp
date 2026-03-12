#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "swarm_planner/msg/swarm_planner_debug.hpp"
#include "swarm_planner/planner_core.h"
#include "swarm_planner/planner_types.h"
#include "swarm_planner/planner_utils.h"

namespace swarm_planner {
namespace {

constexpr std::array<const char*, kNumUavs> kPeerNamespaces{"/px4_1", "/px4_2", "/px4_3"};

rclcpp::QoS makePx4Qos()
{
    return rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile);
}

rclcpp::QoS makeReliableQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::Volatile);
}

geometry_msgs::msg::Vector3 toMsg(const Vector3& v)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
}

}  // namespace

class SwarmPlannerNode : public rclcpp::Node
{
public:
    explicit SwarmPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node("swarm_planner", options)
    {
        const PlannerNodeParams params = loadPlannerNodeParams(*this);

        px4_ns_ = params.px4_ns;
        self_index_ = params.self_index;
        mass_ = params.mass;
        gravity_ = params.gravity;
        compute_hz_ = params.compute_hz;
        target_ned_ = params.target_ned;

        if (!core_.initialize(params.core))
        {
            RCLCPP_WARN(get_logger(), "SwarmPlannerCore initialization failed");
        }

        state_actions_.configure(params.state_actions);

        initializePublishers(params.output_topic);
        initializeSubscriptions(params.payload_navsat_topic);
        const auto period = std::chrono::microseconds(
            static_cast<int64_t>(1'000'000.0 / std::max(1.0, compute_hz_)));
        timer_ = create_wall_timer(period, std::bind(&SwarmPlannerNode::computeAndPublish, this));
        RCLCPP_INFO(
            get_logger(),
            "SwarmPlannerNode started: px4_ns=%s self_index=%d output=%s",
            px4_ns_.c_str(),
            self_index_,
            params.output_topic.c_str());
    }

private:
    void initializePublishers(const std::string& output_topic)
    {
        desired_acc_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            output_topic, makeReliableQos());
        debug_pub_ = create_publisher<swarm_planner::msg::SwarmPlannerDebug>(
            "~/debug", makeReliableQos());
    }

    void initializeSubscriptions(const std::string& payload_navsat_topic)
    {
        const auto px4_qos = makePx4Qos();
        for (size_t i = 0; i < kPeerNamespaces.size(); ++i)
        {
            const std::string peer_ns = kPeerNamespaces[i];
            peer_global_subs_[i] = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
                peer_ns + "/fmu/out/vehicle_global_position",
                px4_qos,
                [this, i](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
                    state_actions_.handlePeerGlobalPosition(i, msg);
                });
            peer_local_subs_[i] = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                peer_ns + "/fmu/out/vehicle_local_position",
                px4_qos,
                [this, i](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                    state_actions_.handlePeerLocalPosition(i, msg);
                });
        }

        payload_navsat_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            payload_navsat_topic,
            makeReliableQos(),
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                state_actions_.handlePayloadNavSat(msg);
            });
    }

    void computeAndPublish()
    {
        const auto now = get_clock()->now();
        SwarmCmdSnapshot snapshot;
        std::string reason;
        if (!state_actions_.getSnapshot(now, snapshot, reason))
        {
            return;
        }

        control::SwarmPlannerCore::Input input;
        for (size_t i = 0; i < kNumUavs; ++i)
        {
            input.uav_positions_ned[i] = snapshot.peers[i].position;
            input.uav_velocities_ned[i] = snapshot.peers[i].velocity;
        }

        input.payload_position_ned = snapshot.payload.position;
        input.payload_velocity_ned = snapshot.payload.velocity;
        input.payload_target_ned = target_ned_;
        input.previous_thrust_vector = last_thrust_vector_;
        input.self_index = self_index_;
        input.mass = mass_;
        input.dt = last_compute_stamp_.nanoseconds() > 0
            ? (now - last_compute_stamp_).seconds()
            : 0.0;

        control::SwarmPlannerCore::Output output;
        if (!core_.compute(input, output) || !output.valid)
        {
            return;
        }

        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp = now;
        msg.header.frame_id = "ned";
        msg.vector.x = output.desired_acceleration.x();
        msg.vector.y = output.desired_acceleration.y();
        msg.vector.z = output.desired_acceleration.z();
        desired_acc_pub_->publish(msg);
        publishDebug(now, output);

        last_thrust_vector_ =
            Vector3(0.0, 0.0, mass_ * gravity_) - mass_ * output.desired_acceleration;
        last_compute_stamp_ = now;
    }

    void publishDebug(
        const rclcpp::Time& now,
        const control::SwarmPlannerCore::Output& output)
    {
        if (debug_pub_->get_subscription_count() == 0 &&
            debug_pub_->get_intra_process_subscription_count() == 0)
        {
            return;
        }

        const auto& debug = output.debug;
        swarm_planner::msg::SwarmPlannerDebug msg;
        msg.header.stamp = now;
        msg.header.frame_id = "ned";
        msg.self_index = debug.self_index;
        msg.mass = debug.mass;
        msg.dt_input = debug.dt_input;
        msg.dt_used = debug.dt_used;
        msg.dt_valid_for_update = debug.dt_valid_for_update;
        msg.structure_locked = debug.structure_locked;
        msg.used_cfo = debug.used_cfo;
        msg.valid = debug.valid;
        msg.payload_position_ned = toMsg(debug.payload_position_ned);
        msg.payload_velocity_ned = toMsg(debug.payload_velocity_ned);
        msg.payload_target_ned = toMsg(debug.payload_target_ned);
        msg.previous_thrust_vector = toMsg(debug.previous_thrust_vector);

        for (size_t i = 0; i < kNumUavs; ++i)
        {
            msg.uav_positions_ned[i] = toMsg(debug.uav_positions_ned[i]);
            msg.uav_velocities_ned[i] = toMsg(debug.uav_velocities_ned[i]);
            msg.beta[i] = debug.beta[i];
        }

        for (size_t i = 0; i < debug.virtual_positions_ned.size(); ++i)
        {
            msg.virtual_positions_ned[i] = toMsg(debug.virtual_positions_ned[i]);
            msg.virtual_velocities_ned[i] = toMsg(debug.virtual_velocities_ned[i]);
        }

        for (size_t i = 0; i < debug.rest_lengths.size(); ++i)
        {
            msg.rest_lengths[i] = debug.rest_lengths[i];
        }

        msg.passive_force = toMsg(debug.passive_force);
        msg.tracking_input = toMsg(debug.tracking_input);
        msg.virtual_acceleration = toMsg(debug.virtual_acceleration);
        msg.mapped_acceleration = toMsg(debug.mapped_acceleration);
        msg.cfo_acceleration = toMsg(debug.cfo_acceleration);
        msg.desired_acceleration = toMsg(debug.desired_acceleration);
        debug_pub_->publish(std::move(msg));
    }

    control::SwarmPlannerCore core_{};
    SwarmStateActions state_actions_{};
    std::string px4_ns_{};
    int self_index_{-1};
    double mass_{2.0};
    double gravity_{9.81};
    double compute_hz_{200.0};
    Vector3 target_ned_{Vector3::Zero()};
    rclcpp::Time last_compute_stamp_{0, 0, RCL_ROS_TIME};
    Vector3 last_thrust_vector_{Vector3::Zero()};
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr desired_acc_pub_;
    rclcpp::Publisher<swarm_planner::msg::SwarmPlannerDebug>::SharedPtr debug_pub_;
    std::array<rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr, kNumUavs>
        peer_global_subs_{};
    std::array<rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr, kNumUavs>
        peer_local_subs_{};
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr payload_navsat_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace swarm_planner

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<swarm_planner::SwarmPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
