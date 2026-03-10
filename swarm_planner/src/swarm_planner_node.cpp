#include "swarm_planner/swarm_planner_node.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>

#include "swarm_planner/param_loader.h"

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

SwarmPlannerNode::SwarmPlannerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("swarm_planner", options)
{
    const auto params = param_loader::load_from_node(*this);

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

    aggregator_.configure(params.aggregator);

    initializePublishers(params.output_topic);
    initializeSubscriptions(params.payload_navsat_topic);
    const auto period = std::chrono::microseconds(
        static_cast<int64_t>(1'000'000.0 / std::max(1.0, compute_hz_)));
    timer_ = create_wall_timer(period, std::bind(&SwarmPlannerNode::computeAndPublish, this));
    RCLCPP_INFO(get_logger(), "SwarmPlannerNode started: px4_ns=%s self_index=%d output=%s",
                px4_ns_.c_str(), self_index_, params.output_topic.c_str());
    RCLCPP_INFO(
        get_logger(),
        "Planner frames: swarm_origin=config payload_navsat_topic=%s",
        params.payload_navsat_topic.c_str());
}

void SwarmPlannerNode::initializePublishers(const std::string& output_topic)
{
    desired_acc_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        output_topic, makeReliableQos());
    debug_pub_ = create_publisher<swarm_planner::msg::SwarmPlannerDebug>(
        "~/debug", makeReliableQos());
}

void SwarmPlannerNode::initializeSubscriptions(const std::string& payload_navsat_topic)
{
    const auto px4_qos = makePx4Qos();
    for (size_t i = 0; i < kPeerNamespaces.size(); ++i)
    {
        const std::string peer_ns = kPeerNamespaces[i];
        peer_global_subs_[i] = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            peer_ns + "/fmu/out/vehicle_global_position", px4_qos,
            [this, i](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
                aggregator_.handlePeerGlobalPosition(i, msg);
            });
        peer_local_subs_[i] = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            peer_ns + "/fmu/out/vehicle_local_position", px4_qos,
            [this, i](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                aggregator_.handlePeerLocalPosition(i, msg);
            });
    }

    const auto payload_qos = makeReliableQos();
    payload_navsat_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        payload_navsat_topic, payload_qos,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            aggregator_.handlePayloadNavSat(msg);
        });
}

/**
 * @brief 聚合编队状态、执行控制计算并发布本机期望加速度。
 *
 * 该函数由定时器周期触发。函数首先从状态聚合器获取时间同步后的
 * 多机与载荷状态快照，然后组装控制核心输入，最后调用
 * `SwarmPlannerCore` 计算并发布当前无人机的期望加速度指令。
 */
void SwarmPlannerNode::computeAndPublish()
{
    const auto now = get_clock()->now();
    SwarmCmdSnapshot snapshot;
    std::string reason;

    // 1. 获取时间同步后的多机与载荷状态快照；若状态不完整则跳过本周期。
    if (!aggregator_.getSnapshot(now, snapshot, reason))
    {
        return;
    }

    control::SwarmPlannerCore::Input input;

    // 2. 组装控制核心输入。
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
    // 首次计算时无历史时间戳，dt 置为 0。
    input.dt = (last_compute_stamp_.nanoseconds() > 0)
        ? (now - last_compute_stamp_).seconds() : 0.0;

    control::SwarmPlannerCore::Output output;

    // 3. 执行控制计算；若结果无效则不发布。
    if (!core_.compute(input, output) || !output.valid)
    {
        return;
    }

    // 4. 发布当前无人机在 NED 坐标系下的期望加速度指令。
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = now;
    msg.header.frame_id = "ned";
    msg.vector.x = output.desired_acceleration.x();
    msg.vector.y = output.desired_acceleration.y();
    msg.vector.z = output.desired_acceleration.z();
    desired_acc_pub_->publish(msg);
    publishDebug(now, output);

    // 5. 缓存本周期结果，供下一周期计算使用。
    last_thrust_vector_ = Vector3(0.0, 0.0, mass_ * gravity_) - mass_ * output.desired_acceleration;
    last_compute_stamp_ = now;
}

void SwarmPlannerNode::publishDebug(
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

}  // namespace swarm_planner
