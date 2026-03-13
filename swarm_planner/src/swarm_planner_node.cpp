#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "swarm_planner/msg/swarm_planner_debug.hpp"
#include "swarm_planner/planner_core.h"
#include "swarm_planner/planner_types.h"
#include "swarm_planner/planner_utils.h"
#include "swarm_planner/swarm_planner_node.h"

namespace swarm_planner {

// ROS2 节点只负责三件事：
// 1) 收集各 UAV 与 payload 的最新状态；
// 2) 在定时器里拼装控制输入并调用 SwarmPlannerCore；
// 3) 将结果和调试信息发布出去。
class SwarmPlannerNode : public rclcpp::Node
{
public:
    explicit SwarmPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node("swarm_planner", options)
    {
        // 所有参数在构造时一次性声明并读取，便于节点启动后保持行为稳定。
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

        // state_actions_ 负责把异步回调里的原始消息缓存成“当前可计算快照”。
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
        // 期望加速度给下游飞控使用，debug 话题仅用于诊断和可视化。
        desired_acc_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            output_topic, node_utils::makeReliableQos());
        debug_pub_ = create_publisher<swarm_planner::msg::SwarmPlannerDebug>(
            "~/debug", node_utils::makeReliableQos());
    }

    void initializeSubscriptions(const std::string& payload_navsat_topic)
    {
        const auto px4_qos = node_utils::makePx4Qos();
        for (size_t i = 0; i < node_utils::kPeerNamespaces.size(); ++i)
        {
            const std::string peer_ns = node_utils::kPeerNamespaces[i];
            // 位置和速度分别来自 PX4 的 global/local topic：
            // global 用于统一转换到 swarm NED 坐标，local 直接提供速度。
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
            node_utils::makeReliableQos(),
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                // payload 当前只依赖 GNSS；速度由工具层用相邻位置差分估计。
                state_actions_.handlePayloadNavSat(msg);
            });
    }

    void computeAndPublish()
    {
        const auto now = get_clock()->now();
        SwarmCmdSnapshot snapshot;
        std::string reason;
        // 如果任一输入源超时，整拍跳过，避免把混合时刻的数据送进控制器。
        if (!state_actions_.getSnapshot(now, snapshot, reason))
        {
            return;
        }

        // 将聚合后的快照复制成 core::Input，保证核心控制器与 ROS 消息类型完全解耦。
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
        // input.dt 仅作为 debug / 兼容信息保留；当前 core 内部实际使用固定离散步长。
        input.dt = last_compute_stamp_.nanoseconds() > 0
            ? (now - last_compute_stamp_).seconds()
            : 0.0;

        control::SwarmPlannerCore::Output output;
        if (!core_.compute(input, output) || !output.valid)
        {
            return;
        }

        auto msg = node_utils::makeVector3StampedMsg(now, output.desired_acceleration);
        desired_acc_pub_->publish(msg);
        publishDebug(now, output);

        // 上一拍推力会在下一拍 CFO 估计中使用，因此这里要把本拍输出反推成推力向量缓存下来。
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
            // 调试消息体较大，没有订阅者时直接跳过，减少拷贝和序列化开销。
            return;
        }

        debug_pub_->publish(node_utils::makeDebugMsg(now, output.debug));
    }

    control::SwarmPlannerCore core_{};
    SwarmStateActions state_actions_{};
    std::string px4_ns_{};
    int self_index_{-1};
    double mass_{2.0};
    double gravity_{9.81};
    double compute_hz_{200.0};
    Vector3 target_ned_{Vector3::Zero()};
    // 下面两个成员共同提供“上一拍控制输入”的记忆，用于 CFO 更新。
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
