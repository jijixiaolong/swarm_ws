#include "swarm_planner/planner_utils.h"

#include <algorithm>
#include <cmath>

namespace swarm_planner {
namespace {

std::string resolveOutputTopic(rclcpp::Node& node, const std::string& px4_ns)
{
    // 允许用户显式覆盖输出话题；未配置时按 px4 命名空间推导默认值，
    // 这样既能单机运行，也能在多机命名空间下自动得到不冲突的话题名。
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
    // 这里使用局部切平面近似，假设编队工作范围相对地球半径足够小。
    // NED 中 z 轴向下，因此高度要用 origin.alt_m - alt_m。
    return {
        (lat_rad - origin.lat_rad) * kEarthRadiusM,
        (lon_rad - origin.lon_rad) * kEarthRadiusM * std::cos(origin.lat_rad),
        origin.alt_m - alt_m};
}

}  // namespace geo

void SwarmStateActions::configure(const SwarmStateActionsConfig& cfg)
{
    // configure 既加载参数，也顺手清空所有缓存，
    // 避免旧任务残留的时戳和速度估计影响新任务。
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
    // 只接受索引有效且经 PX4 标记为 valid 的经纬高数据。
    if (idx >= kNumUavs || !msg->lat_lon_valid || !msg->alt_valid)
    {
        return;
    }

    // 统一用 ROS time 打时间戳，便于后续在同一时钟域内做超时判断。
    peer_positions_[idx].update(
        geo::lla_to_ned(msg->lat, msg->lon, msg->alt, swarm_origin_),
        rclcpp::Clock(RCL_ROS_TIME).now());
}

void SwarmStateActions::handlePeerLocalPosition(
    const size_t idx,
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    // PX4 local_position 直接给出 NED 速度，不需要额外坐标变换。
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
    // NavSatFix 没有 PX4 那样的 valid bit，这里退化为有限值检查。
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
    // 先做完整的新鲜度检查，再统一拷贝。
    // 这样可以保证 snapshot 内所有字段来自“同一拍可接受的数据集合”。
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
    // reason 仅在失败时携带阻塞原因；成功路径清空，便于上层按需打日志。
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
            // 用一阶低通滤波的差分速度抑制 GNSS 噪声：
            // alpha 越大，越相信上一拍速度；越小，越相信当前位置差分。
            Vector3 velocity_estimate =
                alpha * payload_velocity_ +
                (1.0 - alpha) * (pos - payload_position_.value) / dt;
            const double velocity_norm = velocity_estimate.norm();
            if (vel_max > 0.0 &&
                velocity_norm > vel_max)
            {
                // 再做一次模长限幅，避免单次 GNSS 跳变产生过大的虚假速度。
                velocity_estimate *= vel_max / velocity_norm;
            }
            payload_velocity_ = velocity_estimate;
        }
    }
    else
    {
        // 第一帧没有前向差分基础，只能把速度初始化为零。
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

    // 这一段负责节点级参数：自身编号、质量、重力、计算频率和目标点。
    params.px4_ns = node.declare_parameter<std::string>("px4_ns", params.px4_ns);
    params.self_index = node.declare_parameter<int>("self_index", params.self_index);
    params.mass = node.declare_parameter<double>("mass", params.mass);
    params.gravity = node.declare_parameter<double>("gravity", params.gravity);
    params.compute_hz = node.declare_parameter<double>("compute_hz", params.compute_hz);
    params.target_ned.x() = node.declare_parameter<double>("target_x_m", params.target_ned.x());
    params.target_ned.y() = node.declare_parameter<double>("target_y_m", params.target_ned.y());
    params.target_ned.z() = node.declare_parameter<double>("target_z_m", params.target_ned.z());

    // core 参数直接下发给控制器，保持参数名和内部 Config 结构一一对应。
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

    // state_actions 参数控制坐标原点、消息超时和 payload 速度估计器。
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
    // output topic 支持显式配置，也支持基于 px4_ns 自动推导默认值。
    params.output_topic = resolveOutputTopic(node, params.px4_ns);

    return params;
}

namespace control {
namespace planner_utils {

Vector3 computeDesiredPayloadVelocity(
    const SwarmPlannerCore::Input& input,
    const SwarmPlannerCore::Config& cfg)
{
    // 任务层目前是最简单的位置 P 控制：
    // payload 偏离目标越远，给出的期望速度越大，方向指向目标点。
    return -cfg.payload_kp * (input.payload_position_ned - input.payload_target_ned);
}

bool loadRestLengthMatrix(
    const std::vector<double>& rest_lengths_override,
    RestLengthMatrix& rest_lengths)
{
    // 外部参数采用一维数组，内部控制算法用矩阵表示；
    // 这里完成两者之间的搬运和基本数值检查。
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
    // 节点到自身的参考长度理论上必须为 0。
    // 这里即使已经是 0 也再显式写回一次，保证结果矩阵规范化。
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
    // 结构参考长度应当描述无向图，因此矩阵必须对称。
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
        // 首帧或无效 dt 时没有可用微分信息，按 0 处理更稳妥。
        return Vector3::Zero();
    }

    return (velocity_error - previous_velocity_error) / dt;
}

void clampIntegral(Vector3& velocity_integral, const double integral_limit)
{
    if (integral_limit <= 0.0)
    {
        // 配成非正数时等价于关闭积分限幅。
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
    // CFO 更新的必要条件：
    // 1) 功能开关打开；
    // 2) 存在有效离散步长；
    // 3) 缆绳长度足够，避免 cable_direction 数值不稳定。
    return cfg.cfo.enable && dt > 0.0 &&
           cable_length >= std::max(cfg.cfo.l_min_m, 0.0);
}

void flattenRestLengths(
    const RestLengthMatrix& rest_lengths,
    std::array<double, SwarmPlannerCore::kNumNodes * SwarmPlannerCore::kNumNodes>& flat_rest_lengths)
{
    // debug 消息使用固定长度数组传输，发布前把矩阵重新摊平成行优先数组。
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
