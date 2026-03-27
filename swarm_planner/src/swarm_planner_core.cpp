#include "swarm_planner/planner_core.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace swarm_planner {
namespace control {

//==============================================================================
// 仅在本翻译单元内使用的数学工具函数
//==============================================================================
namespace {

constexpr double kFixedComputeDtS = 1.0 / 200.0;

using RestLengthMatrix =
    Eigen::Matrix<double, SwarmPlannerCore::kNumNodes, SwarmPlannerCore::kNumNodes>;

Vector3 computeVelocityErrorDerivative(
    const Vector3& error,
    const Vector3& prev_error,
    double dt,
    bool prev_valid)
{
    if (dt <= 0.0 || !prev_valid)
    {
        return Vector3::Zero();
    }

    return (error - prev_error) / dt;
}

void clampIntegral(Vector3& integral, double limit)
{
    for (int axis = 0; axis < 3; ++axis)
    {
        integral(axis) = std::clamp(integral(axis), -limit, limit);
    }
}

void flattenRestLengths(
    const RestLengthMatrix& m,
    std::array<double, SwarmPlannerCore::kNumNodes * SwarmPlannerCore::kNumNodes>& out)
{
    constexpr int N = SwarmPlannerCore::kNumNodes;
    for (int row = 0; row < N; ++row)
    {
        for (int col = 0; col < N; ++col)
        {
            out[static_cast<size_t>(row * N + col)] = m(row, col);
        }
    }
}

}  // namespace

//==============================================================================
// SwarmPlannerCore 成员函数
//==============================================================================

bool SwarmPlannerCore::initialize(const Config& cfg)
{
    cfg_ = cfg;
    ready_ = false;

    if (cfg_.h_u_m <= 0.0 || !loadRestLengths())
    {
        return false;
    }

    resetRuntimeState();
    ready_ = true;
    return true;
}

void SwarmPlannerCore::reset()
{
    // reset 只清空跨周期的运行态（积分、微分），
    // 不触碰配置和已加载的结构参考长度，以便在不重新 initialize 的情况下重启一次任务。
    resetRuntimeState();
}

void SwarmPlannerCore::resetRuntimeState()
{
    payload_position_integral_.setZero();
    velocity_integral_.setZero();
    prev_velocity_error_.setZero();
    prev_velocity_error_valid_ = false;
}

bool SwarmPlannerCore::finiteVector(const Vector3& v)
{
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

Vector3 SwarmPlannerCore::clipNorm(const Vector3& v, const double max_norm)
{
    if (max_norm <= 0.0)
    {
        return Vector3::Zero();
    }

    const double norm = v.norm();
    if (norm <= max_norm)
    {
        return v;
    }

    return v * (max_norm / norm);
}

bool SwarmPlannerCore::loadRestLengths()
{
    const auto& v = cfg_.rest_lengths_override;
    if (v.size() != static_cast<size_t>(kNumNodes * kNumNodes))
    {
        return false;
    }

    RestLengthMatrix m;
    for (int row = 0; row < kNumNodes; ++row)
    {
        for (int col = 0; col < kNumNodes; ++col)
        {
            const double length = v[static_cast<size_t>(row * kNumNodes + col)];
            if (!std::isfinite(length) || length < 0.0)
            {
                return false;
            }
            if (row == col && length != 0.0)
            {
                return false;
            }
            if (col < row && length != m(col, row))
            {
                return false;
            }
            m(row, col) = length;
        }
    }

    rest_lengths_ = m;
    return true;
}

bool SwarmPlannerCore::validateInput(const Input& input) const
{
    return ready_ && input.self_index >= 0 && input.self_index < kNumUavs;
}

bool SwarmPlannerCore::buildVirtualState(
    const Input& input,
    const double h_u,
    VirtualState& state) const
{
    // 虚拟网络节点布局：
    //   0~(kNumUavs-1)  各 UAV 对应的虚拟点（投影到 payload 正上方 h_u 高度截面）
    //   kNumUavs        payload 正上方 h_u 处的虚拟连接点
    //   kNumUavs+1      payload 本体
    state.q[kNumUavs + 1] = input.payload_position_ned;
    state.qdot[kNumUavs + 1] = input.payload_velocity_ned;
    state.q[kNumUavs] = input.payload_position_ned - Vector3(0.0, 0.0, h_u);
    state.qdot[kNumUavs] = input.payload_velocity_ned;

    for (int i = 0; i < kNumUavs; ++i)
    {
        const double dz =
            std::abs(input.payload_position_ned.z() - input.uav_positions_ned[i].z());

        if (dz < 1e-4)
        {
            return false;
        }

        const double alpha = h_u / dz;
        state.beta[i] = dz / h_u;
        state.q[i] = input.payload_position_ned +
                     alpha * (input.uav_positions_ned[i] - input.payload_position_ned);
        state.qdot[i] = input.payload_velocity_ned +
                        alpha * (input.uav_velocities_ned[i] - input.payload_velocity_ned);
    }

    return true;
}

Vector3 SwarmPlannerCore::computeDesiredPayloadVelocity(
    const Input& input,
    const double dt)
{
    Vector3 position_error = input.payload_position_ned - input.payload_target_ned;
    if (cfg_.payload_error_limit_xy_m > 0.0)
    {
        position_error.x() = std::clamp(
            position_error.x(), -cfg_.payload_error_limit_xy_m, cfg_.payload_error_limit_xy_m);
        position_error.y() = std::clamp(
            position_error.y(), -cfg_.payload_error_limit_xy_m, cfg_.payload_error_limit_xy_m);
    }
    if (cfg_.payload_error_limit_z_m > 0.0)
    {
        position_error.z() = std::clamp(
            position_error.z(), -cfg_.payload_error_limit_z_m, cfg_.payload_error_limit_z_m);
    }

    if (cfg_.payload_ki <= 0.0)
    {
        payload_position_integral_.setZero();
        return -cfg_.payload_kp * position_error;
    }

    if (dt > 0.0)
    {
        payload_position_integral_ += position_error * dt;
        clampIntegral(payload_position_integral_, std::abs(cfg_.payload_integral_limit));
    }

    return -cfg_.payload_kp * position_error
           - cfg_.payload_ki * payload_position_integral_;
}

Vector3 SwarmPlannerCore::computePassiveNetworkForce(
    const int self_index,
    const VirtualState& state) const
{
    const Vector3& q_i = state.q[self_index];
    const Vector3& qdot_i = state.qdot[self_index];

    // 弹簧：将节点对拉回参考结构长度；阻尼：抑制节点间相对速度；摩擦：耗散自身速度。
    Vector3 spring_force = Vector3::Zero();
    Vector3 damping_force = Vector3::Zero();
    for (int j = 0; j < kNumNodes; ++j)
    {
        if (j == self_index)
        {
            continue;
        }

        const Vector3 diff = q_i - state.q[j];
        const double length = diff.norm();
        spring_force += cfg_.spring_k * (1.0 - rest_lengths_(self_index, j) / length) * diff;
        damping_force += cfg_.damping_c1 * (qdot_i - state.qdot[j]);
    }

    return spring_force + damping_force + cfg_.friction_c2 * qdot_i;
}

Vector3 SwarmPlannerCore::computeTrackingInput(
    const Vector3& qdot_i,
    const double dt,
    const Vector3& desired_payload_velocity)
{
    const Vector3 velocity_error = qdot_i - desired_payload_velocity;
    const Vector3 derivative = computeVelocityErrorDerivative(
        velocity_error, prev_velocity_error_, dt, prev_velocity_error_valid_);

    if (dt > 0.0)
    {
        velocity_integral_ += velocity_error * dt;
        clampIntegral(velocity_integral_, std::abs(cfg_.integral_limit));
        prev_velocity_error_ = velocity_error;
        prev_velocity_error_valid_ = true;
    }

    return -cfg_.vel_pid_kp * velocity_error
           - cfg_.vel_pid_ki * velocity_integral_
           - cfg_.vel_pid_kd * derivative;
}

bool SwarmPlannerCore::compute(const Input& input, Output& output)
{
    output = Output{};

    if (!validateInput(input))
    {
        return false;
    }

    constexpr double dt = kFixedComputeDtS;

    VirtualState state;
    if (!buildVirtualState(input, cfg_.h_u_m, state))
    {
        return false;
    }
    const int self_index = input.self_index;
    const double mass = input.mass;

    const Vector3 passive_force = computePassiveNetworkForce(self_index, state);
    const Vector3 desired_payload_velocity = computeDesiredPayloadVelocity(input, dt);
    const Vector3 tracking_input =
        computeTrackingInput(state.qdot[self_index], dt, desired_payload_velocity);

    const Vector3 virtual_acceleration = -passive_force / mass + tracking_input / mass;
    const Vector3 mapped_acceleration = state.beta[self_index] * virtual_acceleration;

    // Payload 重力前馈：每架 UAV 分摊 payload 重量，在 NED 下为负 Z（向上）
    const Vector3 payload_gravity_ff(
        0.0, 0.0, -cfg_.payload_mass * cfg_.gravity / (kNumUavs * mass));

    const Vector3 desired_acceleration =
        clipNorm(mapped_acceleration + payload_gravity_ff, cfg_.acc_norm_limit_m_s2);

    if (!finiteVector(desired_acceleration))
    {
        return false;
    }

    output.desired_acceleration = desired_acceleration;
    output.valid = true;

    auto& dbg = output.debug;
    dbg.uav_positions_ned = input.uav_positions_ned;
    dbg.uav_velocities_ned = input.uav_velocities_ned;
    dbg.payload_position_ned = input.payload_position_ned;
    dbg.payload_velocity_ned = input.payload_velocity_ned;
    dbg.payload_target_ned = input.payload_target_ned;
    dbg.virtual_positions_ned = state.q;
    dbg.virtual_velocities_ned = state.qdot;
    dbg.beta = state.beta;
    flattenRestLengths(rest_lengths_, dbg.rest_lengths);
    dbg.passive_force = passive_force;
    dbg.tracking_input = tracking_input;
    dbg.virtual_acceleration = virtual_acceleration;
    dbg.mapped_acceleration = mapped_acceleration;
    dbg.desired_acceleration = desired_acceleration;
    dbg.self_index = input.self_index;
    dbg.mass = input.mass;
    dbg.structure_locked = ready_;
    dbg.valid = true;
    return true;
}

}  // namespace control
}  // namespace swarm_planner
