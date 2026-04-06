#include "swarm_planner/planner_core.h"

#include <algorithm>
#include <cmath>

namespace swarm_planner {
namespace control {

//==============================================================================
// 仅在本翻译单元内使用的数学工具函数
//==============================================================================
namespace {

constexpr double kFixedComputeDtS = 1.0 / 200.0;

using RestLengthMatrix =
    Eigen::Matrix<double, SwarmPlannerCore::kNumNodes, SwarmPlannerCore::kNumNodes>;

RestLengthMatrix buildEquilateralRestLengthMatrix(double side_length, double h_u)
{
    RestLengthMatrix m = RestLengthMatrix::Zero();
    const double radius = side_length / std::sqrt(3.0);
    const double payload_distance = std::hypot(radius, h_u);

    for (int i = 0; i < SwarmPlannerCore::kNumUavs; ++i)
    {
        for (int j = 0; j < SwarmPlannerCore::kNumUavs; ++j)
        {
            if (i != j)
            {
                m(i, j) = side_length;
            }
        }

        m(i, SwarmPlannerCore::kHubNodeIndex) = radius;
        m(SwarmPlannerCore::kHubNodeIndex, i) = radius;
        m(i, SwarmPlannerCore::kPayloadNodeIndex) = payload_distance;
        m(SwarmPlannerCore::kPayloadNodeIndex, i) = payload_distance;
    }

    m(SwarmPlannerCore::kHubNodeIndex, SwarmPlannerCore::kPayloadNodeIndex) = h_u;
    m(SwarmPlannerCore::kPayloadNodeIndex, SwarmPlannerCore::kHubNodeIndex) = h_u;
    return m;
}

Vector3 computePassivePairContribution(
    const Vector3& q_i,
    const Vector3& qdot_i,
    const Vector3& q_j,
    const Vector3& qdot_j,
    double rest_length,
    double spring_k,
    double damping_c1,
    double spring_force_clamp)
{
    const Vector3 diff = q_i - q_j;
    const double length = diff.norm();
    if (length < 1e-6)
    {
        return Vector3::Zero();
    }

    Vector3 spring_force = spring_k * (1.0 - rest_length / length) * diff;
    if (spring_force_clamp > 0.0)
    {
        const double magnitude = spring_force.norm();
        if (magnitude > spring_force_clamp)
        {
            spring_force *= spring_force_clamp / magnitude;
        }
    }

    return spring_force + damping_c1 * (qdot_i - qdot_j);
}

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
    if (v.empty())
    {
        if (!std::isfinite(cfg_.structure_side_length_m) || cfg_.structure_side_length_m <= 0.0)
        {
            return false;
        }
        rest_lengths_ = buildEquilateralRestLengthMatrix(cfg_.structure_side_length_m, cfg_.h_u_m);
        return true;
    }

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

bool SwarmPlannerCore::buildEquivalentState(
    const Input& input,
    const int self_index,
    const double h_u,
    EquivalentState& state) const
{
    state.q_payload = input.payload_position_ned;
    state.qdot_payload = input.payload_velocity_ned;
    state.q_hub = input.payload_position_ned - Vector3(0.0, 0.0, h_u);
    state.qdot_hub = input.payload_velocity_ned;

    for (int i = 0; i < kNumUavs; ++i)
    {
        const double dz =
            std::abs(input.payload_position_ned.z() - input.uav_positions_ned[static_cast<size_t>(i)].z());
        if (dz < 1e-4)
        {
            return false;
        }

        state.alpha[static_cast<size_t>(i)] = h_u / dz;
        state.beta[static_cast<size_t>(i)] = dz / h_u;
    }

    state.self = buildEquivalentUavNode(input, self_index, state);
    return true;
}

SwarmPlannerCore::EquivalentUavNode SwarmPlannerCore::buildEquivalentUavNode(
    const Input& input,
    const int uav_index,
    const EquivalentState& state) const
{
    const size_t idx = static_cast<size_t>(uav_index);
    const double alpha = state.alpha[idx];

    EquivalentUavNode node;
    node.q = input.payload_position_ned +
             alpha * (input.uav_positions_ned[idx] - input.payload_position_ned);
    node.qdot = input.payload_velocity_ned +
                alpha * (input.uav_velocities_ned[idx] - input.payload_velocity_ned);
    return node;
}

void SwarmPlannerCore::populateVirtualDebugState(
    const Input& input,
    const EquivalentState& state,
    DebugState& debug) const
{
    for (int i = 0; i < kNumUavs; ++i)
    {
        const size_t idx = static_cast<size_t>(i);
        const EquivalentUavNode node =
            (i == debug.self_index) ? state.self : buildEquivalentUavNode(input, i, state);
        debug.virtual_positions_ned[idx] = node.q;
        debug.virtual_velocities_ned[idx] = node.qdot;
    }

    debug.virtual_positions_ned[static_cast<size_t>(kHubNodeIndex)] = state.q_hub;
    debug.virtual_velocities_ned[static_cast<size_t>(kHubNodeIndex)] = state.qdot_hub;
    debug.virtual_positions_ned[static_cast<size_t>(kPayloadNodeIndex)] = state.q_payload;
    debug.virtual_velocities_ned[static_cast<size_t>(kPayloadNodeIndex)] = state.qdot_payload;
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
        Vector3 v_des = -cfg_.payload_kp * position_error;
        if (cfg_.max_payload_velocity > 0.0)
        {
            v_des = clipNorm(v_des, cfg_.max_payload_velocity);
        }
        return v_des;
    }

    // 只对 Z 轴（高度）积分，消除重力引起的高度静差；X/Y 保持纯比例，避免水平振荡。
    if (dt > 0.0)
    {
        payload_position_integral_.z() += position_error.z() * dt;
        payload_position_integral_.z() = std::clamp(
            payload_position_integral_.z(),
            -std::abs(cfg_.payload_integral_limit),
             std::abs(cfg_.payload_integral_limit));
    }

    Vector3 v_desired = -cfg_.payload_kp * position_error
                        - cfg_.payload_ki * payload_position_integral_;

    // 速度限幅：防止大位置误差产生过高的期望速度导致摆锤超调
    if (cfg_.max_payload_velocity > 0.0)
    {
        v_desired = clipNorm(v_desired, cfg_.max_payload_velocity);
    }

    return v_desired;
}

Vector3 SwarmPlannerCore::computePassiveNetworkForce(
    const int self_index,
    const Input& input,
    const EquivalentState& state) const
{
    const Vector3& q_i = state.self.q;
    const Vector3& qdot_i = state.self.qdot;

    Vector3 passive_force = Vector3::Zero();
    for (int j = 0; j < kNumUavs; ++j)
    {
        if (j == self_index)
        {
            continue;
        }

        const EquivalentUavNode neighbor = buildEquivalentUavNode(input, j, state);
        passive_force += computePassivePairContribution(
            q_i,
            qdot_i,
            neighbor.q,
            neighbor.qdot,
            rest_lengths_(self_index, j),
            cfg_.spring_k,
            cfg_.damping_c1,
            cfg_.spring_force_clamp);
    }

    passive_force += computePassivePairContribution(
        q_i,
        qdot_i,
        state.q_hub,
        state.qdot_hub,
        rest_lengths_(self_index, kHubNodeIndex),
        cfg_.spring_k,
        cfg_.damping_c1,
        cfg_.spring_force_clamp);
    passive_force += computePassivePairContribution(
        q_i,
        qdot_i,
        state.q_payload,
        state.qdot_payload,
        rest_lengths_(self_index, kPayloadNodeIndex),
        cfg_.spring_k,
        cfg_.damping_c1,
        cfg_.spring_force_clamp);

    return passive_force + cfg_.friction_c2 * qdot_i;
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
        const double vlimit = std::abs(cfg_.integral_limit);
        for (int i = 0; i < 3; ++i)
        {
            velocity_integral_(i) = std::clamp(velocity_integral_(i), -vlimit, vlimit);
        }
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

    const int self_index = input.self_index;
    EquivalentState state;
    if (!buildEquivalentState(input, self_index, cfg_.h_u_m, state))
    {
        return false;
    }
    const double mass = input.mass;

    const Vector3 passive_force = computePassiveNetworkForce(self_index, input, state);
    const Vector3 desired_payload_velocity = computeDesiredPayloadVelocity(input, dt);
    const Vector3 tracking_input =
        computeTrackingInput(state.self.qdot, dt, desired_payload_velocity);

    const Vector3 virtual_acceleration = -passive_force / mass + tracking_input / mass;
    const Vector3 mapped_acceleration =
        state.beta[static_cast<size_t>(self_index)] * virtual_acceleration;
    const Vector3 desired_acceleration =
        clipNorm(mapped_acceleration, cfg_.acc_norm_limit_m_s2);

    if (!finiteVector(desired_acceleration))
    {
        return false;
    }

    output.desired_acceleration = desired_acceleration;

    auto& dbg = output.debug;
    dbg.uav_positions_ned = input.uav_positions_ned;
    dbg.uav_velocities_ned = input.uav_velocities_ned;
    dbg.payload_position_ned = input.payload_position_ned;
    dbg.payload_velocity_ned = input.payload_velocity_ned;
    dbg.payload_target_ned = input.payload_target_ned;
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
    populateVirtualDebugState(input, state, dbg);
    return true;
}

}  // namespace control
}  // namespace swarm_planner
