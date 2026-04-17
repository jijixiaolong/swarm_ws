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
constexpr double kGravityMps2 = 9.81;

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

double wrapAngle(double angle_rad)
{
    while (angle_rad > M_PI)
    {
        angle_rad -= 2.0 * M_PI;
    }
    while (angle_rad < -M_PI)
    {
        angle_rad += 2.0 * M_PI;
    }
    return angle_rad;
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

    const bool use_geometry = cfg_.rope_length_m > 0.0 && cfg_.uav_uav_distance_m > 0.0;
    if (cfg_.h_u_m <= 0.0 ||
        cfg_.rope_tension_max_n < 0.0 ||
        cfg_.rope_observer_l1 < 0.0 ||
        cfg_.rope_observer_l2 < 0.0 ||
        cfg_.rope_observer_phi < 0.0 ||
        cfg_.azimuth_hold_kp < 0.0 ||
        cfg_.azimuth_hold_kd < 0.0)
    {
        return false;
    }

    if (use_geometry)
    {
        if (!buildRestLengthsFromGeometry()) return false;
    }
    else if (!loadRestLengths())
    {
        return false;
    }

    resetRuntimeState();
    ready_ = true;
    return true;
}

void SwarmPlannerCore::reset()
{
    // reset 清空所有跨周期运行态（积分、微分、CFO 观测器），
    // 不触碰配置和已加载的结构参考长度，以便在不重新 initialize 的情况下完整重启一次任务。
    resetRuntimeState();
}

void SwarmPlannerCore::resetPayloadIntegral()
{
    // 仅清空 payload 位置积分，CFO 观测器状态（rope_axis_velocity_hat_ /
    // rope_axis_disturbance_hat_）保持不变，保留已收敛的绳张力估计。
    payload_position_integral_.setZero();
}

void SwarmPlannerCore::resetAzimuthReference()
{
    azimuth_reference_rad_.fill(0.0);
    azimuth_reference_valid_ = false;
}

void SwarmPlannerCore::resetRuntimeState()
{
    payload_position_integral_.setZero();
    velocity_integral_.setZero();
    prev_velocity_error_.setZero();
    rope_axis_velocity_hat_.fill(0.0);
    rope_axis_disturbance_hat_.fill(0.0);
    azimuth_reference_rad_.fill(0.0);
    azimuth_reference_valid_ = false;
    prev_velocity_error_valid_ = false;
    prev_timestamp_valid_ = false;
    prev_timestamp_s_ = 0.0;
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

bool SwarmPlannerCore::buildRestLengthsFromGeometry()
{
    // 三棱锥几何（UAV 等边三角形 + payload 正下方）：
    //   side   = uav_uav_distance_m    真实 UAV-UAV 边长
    //   radius = side / √3             UAV 到三角形外心水平距离
    //   rope   = rope_length_m         物理绳长（yaml 显式配置）
    //   dz_ref = √(rope² - radius²)    设计高度差（一次性计算，固定）
    //   alpha  = h_u_m / dz_ref        虚拟空间投影缩放比（固定）
    const double side   = cfg_.uav_uav_distance_m;
    const double radius = side / std::sqrt(3.0);
    const double rope   = cfg_.rope_length_m;
    const double dz_ref_sq = rope * rope - radius * radius;

    if (dz_ref_sq <= 0.0)
    {
        return false;  // 绳长不足以撑开三棱锥，几何非法
    }

    const double dz_ref = std::sqrt(dz_ref_sq);
    const double alpha  = cfg_.h_u_m / dz_ref;

    const double virt_uav_uav     = alpha * side;
    const double virt_uav_hub     = alpha * radius;
    const double virt_uav_payload = alpha * rope;

    rest_lengths_.setZero();
    constexpr int hub_idx     = kNumUavs;
    constexpr int payload_idx = kNumUavs + 1;

    for (int i = 0; i < kNumUavs; ++i)
    {
        for (int j = i + 1; j < kNumUavs; ++j)
        {
            rest_lengths_(i, j) = rest_lengths_(j, i) = virt_uav_uav;
        }
        rest_lengths_(i, hub_idx)     = rest_lengths_(hub_idx, i)     = virt_uav_hub;
        rest_lengths_(i, payload_idx) = rest_lengths_(payload_idx, i) = virt_uav_payload;
    }
    rest_lengths_(hub_idx, payload_idx) = rest_lengths_(payload_idx, hub_idx) = cfg_.h_u_m;
    return true;
}

bool SwarmPlannerCore::validateInput(const Input& input) const
{
    return ready_ && input.self_index >= 0 && input.self_index < kNumUavs && input.mass > 0.0;
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

    // 使用三机平均 dz 得到共享 α/β，避免各机独立 β 在高度不对称时形成正反馈（虚拟空间几何自洽 + 映射一致）。
    double sum_dz = 0.0;
    for (int i = 0; i < kNumUavs; ++i)
    {
        const double dz =
            std::abs(input.payload_position_ned.z() - input.uav_positions_ned[i].z());
        if (dz < 1e-4)
        {
            return false;
        }
        sum_dz += dz;
    }
    const double mean_dz = sum_dz / static_cast<double>(kNumUavs);
    const double alpha_shared = h_u / mean_dz;
    const double beta_shared = mean_dz / h_u;

    for (int i = 0; i < kNumUavs; ++i)
    {
        state.beta[i] = beta_shared;
        state.q[i] = input.payload_position_ned +
                     alpha_shared * (input.uav_positions_ned[i] - input.payload_position_ned);
        state.qdot[i] = input.payload_velocity_ned +
                        alpha_shared * (input.uav_velocities_ned[i] - input.payload_velocity_ned);
    }

    return true;
}

SwarmPlannerCore::RopeObserverEstimate SwarmPlannerCore::estimateRopeTensionCfo(
    const Input& input,
    const double dt,
    const Vector3& base_acceleration)
{
    RopeObserverEstimate estimate;
    if (dt <= 0.0)
    {
        return estimate;
    }

    const int self_index = input.self_index;
    const Vector3 rope_direction_ned =
        input.payload_position_ned - input.uav_positions_ned[self_index];
    const double rope_length = rope_direction_ned.norm();
    if (rope_length < 1e-4)
    {
        return estimate;
    }

    const Vector3 rope_axis_ned = rope_direction_ned / rope_length;
    const double measured_rope_axis_velocity =
        input.uav_velocities_ned[self_index].dot(rope_axis_ned);

    const double velocity_hat = rope_axis_velocity_hat_[self_index];
    const double disturbance_hat = rope_axis_disturbance_hat_[self_index];
    const double error = measured_rope_axis_velocity - velocity_hat;

    // 观测器输入 = UAV 实际被命令执行的沿绳加速度 = base·r + rope_comp·r。
    // rope_comp 在本拍被施加到 UAV，其沿绳分量等于 -prev_comp_along_rope（与 rope_axis 同符号的张力推 UAV 远离 payload）。
    // 若仅用 base·r 作输入，UAV 真实动力学 dv/dt = base·r + T/m - d_hat 与观测器模型 dv_hat/dt = base·r + d_hat
    // 在稳态条件下解出 d_hat = T/(2m)，造成"估计张力恒为真实值一半"的系统性低估——对应当前
    // 补偿不足、payload 高度静差。
    // 修正后：input = base·r − prev_comp_along_rope，稳态 d_hat → T/m，3 架 UAV 垂直分量之和 ≈ 负载重力。
    double prev_comp_along_rope = 0.0;
    if (cfg_.rope_tension_compensation_enabled)
    {
        double prev_tension = std::max(input.mass * disturbance_hat, 0.0);
        if (cfg_.rope_tension_max_n > 0.0)
        {
            prev_tension = std::min(prev_tension, cfg_.rope_tension_max_n);
        }
        prev_comp_along_rope = prev_tension / input.mass;
    }
    const double observer_input_rope_axis_acceleration =
        base_acceleration.dot(rope_axis_ned) - prev_comp_along_rope;

    const double sat = (cfg_.rope_observer_phi > 1e-6)
                           ? std::clamp(error / cfg_.rope_observer_phi, -1.0, 1.0)
                           : (error >= 0.0 ? 1.0 : -1.0);
    const double next_velocity_hat =
        velocity_hat + dt * (observer_input_rope_axis_acceleration +
                             disturbance_hat + cfg_.rope_observer_l1 * error);
    const double next_disturbance_hat =
        disturbance_hat + dt * (cfg_.rope_observer_l2 * sat);

    rope_axis_velocity_hat_[self_index] = next_velocity_hat;
    rope_axis_disturbance_hat_[self_index] = next_disturbance_hat;

    double estimated_rope_tension_n = std::max(input.mass * next_disturbance_hat, 0.0);
    if (cfg_.rope_tension_max_n > 0.0)
    {
        estimated_rope_tension_n =
            std::clamp(estimated_rope_tension_n, 0.0, cfg_.rope_tension_max_n);
    }

    estimate.rope_direction_ned = rope_axis_ned;
    estimate.measured_rope_axis_velocity = measured_rope_axis_velocity;
    estimate.observer_input_rope_axis_acceleration = observer_input_rope_axis_acceleration;
    estimate.rope_axis_velocity_hat = next_velocity_hat;
    estimate.rope_axis_disturbance_hat = next_disturbance_hat;
    estimate.rope_axis_observer_error = error;
    estimate.estimated_rope_tension_n = estimated_rope_tension_n;
    estimate.estimated_rope_force_ned = estimated_rope_tension_n * rope_axis_ned;
    if (cfg_.rope_tension_compensation_enabled && estimated_rope_tension_n > 0.0)
    {
        estimate.rope_compensation_acceleration = -estimate.estimated_rope_force_ned / input.mass;
    }
    estimate.valid = true;
    return estimate;
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

    const bool z_i_enabled = cfg_.payload_ki > 0.0;
    const bool xy_i_enabled = cfg_.payload_ki_xy > 0.0;

    if (!z_i_enabled && !xy_i_enabled)
    {
        payload_position_integral_.setZero();
        return -cfg_.payload_kp * position_error;
    }

    const double i_limit = std::abs(cfg_.payload_integral_limit);

    if (dt > 0.0)
    {
        // Z：消除重力/高度静差（原有行为）。
        if (z_i_enabled)
        {
            payload_position_integral_.z() += position_error.z() * dt;
            payload_position_integral_.z() = std::clamp(
                payload_position_integral_.z(), -i_limit, i_limit);
        }
        else
        {
            payload_position_integral_.z() = 0.0;
        }

        // XY：受限积分，死区门控防止把 0.6 Hz 摆动能量积进去。
        if (xy_i_enabled)
        {
            const double err_xy = std::hypot(position_error.x(), position_error.y());
            const double vel_xy = std::hypot(
                input.payload_velocity_ned.x(), input.payload_velocity_ned.y());
            const bool err_outside_band =
                err_xy > cfg_.payload_i_err_deadband_xy_m;
            const bool vel_inside_band =
                cfg_.payload_i_vel_deadband_xy_m_s <= 0.0 ||
                vel_xy < cfg_.payload_i_vel_deadband_xy_m_s;

            if (err_outside_band && vel_inside_band)
            {
                payload_position_integral_.x() += position_error.x() * dt;
                payload_position_integral_.y() += position_error.y() * dt;
                payload_position_integral_.x() = std::clamp(
                    payload_position_integral_.x(), -i_limit, i_limit);
                payload_position_integral_.y() = std::clamp(
                    payload_position_integral_.y(), -i_limit, i_limit);
            }
        }
        else
        {
            payload_position_integral_.x() = 0.0;
            payload_position_integral_.y() = 0.0;
        }
    }

    Vector3 integral_feedback;
    integral_feedback.x() = cfg_.payload_ki_xy * payload_position_integral_.x();
    integral_feedback.y() = cfg_.payload_ki_xy * payload_position_integral_.y();
    integral_feedback.z() = cfg_.payload_ki    * payload_position_integral_.z();

    return -cfg_.payload_kp * position_error - integral_feedback;
}

SwarmPlannerCore::PassiveForceBreakdown SwarmPlannerCore::computePassiveNetworkForce(
    const int self_index,
    const VirtualState& state) const
{
    const Vector3& q_i = state.q[self_index];
    const Vector3& qdot_i = state.qdot[self_index];

    // 弹簧：将节点对拉回参考结构长度；阻尼：抑制节点间相对速度；摩擦：耗散自身速度。
    PassiveForceBreakdown breakdown;
    for (int j = 0; j < kNumNodes; ++j)
    {
        if (j == self_index)
        {
            continue;
        }

        const Vector3 diff = q_i - state.q[j];
        const double length = diff.norm();
        if (length < 1e-6)
        {
            continue;
        }
        breakdown.spring_force +=
            cfg_.spring_k * (1.0 - rest_lengths_(self_index, j) / length) * diff;
        breakdown.damping_force += cfg_.damping_c1 * (qdot_i - state.qdot[j]);
    }

    breakdown.friction_force = cfg_.friction_c2 * qdot_i;
    breakdown.total_force =
        breakdown.spring_force + breakdown.damping_force + breakdown.friction_force;
    return breakdown;
}

void SwarmPlannerCore::initializeAzimuthReference(const VirtualState& state)
{
    constexpr int payload_index = kNumUavs + 1;
    for (int i = 0; i < kNumUavs; ++i)
    {
        const Vector3 rel = state.q[i] - state.q[payload_index];
        azimuth_reference_rad_[i] = std::atan2(rel.y(), rel.x());
    }
    azimuth_reference_valid_ = true;
}

Vector3 SwarmPlannerCore::computeAzimuthStabilizationAcceleration(
    const int self_index,
    const VirtualState& state) const
{
    if (cfg_.azimuth_hold_kp <= 0.0 && cfg_.azimuth_hold_kd <= 0.0)
    {
        return Vector3::Zero();
    }

    constexpr int payload_index = kNumUavs + 1;
    const Vector3 rel = state.q[self_index] - state.q[payload_index];
    const double radius_xy = std::hypot(rel.x(), rel.y());
    if (radius_xy <= 1e-6)
    {
        return Vector3::Zero();
    }

    const Vector3 rel_vel = state.qdot[self_index] - state.qdot[payload_index];
    const Vector3 radial_dir(rel.x() / radius_xy, rel.y() / radius_xy, 0.0);
    const Vector3 tangential_dir(-radial_dir.y(), radial_dir.x(), 0.0);
    const double tangential_speed = rel_vel.dot(tangential_dir);

    double tangential_acceleration = -cfg_.azimuth_hold_kd * tangential_speed;
    if (azimuth_reference_valid_)
    {
        const double theta = std::atan2(rel.y(), rel.x());
        const double theta_error = wrapAngle(theta - azimuth_reference_rad_[self_index]);
        tangential_acceleration += -cfg_.azimuth_hold_kp * (radius_xy * theta_error);
    }

    return tangential_acceleration * tangential_dir;
}


SwarmPlannerCore::TrackingInputBreakdown SwarmPlannerCore::computeTrackingInput(
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

    TrackingInputBreakdown breakdown;
    breakdown.proportional_term = -cfg_.vel_pid_kp * velocity_error;
    breakdown.integral_term = -cfg_.vel_pid_ki * velocity_integral_;
    breakdown.derivative_term = -cfg_.vel_pid_kd * derivative;
    breakdown.total_force =
        breakdown.proportional_term + breakdown.integral_term + breakdown.derivative_term;
    return breakdown;
}

bool SwarmPlannerCore::compute(const Input& input, Output& output)
{
    output = Output{};

    if (!validateInput(input))
    {
        return false;
    }

    // 优先使用调用方传入的时间戳计算真实 dt，避免硬编码频率与实际调用频率不符导致 D 项放大。
    // 退回条件：首次调用 / 时间戳未填（0）/ dt 异常（< 1ms 或 > 200ms）。
    double dt = kFixedComputeDtS;
    if (input.timestamp_s > 0.0 && prev_timestamp_valid_)
    {
        const double measured_dt = input.timestamp_s - prev_timestamp_s_;
        if (measured_dt >= 0.001 && measured_dt <= 0.2)
        {
            dt = measured_dt;
        }
    }
    if (input.timestamp_s > 0.0)
    {
        prev_timestamp_s_ = input.timestamp_s;
        prev_timestamp_valid_ = true;
    }

    VirtualState state;
    if (!buildVirtualState(input, cfg_.h_u_m, state))
    {
        return false;
    }
    const int self_index = input.self_index;
    const double mass = input.mass;

    if (!azimuth_reference_valid_ &&
        (cfg_.azimuth_hold_kp > 0.0 || cfg_.azimuth_hold_kd > 0.0))
    {
        initializeAzimuthReference(state);
    }

    const PassiveForceBreakdown passive_force_breakdown =
        computePassiveNetworkForce(self_index, state);
    const Vector3 desired_payload_velocity = computeDesiredPayloadVelocity(input, dt);
    const TrackingInputBreakdown tracking_breakdown =
        computeTrackingInput(state.qdot[self_index], dt, desired_payload_velocity);
    const Vector3 virtual_acceleration =
        -passive_force_breakdown.total_force / mass + tracking_breakdown.total_force / mass;
    const Vector3 mapped_acceleration = state.beta[self_index] * virtual_acceleration;
    const Vector3 azimuth_stabilization_acceleration =
        computeAzimuthStabilizationAcceleration(self_index, state);

    // 基础期望加速度（不含绳补偿），作为 CFO 观测器参考输入（方案C）
    const Vector3 base_acceleration =
        mapped_acceleration + azimuth_stabilization_acceleration;

    const RopeObserverEstimate rope_observer_estimate =
        estimateRopeTensionCfo(input, dt, base_acceleration);
    Vector3 rope_compensation_acceleration = Vector3::Zero();
    Vector3 estimated_rope_force_ned = Vector3::Zero();
    double estimated_rope_tension_n = 0.0;
    if (rope_observer_estimate.valid)
    {
        estimated_rope_force_ned = rope_observer_estimate.estimated_rope_force_ned;
        estimated_rope_tension_n = rope_observer_estimate.estimated_rope_tension_n;
        rope_compensation_acceleration = rope_observer_estimate.rope_compensation_acceleration;
    }

    const Vector3 unclipped_desired_acceleration =
        base_acceleration + rope_compensation_acceleration;
    const Vector3 desired_acceleration =
        clipNorm(unclipped_desired_acceleration, cfg_.acc_norm_limit_m_s2);

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
    dbg.desired_payload_velocity = desired_payload_velocity;
    dbg.prev_thrust_ned = input.prev_thrust_ned;
    dbg.virtual_positions_ned = state.q;
    dbg.virtual_velocities_ned = state.qdot;
    dbg.beta = state.beta;
    flattenRestLengths(rest_lengths_, dbg.rest_lengths);
    dbg.spring_force = passive_force_breakdown.spring_force;
    dbg.damping_force = passive_force_breakdown.damping_force;
    dbg.friction_force = passive_force_breakdown.friction_force;
    dbg.passive_force = passive_force_breakdown.total_force;
    dbg.tracking_p_term = tracking_breakdown.proportional_term;
    dbg.tracking_i_term = tracking_breakdown.integral_term;
    dbg.tracking_d_term = tracking_breakdown.derivative_term;
    dbg.tracking_input = tracking_breakdown.total_force;
    dbg.virtual_acceleration = virtual_acceleration;
    dbg.mapped_acceleration = mapped_acceleration;
    dbg.azimuth_stabilization_acceleration = azimuth_stabilization_acceleration;
    dbg.separation_acceleration = Vector3::Zero();
    dbg.base_acceleration = base_acceleration;
    dbg.rope_direction_ned = rope_observer_estimate.rope_direction_ned;
    dbg.estimated_rope_force_ned = estimated_rope_force_ned;
    dbg.rope_compensation_acceleration = rope_compensation_acceleration;
    dbg.unclipped_desired_acceleration = unclipped_desired_acceleration;
    dbg.desired_acceleration = desired_acceleration;
    dbg.dt_used = dt;
    dbg.measured_rope_axis_velocity = rope_observer_estimate.measured_rope_axis_velocity;
    dbg.observer_input_rope_axis_acceleration =
        rope_observer_estimate.observer_input_rope_axis_acceleration;
    dbg.rope_axis_velocity_hat = rope_observer_estimate.rope_axis_velocity_hat;
    dbg.rope_axis_disturbance_hat = rope_observer_estimate.rope_axis_disturbance_hat;
    dbg.rope_axis_observer_error = rope_observer_estimate.rope_axis_observer_error;
    dbg.estimated_rope_tension_n = estimated_rope_tension_n;
    dbg.self_index = input.self_index;
    dbg.mass = input.mass;
    dbg.prev_thrust_valid = input.prev_thrust_valid;
    dbg.structure_locked = ready_;
    dbg.rope_observer_valid = rope_observer_estimate.valid;
    dbg.valid = true;
    return true;
}

}  // namespace control
}  // namespace swarm_planner
