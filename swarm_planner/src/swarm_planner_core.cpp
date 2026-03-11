#include "swarm_planner/swarm_planner_core.h"

#include <algorithm>
#include <cmath>

namespace swarm_planner {
namespace control {

bool SwarmPlannerCore::validateConfig(const Config& cfg)
{
    if (!std::isfinite(cfg.gravity) || cfg.gravity <= 0.0) return false;
    if (!std::isfinite(cfg.h_u_m) || cfg.h_u_m <= 0.0) return false;
    if (!std::isfinite(cfg.spring_k) || cfg.spring_k < 0.0) return false;
    if (!std::isfinite(cfg.damping_c1) || cfg.damping_c1 < 0.0) return false;
    if (!std::isfinite(cfg.friction_c2) || cfg.friction_c2 < 0.0) return false;
    if (!std::isfinite(cfg.vel_pid_kp) || cfg.vel_pid_kp < 0.0) return false;
    if (!std::isfinite(cfg.vel_pid_ki) || cfg.vel_pid_ki < 0.0) return false;
    if (!std::isfinite(cfg.vel_pid_kd) || cfg.vel_pid_kd < 0.0) return false;
    if (!std::isfinite(cfg.payload_kp) || cfg.payload_kp < 0.0) return false;
    if (!std::isfinite(cfg.acc_norm_limit_m_s2) || cfg.acc_norm_limit_m_s2 <= 0.0) return false;
    if (!std::isfinite(cfg.dt_min_s) || cfg.dt_min_s <= 0.0) return false;
    if (!std::isfinite(cfg.dt_max_s) || cfg.dt_max_s < cfg.dt_min_s) return false;
    if (!std::isfinite(cfg.epsilon) || cfg.epsilon <= 0.0) return false;
    if (!std::isfinite(cfg.cfo.l1) || cfg.cfo.l1 < 0.0) return false;
    if (!std::isfinite(cfg.cfo.l2) || cfg.cfo.l2 < 0.0) return false;
    if (!std::isfinite(cfg.cfo.phi) || cfg.cfo.phi <= 0.0) return false;
    if (!std::isfinite(cfg.cfo.fmax_n) || cfg.cfo.fmax_n < 0.0) return false;
    if (!std::isfinite(cfg.cfo.l_min_m) || cfg.cfo.l_min_m < 0.0) return false;

    return true;
}

bool SwarmPlannerCore::initialize(const Config& cfg)
{
    if (!validateConfig(cfg))
    {
        return false;
    }

    cfg_ = cfg;
    initialized_ = false;
    clearRuntimeState();
    structure_locked_ = false;
    rest_lengths_.setZero();

    if (!applyConfiguredRestLengths())
    {
        return false;
    }

    initialized_ = true;
    return true;
}

void SwarmPlannerCore::reset()
{
    clearRuntimeState();
}

void SwarmPlannerCore::clearRuntimeState()
{
    velocity_integral_.setZero();
    prev_velocity_error_.setZero();
    prev_velocity_error_valid_ = false;
    cfo_hat_v_parallel_ = 0.0;
    cfo_hat_d_parallel_ = 0.0;
}

void SwarmPlannerCore::resetStructureReference()
{
    clearRuntimeState();
    structure_locked_ = false;
    rest_lengths_.setZero();

    if (!applyConfiguredRestLengths())
    {
        initialized_ = false;
    }
}

bool SwarmPlannerCore::finiteVector(const Vector3& v)
{
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

double SwarmPlannerCore::sat(double x)
{
    return std::clamp(x, -1.0, 1.0);
}

Vector3 SwarmPlannerCore::clipNorm(const Vector3& v, const double max_norm)
{
    if (max_norm <= 0.0)
    {
        return Vector3::Zero();
    }

    const double n = v.norm();
    if (!std::isfinite(n) || n <= max_norm)
    {
        return v;
    }

    if (n < 1e-9)
    {
        return Vector3::Zero();
    }

    return v * (max_norm / n);
}

bool SwarmPlannerCore::applyConfiguredRestLengths()
{
    structure_locked_ = false;
    rest_lengths_.setZero();

    if (cfg_.rest_lengths_override.empty())
    {
        return false;
    }

    if (cfg_.rest_lengths_override.size() != kNumNodes * kNumNodes)
    {
        return false;
    }

    const double tol = std::max(cfg_.epsilon, 1e-6);
    for (int row = 0; row < kNumNodes; ++row)
    {
        for (int col = 0; col < kNumNodes; ++col)
        {
            const double len = cfg_.rest_lengths_override[row * kNumNodes + col];
            if (!std::isfinite(len) || len < 0.0)
            {
                return false;
            }

            if (row == col)
            {
                if (std::abs(len) > tol)
                {
                    return false;
                }
                rest_lengths_(row, col) = 0.0;
                continue;
            }

            rest_lengths_(row, col) = len;
        }
    }

    for (int row = 0; row < kNumNodes; ++row)
    {
        for (int col = row + 1; col < kNumNodes; ++col)
        {
            if (std::abs(rest_lengths_(row, col) - rest_lengths_(col, row)) > tol)
            {
                return false;
            }
        }
    }

    structure_locked_ = true;
    return true;
}

bool SwarmPlannerCore::validateInput(const Input& input) const
{
    if (!initialized_)
    {
        return false;
    }

    if (input.self_index < 0 || input.self_index >= kNumUavs)
    {
        return false;
    }

    if (!std::isfinite(input.mass) || input.mass <= 0.0)
    {
        return false;
    }

    if (!std::isfinite(input.dt) || input.dt < 0.0)
    {
        return false;
    }

    for (int i = 0; i < kNumUavs; ++i)
    {
        if (!finiteVector(input.uav_positions_ned[i]) || !finiteVector(input.uav_velocities_ned[i]))
        {
            return false;
        }
    }

    return finiteVector(input.payload_position_ned) &&
           finiteVector(input.payload_velocity_ned) &&
           finiteVector(input.payload_target_ned) &&
           finiteVector(input.previous_thrust_vector);
}
SwarmPlannerCore::VirtualState SwarmPlannerCore::buildVirtualState(
    const Input& input,
    const double eps,
    const double h_u) const
{
    VirtualState state;

    // The paper defines q_l as the load node and q_u as the center of a
    // virtual tray located h_u above the load. In NED, "above" means a
    // smaller z value, so the tray center subtracts the down-axis offset.
    state.q[kNumUavs + 1] = input.payload_position_ned;
    state.qdot[kNumUavs + 1] = input.payload_velocity_ned;
    state.q[kNumUavs] = input.payload_position_ned - Vector3(0.0, 0.0, h_u);
    state.qdot[kNumUavs] = input.payload_velocity_ned;

    for (int i = 0; i < kNumUavs; ++i)
    {
        const double dz = std::abs(input.payload_position_ned.z() - input.uav_positions_ned[i].z());
        const double alpha = h_u / std::max(dz, eps);
        // beta = |z_l - z_i| / |z_u - z_l| appears in the paper's mapping from
        // virtual-node acceleration back to the real UAV acceleration.
        state.beta[i] = dz / std::max(h_u, eps);
        state.q[i] = input.payload_position_ned +
                     alpha * (input.uav_positions_ned[i] - input.payload_position_ned);
        state.qdot[i] = input.payload_velocity_ned +
                        alpha * (input.uav_velocities_ned[i] - input.payload_velocity_ned);
    }

    return state;
}

Vector3 SwarmPlannerCore::computePassiveNetworkForce(
    const int self_index,
    const VirtualState& state,
    const double eps) const
{
    const Vector3 q_i = state.q[self_index];
    const Vector3 qdot_i = state.qdot[self_index];

    Vector3 spring_force = Vector3::Zero();
    Vector3 damping_force = Vector3::Zero();
    for (int j = 0; j < kNumNodes; ++j)
    {
        if (j == self_index)
        {
            continue;
        }

        const Vector3 diff = q_i - state.q[j];
        const double l_ij = diff.norm();
        if (l_ij > eps)
        {
            const double l0 = rest_lengths_(self_index, j);
            spring_force += cfg_.spring_k * (1.0 - l0 / l_ij) * diff;
        }

        damping_force += cfg_.damping_c1 * (qdot_i - state.qdot[j]);
    }

    const Vector3 friction_force = cfg_.friction_c2 * qdot_i;
    return spring_force + damping_force + friction_force;
}

Vector3 SwarmPlannerCore::computeTrackingInput(
    const Vector3& qdot_i,
    const double mass,
    const double dt,
    const Vector3& desired_payload_velocity,
    const Vector3& desired_payload_acceleration)
{
    const Vector3 velocity_error = qdot_i - desired_payload_velocity;

    Vector3 velocity_error_derivative = Vector3::Zero();
    if (dt > 0.0 && prev_velocity_error_valid_)
    {
        velocity_error_derivative = (velocity_error - prev_velocity_error_) / dt;
    }

    if (dt > 0.0)
    {
        velocity_integral_ += velocity_error * dt;
        const double integral_limit = std::abs(cfg_.integral_limit);
        if (integral_limit > 0.0)
        {
            for (int k = 0; k < 3; ++k)
            {
                velocity_integral_(k) =
                    std::clamp(velocity_integral_(k), -integral_limit, integral_limit);
            }
        }
        prev_velocity_error_ = velocity_error;
        prev_velocity_error_valid_ = true;
    }

    // u_i follows the paper's feed-forward plus PID feedback structure. In the
    // fixed-point case, v_d_dot comes from the payload position servo above.
    return mass * desired_payload_acceleration
           - cfg_.vel_pid_kp * velocity_error
           - cfg_.vel_pid_ki * velocity_integral_
           - cfg_.vel_pid_kd * velocity_error_derivative;
}

SwarmPlannerCore::CfoResult SwarmPlannerCore::computeCfoAcceleration(
    const Input& input,
    const double eps,
    const double mass,
    const bool dt_valid_for_update)
{
    CfoResult result;
    if (!cfg_.cfo.enable)
    {
        return result;
    }

    const int i = input.self_index;
    const Vector3 t_i = input.payload_position_ned - input.uav_positions_ned[i];
    const double t_norm = t_i.norm();
    // CFO estimates the cable-direction disturbance and only injects
    // tensile compensation, never compression.
    const bool cfo_ok = dt_valid_for_update && input.dt > 0.0 &&
                        t_norm >= std::max(cfg_.cfo.l_min_m, 0.0);
    if (!cfo_ok)
    {
        return result;
    }

    const Vector3 t0 = t_i / t_norm;
    const double v_parallel = input.uav_velocities_ned[i].dot(t0);
    const Vector3 G(0.0, 0.0, -mass * cfg_.gravity);
    const double u_parallel = (input.previous_thrust_vector + G).dot(t0) / mass;
    const double e = v_parallel - cfo_hat_v_parallel_;
    cfo_hat_v_parallel_ += input.dt * (u_parallel + cfo_hat_d_parallel_ + cfg_.cfo.l1 * e);
    const double phi = std::max(cfg_.cfo.phi, eps);
    cfo_hat_d_parallel_ += input.dt * (cfg_.cfo.l2 * sat(e / phi));
    const double tau = std::clamp(-mass * cfo_hat_d_parallel_, 0.0, cfg_.cfo.fmax_n);
    const Vector3 f_hat = tau * (-t0);
    result.acceleration = f_hat / mass;
    result.used = true;
    return result;
}

bool SwarmPlannerCore::compute(const Input& input, Output& output)
{
    // 先清空输出，避免输入校验失败或中间数值异常时，调用方误用上一次的控制结果。
    output = Output{};

    // 在更新积分器、结构参考等控制器内部状态之前，先拦截未初始化或非有限输入。
    if (!validateInput(input))
    {
        return false;
    }

    // 用下界保护几何量中的除法，同时只有当采样周期落在配置范围内时才允许时序状态更新。
    const double eps = std::max(cfg_.epsilon, 1e-6);
    const double h_u = std::max(cfg_.h_u_m, eps);
    const double mass = std::max(input.mass, eps);

    double dt = input.dt;
    const bool dt_valid_for_update = dt >= cfg_.dt_min_s && dt <= cfg_.dt_max_s;
    if (!dt_valid_for_update)
    {
        dt = 0.0;
    }

    // 根据当前载荷和无人机状态重建论文中的虚拟节点网络，先在虚拟域内完成控制计算。
    const VirtualState state = buildVirtualState(input, eps, h_u);

    if (!structure_locked_)
    {
        return false;
    }

    // 组合当前无人机的局部控制律：虚拟弹簧阻尼网络的被动耦合项，加上载荷定点跟踪项。
    const int i = input.self_index;
    const Vector3 passive_force = computePassiveNetworkForce(i, state, eps);
    const Vector3 desired_payload_velocity =
        -cfg_.payload_kp * (input.payload_position_ned - input.payload_target_ned);
    const Vector3 desired_payload_acceleration =
        -cfg_.payload_kp * input.payload_velocity_ned;
    const Vector3 u_i = computeTrackingInput(
        state.qdot[i],
        mass,
        dt,
        desired_payload_velocity,
        desired_payload_acceleration);

    // qdd_i 表示第 i 个虚拟节点的期望加速度，由被动网络稳定项和定点跟踪输入共同组成。
    const Vector3 qdd_i = -passive_force / mass + u_i / mass;

    // 文档中的映射是 a_i = qdd_l + beta * (qdd_i - qdd_l)。
    // 点保持场景下，qdd_l 由外环位置伺服生成的 payload 期望加速度给出；
    // 这样不会把所有无人机共享的载荷平移加速度错误放大为 beta 倍。
    const Vector3 qdd_l = desired_payload_acceleration;
    const Vector3 a_id = qdd_l + state.beta[i] * (qdd_i - qdd_l);
    const CfoResult cfo = computeCfoAcceleration(input, eps, mass, dt_valid_for_update);

    // 输出前先做限幅，并拒绝任何非有限结果，确保下游只接收到安全的加速度指令。
    Vector3 a_des = a_id + cfo.acceleration;
    a_des = clipNorm(a_des, cfg_.acc_norm_limit_m_s2);
    if (!finiteVector(a_des))
    {
        return false;
    }

    output.desired_acceleration = a_des;
    output.valid = true;
    output.used_cfo = cfo.used;
    output.debug.uav_positions_ned = input.uav_positions_ned;
    output.debug.uav_velocities_ned = input.uav_velocities_ned;
    output.debug.payload_position_ned = input.payload_position_ned;
    output.debug.payload_velocity_ned = input.payload_velocity_ned;
    output.debug.payload_target_ned = input.payload_target_ned;
    output.debug.previous_thrust_vector = input.previous_thrust_vector;
    output.debug.virtual_positions_ned = state.q;
    output.debug.virtual_velocities_ned = state.qdot;
    output.debug.beta = state.beta;
    for (int row = 0; row < kNumNodes; ++row)
    {
        for (int col = 0; col < kNumNodes; ++col)
        {
            output.debug.rest_lengths[row * kNumNodes + col] = rest_lengths_(row, col);
        }
    }
    output.debug.passive_force = passive_force;
    output.debug.tracking_input = u_i;
    output.debug.virtual_acceleration = qdd_i;
    output.debug.mapped_acceleration = a_id;
    output.debug.cfo_acceleration = cfo.acceleration;
    output.debug.desired_acceleration = a_des;
    output.debug.self_index = input.self_index;
    output.debug.mass = input.mass;
    output.debug.dt_input = input.dt;
    output.debug.dt_used = dt;
    output.debug.dt_valid_for_update = dt_valid_for_update;
    output.debug.structure_locked = structure_locked_;
    output.debug.used_cfo = cfo.used;
    output.debug.valid = true;
    return true;
}

}  // namespace control
}  // namespace swarm_planner
