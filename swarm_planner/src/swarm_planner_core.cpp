#include "swarm_planner/planner_core.h"
#include "swarm_planner/planner_utils.h"

#include <algorithm>
#include <cmath>

namespace swarm_planner {
namespace control {

bool SwarmPlannerCore::initialize(const Config& cfg)
{
    // initialize 会完整替换控制参数，并清空所有跨周期内部状态，
    // 确保重新加载参数后不会继承上一轮飞行任务的积分量或观测器估计值。
    cfg_ = cfg;
    initialized_ = false;
    reset();

    // 目前只对 CFO 的边界层参数做显式健壮性检查；其余参数由控制逻辑自然约束。
    const bool cfg_valid =
        !cfg_.cfo.enable || (std::isfinite(cfg_.cfo.phi) && cfg_.cfo.phi > 0.0);

    if (!cfg_valid || !loadAndValidateRestLengths())
    {
        return false;
    }

    initialized_ = true;
    return true;
}

void SwarmPlannerCore::reset()
{
    // 这里重置的是“运行态”而不是“配置态”。
    // 参数 cfg_ 保留，便于节点在不中断参数配置的情况下重新开始一次控制任务。
    structure_locked_ = false;
    rest_lengths_.setZero();
    velocity_integral_.setZero();
    prev_velocity_error_.setZero();
    prev_velocity_error_valid_ = false;
    cfo_hat_v_parallel_ = 0.0;
    cfo_hat_d_parallel_ = 0.0;
}

bool SwarmPlannerCore::finiteVector(const Vector3& v)
{
    // 下游会把期望加速度直接送给控制器，因此这里必须防止任意一个轴出现 NaN/Inf。
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

double SwarmPlannerCore::sat(const double x)
{
    // CFO 使用标准饱和函数代替 sign，减轻误差接近零时的抖振。
    return std::clamp(x, -1.0, 1.0);
}

Vector3 SwarmPlannerCore::clipNorm(const Vector3& v, const double max_norm)
{
    // 用总模长限幅而不是逐轴限幅，避免改变指令方向。
    if (max_norm <= 0.0)
    {
        return Vector3::Zero();
    }

    const double norm = v.norm();
    if (!std::isfinite(norm) || norm <= max_norm)
    {
        return v;
    }

    return v * (max_norm / norm);
}

bool SwarmPlannerCore::loadAndValidateRestLengths()
{
    // rest_lengths_override 由参数服务器提供，按行优先展开成 kNumNodes x kNumNodes 矩阵。
    // 只有在尺寸、对角线和对称性都满足要求时，才允许结构控制生效。
    const auto& v = cfg_.rest_lengths_override;
    if (v.size() != static_cast<size_t>(kNumNodes * kNumNodes) ||
        !planner_utils::loadRestLengthMatrix(v, rest_lengths_) ||
        !planner_utils::validateRestLengthDiagonal(rest_lengths_) ||
        !planner_utils::validateRestLengthSymmetry(rest_lengths_))
    {
        return false;
    }
    structure_locked_ = true;
    return true;
}

bool SwarmPlannerCore::validateInput(const Input& input) const
{
    // core 只验证本函数必需的最低条件；
    // 传感器新鲜度、坐标转换等更高层约束由 node / state_actions 负责。
    return initialized_ && input.self_index >= 0 && input.self_index < kNumUavs;
}

SwarmPlannerCore::VirtualState SwarmPlannerCore::buildVirtualState(
    const Input& input,
    const double h_u) const
{
    VirtualState state;

    // 虚拟网络总共有 5 个节点：
    // 0~2 是三架 UAV 对应的虚拟节点；
    // 3 是位于 payload 正上方 h_u 的“虚拟连接点”；
    // 4 是 payload 本体。
    state.q[kNumUavs + 1] = input.payload_position_ned;
    state.qdot[kNumUavs + 1] = input.payload_velocity_ned;
    state.q[kNumUavs] = input.payload_position_ned - Vector3(0.0, 0.0, h_u);
    state.qdot[kNumUavs] = input.payload_velocity_ned;

    for (int i = 0; i < kNumUavs; ++i)
    {
        // beta = dz / h_u 是文献里从虚拟网络回映到真实 UAV 的尺度因子。
        // 这里默认每架 UAV 与 payload 存在非零高度差；若 dz 退化到 0，会导致模型奇异。
        const double dz =
            std::abs(input.payload_position_ned.z() - input.uav_positions_ned[i].z());
        const double alpha = h_u / dz;
        state.beta[i] = dz / h_u;
        state.q[i] = input.payload_position_ned +
                     alpha * (input.uav_positions_ned[i] - input.payload_position_ned);
        state.qdot[i] = input.payload_velocity_ned +
                        alpha * (input.uav_velocities_ned[i] - input.payload_velocity_ned);
    }

    return state;
}

Vector3 SwarmPlannerCore::computePassiveNetworkForce(
    const int self_index,
    const VirtualState& state) const
{
    const Vector3& q_i = state.q[self_index];
    const Vector3& qdot_i = state.qdot[self_index];

    // 被动网络力由三部分组成：
    // 1) 弹簧项：把节点对之间的实际距离拉回参考结构长度；
    // 2) 阻尼项：抑制节点之间的相对速度；
    // 3) 摩擦项：给当前节点一个与自身速度相关的耗散力。
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
        // 该写法等价于沿 diff 方向施加 Hooke 型恢复力，
        // 当 length == rest_lengths 时该节点对的弹簧项为零。
        spring_force += cfg_.spring_k *
                        (1.0 - rest_lengths_(self_index, j) / length) * diff;

        // 阻尼项不依赖几何距离，只对相对速度做线性抑制。
        damping_force += cfg_.damping_c1 * (qdot_i - state.qdot[j]);
    }

    return spring_force + damping_force + cfg_.friction_c2 * qdot_i;
}

Vector3 SwarmPlannerCore::computeTrackingInput(
    const Vector3& qdot_i,
    const double dt,
    const Vector3& desired_payload_velocity)
{
    // 任务层当前只做一件事：让 payload 以一阶形式逼近目标点，
    // 再把该期望速度交给虚拟节点速度 PID 生成控制输入。
    const Vector3 velocity_error = qdot_i - desired_payload_velocity;
    const Vector3 velocity_error_derivative =
        planner_utils::computeVelocityErrorDerivative(
            velocity_error, prev_velocity_error_, dt, prev_velocity_error_valid_);

    if (dt > 0.0)
    {
        // 积分项只在有效离散步长下更新，防止首次运行或异常时钟跳变污染状态。
        velocity_integral_ += velocity_error * dt;
        planner_utils::clampIntegral(velocity_integral_, std::abs(cfg_.integral_limit));
        prev_velocity_error_ = velocity_error;
        prev_velocity_error_valid_ = true;
    }

    return -cfg_.vel_pid_kp * velocity_error
           - cfg_.vel_pid_ki * velocity_integral_
           - cfg_.vel_pid_kd * velocity_error_derivative;
}

SwarmPlannerCore::CfoResult SwarmPlannerCore::computeCfoAcceleration(
    const Input& input,
    const double mass,
    const double dt)
{
    CfoResult result;
    const int i = input.self_index;
    const Vector3 cable_vector = input.payload_position_ned - input.uav_positions_ned[i];
    const double cable_length = cable_vector.norm();
    // CFO 只补偿缆绳方向上的未知项；若缆绳太短、未启用功能或 dt 非法，则直接跳过。
    if (!planner_utils::cfoReadyForUpdate(cfg_, cable_length, dt))
    {
        return result;
    }

    const Vector3 cable_direction = cable_vector / cable_length;
    // 沿缆绳方向分解速度和输入，构造一维观测器。
    const double v_parallel = input.uav_velocities_ned[i].dot(cable_direction);
    const Vector3 gravity_force(0.0, 0.0, -mass * cfg_.gravity);
    const double u_parallel =
        (input.previous_thrust_vector + gravity_force).dot(cable_direction) / mass;
    const double error = v_parallel - cfo_hat_v_parallel_;

    // cfo_hat_v_parallel_ 估计平行于缆绳的速度，
    // cfo_hat_d_parallel_ 估计未建模扰动/外力加速度项。
    cfo_hat_v_parallel_ +=
        dt * (u_parallel + cfo_hat_d_parallel_ + cfg_.cfo.l1 * error);
    cfo_hat_d_parallel_ +=
        dt * (cfg_.cfo.l2 * sat(error / cfg_.cfo.phi));

    // 观测到的扰动被转换成沿缆绳反方向的补偿拉力，并受最大拉力上限约束。
    const double tau =
        std::clamp(-mass * cfo_hat_d_parallel_, 0.0, cfg_.cfo.fmax_n);
    result.acceleration = tau * (-cable_direction) / mass;
    result.used = true;
    return result;
}

bool SwarmPlannerCore::compute(const Input& input, Output& output)
{
    output = Output{};
    // 输入不完整或 self_index 非法时，直接拒绝本次计算。
    if (!validateInput(input))
    {
        return false;
    }

    // 当前实现固定按 200 Hz 控制周期离散化，避免运行时 dt 抖动影响积分/微分项。
    constexpr double dt_used = planner_utils::kFixedComputeDtS;

    // 将真实 UAV/payload 状态映射到内部虚拟节点网络。
    // 后续所有结构控制都在这组虚拟节点上完成，再映射回真实机体。
    const VirtualState state = buildVirtualState(input, cfg_.h_u_m);

    // 结构参考长度必须先成功加载，否则弹簧网络没有目标形状可跟踪。
    if (!structure_locked_)
    {
        return false;
    }

    const int self_index = input.self_index;
    const double mass = input.mass;
    // 结构项负责维持虚拟编队几何关系。
    const Vector3 passive_force = computePassiveNetworkForce(self_index, state);

    // 任务项先由 payload 位置误差生成期望速度，再由速度 PID 形成控制输入。
    const Vector3 desired_payload_velocity =
        planner_utils::computeDesiredPayloadVelocity(input, cfg_);

    // 跟踪项作用在当前 UAV 对应的虚拟节点速度上，而不是直接作用在 payload 本体上。
    const Vector3 tracking_input =
        computeTrackingInput(state.qdot[self_index], dt_used, desired_payload_velocity);
    // 虚拟加速度 = 结构约束项 + 任务跟踪项，再按 beta 映射回当前真实 UAV。
    const Vector3 virtual_acceleration = -passive_force / mass + tracking_input / mass;

    // beta 映射用于恢复真实 UAV 与虚拟节点之间的纵向尺度差。
    const Vector3 mapped_acceleration = state.beta[self_index] * virtual_acceleration;
    // CFO 是沿缆绳方向的附加补偿项，可选开启。
    const CfoResult cfo = computeCfoAcceleration(input, mass, dt_used);

    // 最终指令由结构项、任务项和 CFO 补偿叠加而成。
    Vector3 desired_acceleration = mapped_acceleration + cfo.acceleration;
    desired_acceleration = clipNorm(desired_acceleration, cfg_.acc_norm_limit_m_s2);
    // 输出前再做一次数值健壮性检查，避免 NaN/Inf 进入下游控制器。
    if (!finiteVector(desired_acceleration))
    {
        return false;
    }

    output.desired_acceleration = desired_acceleration;
    output.valid = true;
    output.used_cfo = cfo.used;

    // debug 字段完整保留中间量，便于在线分析每一步控制量来源。
    auto& debug = output.debug;
    debug.uav_positions_ned = input.uav_positions_ned;
    debug.uav_velocities_ned = input.uav_velocities_ned;
    debug.payload_position_ned = input.payload_position_ned;
    debug.payload_velocity_ned = input.payload_velocity_ned;
    debug.payload_target_ned = input.payload_target_ned;
    debug.previous_thrust_vector = input.previous_thrust_vector;
    debug.virtual_positions_ned = state.q;
    debug.virtual_velocities_ned = state.qdot;
    debug.beta = state.beta;
    planner_utils::flattenRestLengths(rest_lengths_, debug.rest_lengths);
    debug.passive_force = passive_force;
    debug.tracking_input = tracking_input;
    debug.virtual_acceleration = virtual_acceleration;
    debug.mapped_acceleration = mapped_acceleration;
    debug.cfo_acceleration = cfo.acceleration;
    debug.desired_acceleration = desired_acceleration;
    debug.self_index = input.self_index;
    debug.mass = input.mass;
    debug.dt_input = input.dt;
    debug.dt_used = dt_used;
    debug.dt_valid_for_update = true;
    debug.structure_locked = structure_locked_;
    debug.used_cfo = cfo.used;
    debug.valid = true;
    return true;
}

}  // namespace control
}  // namespace swarm_planner
