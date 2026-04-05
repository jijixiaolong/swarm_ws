#ifndef SWARM_PLANNER_PLANNER_CORE_H_
#define SWARM_PLANNER_PLANNER_CORE_H_

#include <array>
#include <vector>

#include <Eigen/Dense>

#include "planner_types.h"

namespace swarm_planner {
namespace control {

class SwarmPlannerCore
{
public:
    // kNumUavs 与外层 swarm_planner::kNumUavs 保持一致，这里转为 int 以兼容 Eigen 模板。
    static constexpr int kNumUavs = static_cast<int>(swarm_planner::kNumUavs);
    static constexpr int kNumNodes = kNumUavs + 2;

    struct Config
    {
        double h_u_m{1.0};
        double spring_k{1.2};
        double spring_force_clamp{0.0};  // per-edge spring force clamp (N); 0 = disabled
        double damping_c1{0.8};
        double friction_c2{0.3};
        double vel_pid_kp{1.0};
        double vel_pid_ki{0.0};
        double vel_pid_kd{0.2};
        double payload_kp{1.2};
        double payload_ki{0.0};
        double acc_norm_limit_m_s2{6.0};
        double integral_limit{2.0};
        double payload_integral_limit{2.0};
        double payload_error_limit_xy_m{0.6};
        double payload_error_limit_z_m{0.5};
        double max_payload_velocity{0.0};  // desired payload velocity norm clamp (m/s); 0 = disabled
        double structure_side_length_m{0.0};
        std::vector<double> rest_lengths_override{};
    };

    struct Input
    {
        std::array<Vector3, kNumUavs> uav_positions_ned{};
        std::array<Vector3, kNumUavs> uav_velocities_ned{};
        Vector3 payload_position_ned{Vector3::Zero()};
        Vector3 payload_velocity_ned{Vector3::Zero()};
        Vector3 payload_target_ned{Vector3::Zero()};
        int self_index{-1};
        double mass{1.0};
    };

    struct DebugState
    {
        std::array<Vector3, kNumUavs> uav_positions_ned{};
        std::array<Vector3, kNumUavs> uav_velocities_ned{};
        Vector3 payload_position_ned{Vector3::Zero()};
        Vector3 payload_velocity_ned{Vector3::Zero()};
        Vector3 payload_target_ned{Vector3::Zero()};
        std::array<Vector3, kNumNodes> virtual_positions_ned{};
        std::array<Vector3, kNumNodes> virtual_velocities_ned{};
        std::array<double, kNumUavs> beta{};
        std::array<double, kNumNodes * kNumNodes> rest_lengths{};
        Vector3 passive_force{Vector3::Zero()};
        Vector3 tracking_input{Vector3::Zero()};
        Vector3 virtual_acceleration{Vector3::Zero()};
        Vector3 mapped_acceleration{Vector3::Zero()};
        Vector3 desired_acceleration{Vector3::Zero()};
        int self_index{-1};
        double mass{0.0};
        bool structure_locked{false};
        bool valid{false};
    };

    struct Output
    {
        Vector3 desired_acceleration{Vector3::Zero()};
        DebugState debug{};
    };

    // initialize: 加载配置并重新初始化所有运行态，返回 false 表示配置非法。
    bool initialize(const Config& cfg);
    // reset: 清空积分/微分等运行态，保留配置和结构参考长度，可在任务重置时调用。
    void reset();
    bool compute(const Input& input, Output& output);

    bool ready() const { return ready_; }
    const Config& config() const { return cfg_; }

private:
    using NodeVector = std::array<Vector3, kNumNodes>;

    struct VirtualState
    {
        NodeVector q{};
        NodeVector qdot{};
        std::array<double, kNumUavs> beta{};
    };

    static bool finiteVector(const Vector3& v);
    static Vector3 clipNorm(const Vector3& v, double max_norm);
    void resetRuntimeState();
    bool loadRestLengths();
    bool validateInput(const Input& input) const;
    bool buildVirtualState(const Input& input, double h_u, VirtualState& state) const;
    Vector3 computeDesiredPayloadVelocity(const Input& input, double dt);
    Vector3 computePassiveNetworkForce(int self_index, const VirtualState& state) const;
    Vector3 computeTrackingInput(
        const Vector3& qdot_i,
        double dt,
        const Vector3& desired_payload_velocity);

    Config cfg_{};
    bool ready_{false};
    Eigen::Matrix<double, kNumNodes, kNumNodes> rest_lengths_{
        Eigen::Matrix<double, kNumNodes, kNumNodes>::Zero()};
    Vector3 payload_position_integral_{Vector3::Zero()};
    Vector3 velocity_integral_{Vector3::Zero()};
    Vector3 prev_velocity_error_{Vector3::Zero()};
    bool prev_velocity_error_valid_{false};
};

}  // namespace control
}  // namespace swarm_planner

#endif  // SWARM_PLANNER_PLANNER_CORE_H_
