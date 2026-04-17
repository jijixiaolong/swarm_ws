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
        double damping_c1{0.8};
        double friction_c2{0.3};
        double vel_pid_kp{1.0};
        double vel_pid_ki{0.0};
        double vel_pid_kd{0.2};
        double payload_kp{1.2};
        double payload_ki{0.0};
        // XY 位置积分增益；0 表示禁用（保持原先 Z-only 行为）。
        double payload_ki_xy{0.0};
        // XY 误差死区：|e_xy| 小于该值时冻结积分，避免极限环噪声被积分。
        double payload_i_err_deadband_xy_m{0.0};
        // XY 负载速度死区：|v_payload_xy| 超过该值时冻结积分（负载仍在摆动，勿积入摆动能量）。
        double payload_i_vel_deadband_xy_m_s{0.0};
        double acc_norm_limit_m_s2{6.0};
        double integral_limit{2.0};
        double payload_integral_limit{2.0};
        double payload_error_limit_xy_m{0.6};
        double payload_error_limit_z_m{0.5};
        bool rope_tension_compensation_enabled{false};
        double rope_tension_max_n{0.0};
        double rope_observer_l1{20.0};
        double rope_observer_l2{50.0};
        double rope_observer_phi{0.1};
        double azimuth_hold_kp{0.0};
        double azimuth_hold_kd{0.0};
        /// UAV 等边三角形边长（米）。与 rope_length_m 共同确定三棱锥几何。
        double uav_uav_distance_m{0.0};
        /// 物理绳长（米）。>0 且 uav_uav_distance_m>0 时，在 initialize() 中一次性
        /// 计算 rest_lengths_（固定不变）；否则退回到 rest_lengths_override 静态模式。
        double rope_length_m{0.0};
        std::vector<double> rest_lengths_override{};
    };

    struct Input
    {
        std::array<Vector3, kNumUavs> uav_positions_ned{};
        std::array<Vector3, kNumUavs> uav_velocities_ned{};
        Vector3 payload_position_ned{Vector3::Zero()};
        Vector3 payload_velocity_ned{Vector3::Zero()};
        Vector3 payload_target_ned{Vector3::Zero()};
        Vector3 prev_thrust_ned{Vector3::Zero()};
        int self_index{-1};
        double mass{1.0};
        bool prev_thrust_valid{false};
        /// 当前调用的时间戳（秒，绝对值不重要，仅用于计算两次调用间的真实 dt）。
        /// 不填（保持 0）时退回到 kFixedComputeDtS。
        double timestamp_s{0.0};
    };

    struct DebugState
    {
        std::array<Vector3, kNumUavs> uav_positions_ned{};
        std::array<Vector3, kNumUavs> uav_velocities_ned{};
        Vector3 payload_position_ned{Vector3::Zero()};
        Vector3 payload_velocity_ned{Vector3::Zero()};
        Vector3 payload_target_ned{Vector3::Zero()};
        Vector3 desired_payload_velocity{Vector3::Zero()};
        std::array<Vector3, kNumNodes> virtual_positions_ned{};
        std::array<Vector3, kNumNodes> virtual_velocities_ned{};
        std::array<double, kNumUavs> beta{};
        std::array<double, kNumNodes * kNumNodes> rest_lengths{};
        Vector3 prev_thrust_ned{Vector3::Zero()};
        Vector3 spring_force{Vector3::Zero()};
        Vector3 damping_force{Vector3::Zero()};
        Vector3 friction_force{Vector3::Zero()};
        Vector3 passive_force{Vector3::Zero()};
        Vector3 tracking_p_term{Vector3::Zero()};
        Vector3 tracking_i_term{Vector3::Zero()};
        Vector3 tracking_d_term{Vector3::Zero()};
        Vector3 tracking_input{Vector3::Zero()};
        Vector3 virtual_acceleration{Vector3::Zero()};
        Vector3 mapped_acceleration{Vector3::Zero()};
        Vector3 azimuth_stabilization_acceleration{Vector3::Zero()};
        Vector3 separation_acceleration{Vector3::Zero()};
        Vector3 base_acceleration{Vector3::Zero()};
        Vector3 rope_direction_ned{Vector3::Zero()};
        Vector3 estimated_rope_force_ned{Vector3::Zero()};
        Vector3 rope_compensation_acceleration{Vector3::Zero()};
        Vector3 unclipped_desired_acceleration{Vector3::Zero()};
        Vector3 desired_acceleration{Vector3::Zero()};
        double dt_used{0.0};
        double measured_rope_axis_velocity{0.0};
        double observer_input_rope_axis_acceleration{0.0};
        double rope_axis_velocity_hat{0.0};
        double rope_axis_disturbance_hat{0.0};
        double rope_axis_observer_error{0.0};
        double estimated_rope_tension_n{0.0};
        int self_index{-1};
        double mass{0.0};
        bool prev_thrust_valid{false};
        bool structure_locked{false};
        bool rope_observer_valid{false};
        bool valid{false};
    };

    struct Output
    {
        Vector3 desired_acceleration{Vector3::Zero()};
        bool valid{false};
        DebugState debug{};
    };

    // initialize: 加载配置并重新初始化所有运行态，返回 false 表示配置非法。
    bool initialize(const Config& cfg);
    // reset: 清空所有跨周期运行态（积分、微分、CFO 观测器），保留配置和结构参考长度。
    void reset();
    // resetPayloadIntegral: 仅清空 payload 位置积分，保留 CFO 观测器已收敛的状态。
    // 用于 FORM_HOLD → PAYLOAD_TRACK 切换，避免携带旧积分的同时保留绳张力估计。
    void resetPayloadIntegral();
    // resetAzimuthReference: 作废当前 azimuth 参考角；下次 compute() 会按彼时几何重新捕获。
    // 用于 FORM_HOLD → PAYLOAD_TRACK 切换，避免用起飞期捕获的过期参考角产生持续切向力。
    void resetAzimuthReference();
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

    struct PassiveForceBreakdown
    {
        Vector3 spring_force{Vector3::Zero()};
        Vector3 damping_force{Vector3::Zero()};
        Vector3 friction_force{Vector3::Zero()};
        Vector3 total_force{Vector3::Zero()};
    };

    struct RopeObserverEstimate
    {
        Vector3 rope_direction_ned{Vector3::Zero()};
        Vector3 estimated_rope_force_ned{Vector3::Zero()};
        Vector3 rope_compensation_acceleration{Vector3::Zero()};
        double measured_rope_axis_velocity{0.0};
        double observer_input_rope_axis_acceleration{0.0};
        double rope_axis_velocity_hat{0.0};
        double rope_axis_disturbance_hat{0.0};
        double rope_axis_observer_error{0.0};
        double estimated_rope_tension_n{0.0};
        bool valid{false};
    };

    struct TrackingInputBreakdown
    {
        Vector3 proportional_term{Vector3::Zero()};
        Vector3 integral_term{Vector3::Zero()};
        Vector3 derivative_term{Vector3::Zero()};
        Vector3 total_force{Vector3::Zero()};
    };

    static bool finiteVector(const Vector3& v);
    static Vector3 clipNorm(const Vector3& v, double max_norm);
    void resetRuntimeState();
    bool loadRestLengths();
    bool buildRestLengthsFromGeometry();
    bool validateInput(const Input& input) const;
    bool buildVirtualState(const Input& input, double h_u, VirtualState& state) const;
    RopeObserverEstimate estimateRopeTensionCfo(const Input& input, double dt,
                                                const Vector3& base_acceleration);
    Vector3 computeDesiredPayloadVelocity(const Input& input, double dt);
    PassiveForceBreakdown computePassiveNetworkForce(
        int self_index,
        const VirtualState& state) const;
    void initializeAzimuthReference(const VirtualState& state);
    Vector3 computeAzimuthStabilizationAcceleration(
        int self_index,
        const VirtualState& state) const;
    TrackingInputBreakdown computeTrackingInput(
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
    std::array<double, kNumUavs> rope_axis_velocity_hat_{};
    std::array<double, kNumUavs> rope_axis_disturbance_hat_{};
    std::array<double, kNumUavs> azimuth_reference_rad_{};
    bool azimuth_reference_valid_{false};
    bool prev_velocity_error_valid_{false};
    double prev_timestamp_s_{0.0};
    bool prev_timestamp_valid_{false};
};

}  // namespace control
}  // namespace swarm_planner

#endif  // SWARM_PLANNER_PLANNER_CORE_H_
