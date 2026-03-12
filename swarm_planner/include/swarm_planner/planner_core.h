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
    static constexpr int kNumUavs = 3;
    static constexpr int kNumNodes = kNumUavs + 2;

    struct CFOConfig
    {
        bool enable{false};
        double l1{8.0};
        double l2{20.0};
        double phi{0.2};
        double fmax_n{8.0};
        double l_min_m{0.2};
    };

    struct Config
    {
        double gravity{9.81};
        double h_u_m{1.0};
        double spring_k{1.2};
        double damping_c1{0.8};
        double friction_c2{0.3};
        double vel_pid_kp{1.0};
        double vel_pid_ki{0.0};
        double vel_pid_kd{0.2};
        double payload_kp{1.2};
        double acc_norm_limit_m_s2{6.0};
        double dt_min_s{0.001};
        double dt_max_s{0.2};
        double integral_limit{2.0};
        std::vector<double> rest_lengths_override{};
        CFOConfig cfo{};
    };

    struct Input
    {
        std::array<Vector3, kNumUavs> uav_positions_ned{};
        std::array<Vector3, kNumUavs> uav_velocities_ned{};
        Vector3 payload_position_ned{Vector3::Zero()};
        Vector3 payload_velocity_ned{Vector3::Zero()};
        Vector3 payload_target_ned{Vector3::Zero()};
        Vector3 previous_thrust_vector{Vector3::Zero()};
        int self_index{-1};
        double mass{1.0};
        double dt{0.0};
    };

    struct DebugState
    {
        std::array<Vector3, kNumUavs> uav_positions_ned{};
        std::array<Vector3, kNumUavs> uav_velocities_ned{};
        Vector3 payload_position_ned{Vector3::Zero()};
        Vector3 payload_velocity_ned{Vector3::Zero()};
        Vector3 payload_target_ned{Vector3::Zero()};
        Vector3 previous_thrust_vector{Vector3::Zero()};
        std::array<Vector3, kNumNodes> virtual_positions_ned{};
        std::array<Vector3, kNumNodes> virtual_velocities_ned{};
        std::array<double, kNumUavs> beta{};
        std::array<double, kNumNodes * kNumNodes> rest_lengths{};
        Vector3 passive_force{Vector3::Zero()};
        Vector3 tracking_input{Vector3::Zero()};
        Vector3 virtual_acceleration{Vector3::Zero()};
        Vector3 mapped_acceleration{Vector3::Zero()};
        Vector3 cfo_acceleration{Vector3::Zero()};
        Vector3 desired_acceleration{Vector3::Zero()};
        int self_index{-1};
        double mass{0.0};
        double dt_input{0.0};
        double dt_used{0.0};
        bool dt_valid_for_update{false};
        bool structure_locked{false};
        bool used_cfo{false};
        bool valid{false};
    };

    struct Output
    {
        Vector3 desired_acceleration{Vector3::Zero()};
        bool valid{false};
        bool used_cfo{false};
        DebugState debug{};
    };

    bool initialize(const Config& cfg);
    void reset();
    void resetStructureReference();
    bool compute(const Input& input, Output& output);

    bool structureLocked() const { return structure_locked_; }
    bool initialized() const { return initialized_; }
    const Config& config() const { return cfg_; }

private:
    using NodeVector = std::array<Vector3, kNumNodes>;

    struct VirtualState
    {
        NodeVector q{};
        NodeVector qdot{};
        std::array<double, kNumUavs> beta{};
    };

    struct CfoResult
    {
        Vector3 acceleration{Vector3::Zero()};
        bool used{false};
    };

    static bool finiteVector(const Vector3& v);
    static double sat(double x);
    static Vector3 clipNorm(const Vector3& v, double max_norm);
    void clearRuntimeState();
    bool applyConfiguredRestLengths();
    bool validateInput(const Input& input) const;
    VirtualState buildVirtualState(const Input& input, double h_u) const;
    Vector3 computePassiveNetworkForce(int self_index, const VirtualState& state) const;
    Vector3 computeTrackingInput(
        const Vector3& qdot_i,
        double dt,
        const Vector3& desired_payload_velocity);
    CfoResult computeCfoAcceleration(
        const Input& input,
        double mass,
        double dt);

    Config cfg_{};
    bool initialized_{false};
    bool structure_locked_{false};
    Eigen::Matrix<double, kNumNodes, kNumNodes> rest_lengths_{
        Eigen::Matrix<double, kNumNodes, kNumNodes>::Zero()};
    Vector3 velocity_integral_{Vector3::Zero()};
    Vector3 prev_velocity_error_{Vector3::Zero()};
    bool prev_velocity_error_valid_{false};
    double cfo_hat_v_parallel_{0.0};
    double cfo_hat_d_parallel_{0.0};
};

}  // namespace control
}  // namespace swarm_planner

#endif  // SWARM_PLANNER_PLANNER_CORE_H_
