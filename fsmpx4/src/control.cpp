#include "control.h"
#include "param_loader.h"

#include <algorithm>
#include <cmath>

namespace fsmpx4 {
namespace control {

void PositionAttitudeController::Config::load_from_params(const param_loader::FSMParams& params)
{
    kp.x() = params.gains.Kp_x;
    kp.y() = params.gains.Kp_y;
    kp.z() = params.gains.Kp_z;
    kv.x() = params.gains.Kv_x;
    kv.y() = params.gains.Kv_y;
    kv.z() = params.gains.Kv_z;
    kvi.x() = params.gains.Kvi_x;
    kvi.y() = params.gains.Kvi_y;
    kvi.z() = params.gains.Kvi_z;
    mass = params.physical.mass;
    gravity = params.physical.gravity;
    use_integral = params.basic.use_integral;
    hover_thrust_default = params.thr_map.hover_percentage;
}

bool PositionAttitudeController::initialize(const Config& cfg)
{
    // 基本参数校验
    if (cfg.mass <= 0.0 || cfg.gravity <= 0.0) return false;
    if ((cfg.kp.array() < 0.0).any()) return false;
    if ((cfg.kv.array() < 0.0).any()) return false;
    if ((cfg.kvi.array() < 0.0).any()) return false;
    if (cfg.hover_thrust_default <= 0.0) return false;

    cfg_ = cfg;
    initialized_ = true;
    resetIntegrator();
    return true;
}


types::ControlOutput PositionAttitudeController::computeControl(const types::UAVState& state,
                                                                const types::UAVCommand& cmd) const
{
    if (!initialized_) {
        types::ControlOutput output;
        output.timestamp = state.timestamp;
        output.Rd = state.rotation;
        output.qd = types::Quaternion(state.rotation);
        output.qd.normalize();
        return output;
    }

    // eX = x - xd, eV = v - vd
    const types::Vector3 eX = state.position - cmd.position;
    const types::Vector3 eV = state.velocity - cmd.velocity;

    // 计算时间间隔
    double dt = computeDeltaTime(state.timestamp);
    
    // 更新时间戳
    last_timestamp_ = std::isfinite(state.timestamp) ? state.timestamp : 0.0;

    accumulateIntegral(eX, dt);

    // A = -Kx eX - Kv eV - m g e3 + m a_d
    const types::Vector3 A = -cfg_.kp.cwiseProduct(eX)
                             - cfg_.kv.cwiseProduct(eV)
                             - cfg_.kvi.cwiseProduct(position_integral_)
                             - cfg_.mass * cfg_.gravity * types::Vector3::UnitZ()
                             + cfg_.mass * cmd.acceleration;

    return computeFromA(state, A, cmd.yaw_desired, cmd.b1d);
}

types::ControlOutput PositionAttitudeController::computeFromDesiredAcceleration(
    const types::UAVState& state,
    const types::Vector3& desired_acceleration,
    double yaw_desired,
    const types::Vector3& b1d) const
{
    if (!initialized_) {
        types::ControlOutput output;
        output.timestamp = state.timestamp;
        output.Rd = state.rotation;
        output.qd = types::Quaternion(state.rotation);
        output.qd.normalize();
        return output;
    }

    const types::Vector3 A = -cfg_.mass * cfg_.gravity * types::Vector3::UnitZ() +
                             cfg_.mass * desired_acceleration;
    return computeFromA(state, A, yaw_desired, b1d);
}

types::ControlOutput PositionAttitudeController::computeFromA(const types::UAVState& state,
                                                              const types::Vector3& A,
                                                       double yaw_desired,
                                                              const types::Vector3& b1d) const
{
    types::ControlOutput output;
    output.timestamp = state.timestamp;
    output.A = A;
    output.thrust_vector = -A;

    const double a_norm = A.norm();
    if (a_norm < 1e-6) {
        output.Rd = state.rotation;
        output.qd = types::Quaternion(state.rotation);
        output.qd.normalize();
        output.thrust = 0.0;
        output.thrust_vector = types::Vector3::Zero();
        output.valid = false;
        return output;
    }

    // 机体系 z 轴与合力方向相反
    const types::Vector3 b3c = (-A / a_norm);

    types::Vector3 b1ref;
    if (b1d.norm() > 1e-6) {
        b1ref = b1d;
    } else {
        b1ref = types::Vector3(std::cos(yaw_desired), std::sin(yaw_desired), 0.0);
    }

    // 将 b1ref 投影到与 b3c 正交的平面，得到机体系 x 轴 b1c
    types::Vector3 b1_proj = b1ref - b3c * (b3c.dot(b1ref));
    double proj_norm = b1_proj.norm();
    if (proj_norm < 1e-6) {
        // 与 b3c 近平行时挑选任意正交向量
        const types::Vector3 arbitrary = std::abs(b3c.dot(types::Vector3::UnitX())) < 0.9
            ? types::Vector3::UnitX()
            : types::Vector3::UnitY();
        b1_proj = arbitrary - b3c * (b3c.dot(arbitrary));
        proj_norm = std::max(b1_proj.norm(), 1e-6);
    }
    const types::Vector3 b1c = b1_proj / proj_norm;
    const types::Vector3 b2c = b3c.cross(b1c);

    types::Matrix3 Rd;
    Rd.col(0) = b1c;
    Rd.col(1) = b2c;
    Rd.col(2) = b3c;

    types::Quaternion qd(Rd);
    qd.normalize();

    output.Rd = Rd;
    output.qd = qd;
    const double hover_thrust = (std::isfinite(state.hover_thrust) && state.hover_thrust > 0.0)
                                ? state.hover_thrust
                                : cfg_.hover_thrust_default;
    const double weight = cfg_.mass * cfg_.gravity;
    double normalized_thrust = 0.0;
    if (weight > 0.0)
    {
        const double thrust_ratio = a_norm / weight;
        normalized_thrust = thrust_ratio * hover_thrust;
    }
    // PX4 约定：机体系 Z 轴正向朝下，期望向上的推力为负值
    output.thrust = -normalized_thrust;
    output.valid = true;
    return output;
}


types::Quaternion PositionAttitudeController::computeDesiredRotation(const types::UAVState& state,
                                                                     const types::UAVCommand& cmd) const
{
    return computeControl(state, cmd).qd;
}


void PositionAttitudeController::resetIntegrator()
{
    position_integral_.setZero();
    last_timestamp_ = 0.0;
}

void PositionAttitudeController::accumulateIntegral(const types::Vector3& eX, double dt) const
{
    if (!cfg_.use_integral)
    {
        position_integral_.setZero();
        return;
    }

    if (dt <= 0.0)
    {
        return;
    }

    position_integral_ += eX * dt;

    const double limit = std::abs(cfg_.integral_limit);
    if (limit > 0.0)
    {
        for (int i = 0; i < 3; ++i)
        {
            position_integral_(i) = std::clamp(position_integral_(i), -limit, limit);
        }
    }
}

double PositionAttitudeController::computeDeltaTime(double current_timestamp) const
{
    double dt = 0.0;
    const double prev_timestamp = last_timestamp_;
    
    if (std::isfinite(current_timestamp) && prev_timestamp > 0.0)
    {
        dt = current_timestamp - prev_timestamp;
        // 检查时间间隔的合理性：必须是正数且不能太大（防止异常值）
        if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0)
        {
            dt = 0.0;
        }
    }
    
    return dt;
}

}  // namespace control
}  // namespace fsmpx4
