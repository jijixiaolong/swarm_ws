#ifndef CONTROL_H_
#define CONTROL_H_

#include "types.h"

namespace fsmpx4 {
namespace param_loader {
struct FSMParams;
}

namespace control {

class PositionAttitudeController {
public:
    struct Config {
        types::Vector3 kp{types::Vector3::Constant(3.0)};  // 位置比例增益
        types::Vector3 kv{types::Vector3::Constant(2.0)};  // 速度比例增益
        types::Vector3 kvi{types::Vector3::Zero()};        // 位置积分增益
        double mass{1.0};                    // 质量 kg
        double gravity{9.81};                // 重力 m/s^2
        types::Vector3 b1d{types::Vector3::UnitX()};  // 航向参考向量（与 b3 正交），由指令提供
        bool use_integral{false};            // 是否启用位置积分
        double integral_limit{1.5};          // 积分限幅
        double hover_thrust_default{0.6};    // 默认悬停油门（归一化）

        void load_from_params(const param_loader::FSMParams& params);
    };

    PositionAttitudeController() = default;

    bool initialize(const Config& cfg);

    // 输入完整指令cmd（位置/速度/加速度/航向），输出期望四元数
    types::ControlOutput computeControl(const types::UAVState& state,
                                        const types::UAVCommand& cmd) const;

    // 直接使用期望加速度生成姿态和油门（仅做低层映射）
    types::ControlOutput computeFromDesiredAcceleration(const types::UAVState& state,
                                                        const types::Vector3& desired_acceleration,
                                                        double yaw_desired,
                                                        const types::Vector3& b1d) const;

    types::Quaternion computeDesiredRotation(const types::UAVState& state,
                                             const types::UAVCommand& cmd) const;

    const Config& config() const { return cfg_; }
    bool initialized() const { return initialized_; }
    void resetIntegrator();

private:
    types::ControlOutput computeFromA(const types::UAVState& state,
                                      const types::Vector3& A,
                                      double yaw_desired,
                                      const types::Vector3& b1d) const;
    void accumulateIntegral(const types::Vector3& eX, double dt) const;
    double computeDeltaTime(double current_timestamp) const;

    Config cfg_{};
    bool initialized_{false};
    mutable types::Vector3 position_integral_{types::Vector3::Zero()};
    mutable double last_timestamp_{0.0};
};

}  // namespace control
}  // namespace fsmpx4

#endif  // CONTROL_H_
