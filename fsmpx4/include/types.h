#ifndef TYPES_H_
#define TYPES_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace fsmpx4 {
namespace types {

// Eigen类型别名
using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;
using Quaternion = Eigen::Quaterniond;

//=============================================================================
// 核心数据结构定义
//=============================================================================

/**
 * UAV状态结构体
 * 包含无人机的完整状态信息
 */
struct UAVState {
    Vector3 position;           // 位置 [x, y, z]
    Vector3 velocity;           // 速度 [vx, vy, vz]
    Matrix3 rotation;           // 旋转矩阵 R
    Vector3 angular_velocity;   // 角速度 [wx, wy, wz]
    double hover_thrust;        // 悬停推力值
    double timestamp;           // 时间戳

    UAVState() :
        position(Vector3::Zero()),
        velocity(Vector3::Zero()),
        rotation(Matrix3::Identity()),
        angular_velocity(Vector3::Zero()),
        hover_thrust(-0.5),
        timestamp(0.0) {}
};

/**
 * 无人机指令结构体
 * 包含期望的位置、速度、姿态等完整指令信息
 */
struct UAVCommand {
    Vector3 position;           // 期望位置
    Vector3 velocity;           // 期望速度
    Vector3 acceleration;       // 期望加速度
    Vector3 b1d;                // 期望方向向量
    double yaw_desired;         // 期望偏航角

    UAVCommand() :
        position(Vector3::Zero()),
        velocity(Vector3::Zero()),
        acceleration(Vector3::Zero()),
        b1d(Vector3::UnitX()),
        yaw_desired(0.0) {}
};

/**
 * 控制输出结构体
 * 控制器计算得到的控制输出
 */
struct ControlOutput {
    double thrust;              // 归一化推力指令（机体系 z 轴，负值表示向上）
    Vector3 thrust_vector;      // 完整推力向量 [Fx, Fy, Fz] (牛顿)
    Vector3 A;                  // 期望加速度向量
    Quaternion qd;              // 期望姿态四元数
    bool valid;                 // 输出有效性

    ControlOutput() :
        thrust(0.0),
        thrust_vector(Vector3::Zero()),
        A(Vector3::Zero()),
        qd(Quaternion::Identity()),
        valid(false) {}
};

} // namespace types
} // namespace fsmpx4

#endif  // TYPES_H_
