#ifndef INPUT_H_
#define INPUT_H_

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include "types.h"
#include "timed_data.h"
#include "geo_utils.h"

namespace fsmpx4 {
namespace input {

//=============================================================================
// RC_Receiver — 遥控器输入（极简 edge-detect 状态机）
//=============================================================================
class RC_Receiver
{
public:
    explicit RC_Receiver(types::UAVState& state);

    void feed(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg);
    bool fresh(const rclcpp::Time& now, double timeout_s = 0.5) const { return stamp_.fresh(now, timeout_s); }

    double getDesiredThrust() const;
    types::Matrix3 getDesiredRotationMatrix();
    double getManualYawSetpoint() const { return manual_yaw_sp_; }

    // 通道值 (dead-zone 已滤除)
    double ch[4]{};

    // 模式/档位原始值
    double mode{0.0};
    double gear{0.0};

    // 当前状态 & 边沿标志
    bool is_offboard_stabilized_mode{false};
    bool enter_offboard_stabilized_mode{false};
    bool is_hover_mode{false};
    bool enter_hover_mode{false};
    bool is_command_mode{false};
    bool enter_command_mode{false};

    // 阈值
    static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;
    static constexpr double HOVER_THRESHOLD = 0.45;
    static constexpr double CMD_THRESHOLD = 0.75;
    static constexpr double DEAD_ZONE = 0.1;

private:
    types::UAVState& state_;
    TimedData<int> stamp_;        // value unused, only stamp matters
    double last_mode_{0.0};
    double last_gear_{0.0};
    bool initialized_{false};

    // 偏航角积分
    double manual_yaw_sp_{0.0};
    bool yaw_sp_initialized_{false};
    rclcpp::Time last_yaw_update_stamp_;
};

//=============================================================================
// IMU_Reader — 姿态 & 角速度写入
//=============================================================================
class IMU_Reader
{
public:
    explicit IMU_Reader(types::UAVState& state);

    void feedAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void feedAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg);
    bool fresh(const rclcpp::Time& now, double timeout_s = 0.2) const { return stamp_.fresh(now, timeout_s); }

private:
    types::UAVState& state_;
    TimedData<int> stamp_;
};

//=============================================================================
// Position_Reader — 位置 & 速度写入
//=============================================================================
class Position_Reader
{
public:
    explicit Position_Reader(types::UAVState& state);

    void feedGlobalPosition(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void feedLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

    void setGpsOrigin(double lat_deg, double lon_deg, double alt_m) { origin_.set(lat_deg, lon_deg, alt_m); }

    bool positionFresh(const rclcpp::Time& now, double timeout_s = 0.5) const
    {
        return horizontal_position_.fresh(now, timeout_s) && vertical_position_.fresh(now, timeout_s);
    }
    bool velocityFresh(const rclcpp::Time& now, double timeout_s = 0.5) const { return velocity_.fresh(now, timeout_s); }

private:
    types::UAVState& state_;
    geo::GpsOrigin origin_;
    TimedData<int> horizontal_position_;
    TimedData<int> vertical_position_;
    TimedData<types::Vector3> velocity_;
};

}  // namespace input
}  // namespace fsmpx4

#endif  // INPUT_H_
