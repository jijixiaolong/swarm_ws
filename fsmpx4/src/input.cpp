#include "input.h"

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>

namespace fsmpx4 {
namespace input {

//==============================================================================
// RC_Receiver
//==============================================================================

RC_Receiver::RC_Receiver(types::UAVState& state)
    : state_(state),
      last_yaw_update_stamp_(rclcpp::Clock(RCL_ROS_TIME).now())
{
}

void RC_Receiver::feed(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg)
{
    stamp_.update(0, rclcpp::Clock(RCL_ROS_TIME).now());

    // Dead-zone filter
    auto dz = [](double v) -> double {
        constexpr double D = DEAD_ZONE;
        if (v > D) return (v - D) / (1.0 - D);
        if (v < -D) return (v + D) / (1.0 - D);
        return 0.0;
    };
    ch[0] = dz(msg->roll);
    ch[1] = dz(msg->pitch);
    ch[2] = dz(msg->throttle);
    ch[3] = dz(msg->yaw);

    mode = (msg->aux1 + 1.0) / 2.0;
    gear = (msg->aux2 + 1.0) / 2.0;

    // Current zone detection
    const bool offboard = mode > API_MODE_THRESHOLD_VALUE;
    const bool in_hover = offboard && gear > HOVER_THRESHOLD && gear < CMD_THRESHOLD;
    const bool in_cmd   = offboard && gear >= CMD_THRESHOLD;

    // Edge detection (suppress on first frame)
    if (initialized_)
    {
        const bool was_offboard = last_mode_ > API_MODE_THRESHOLD_VALUE;
        const bool was_hover = was_offboard && last_gear_ > HOVER_THRESHOLD && last_gear_ < CMD_THRESHOLD;
        const bool was_cmd   = was_offboard && last_gear_ >= CMD_THRESHOLD;

        enter_offboard_stabilized_mode = offboard && !was_offboard;
        enter_hover_mode               = in_hover && !was_hover;
        enter_command_mode             = in_cmd   && !was_cmd;
    }
    else
    {
        enter_offboard_stabilized_mode = false;
        enter_hover_mode               = false;
        enter_command_mode             = false;
        initialized_ = true;
    }

    is_offboard_stabilized_mode = offboard;
    is_hover_mode               = in_hover;
    is_command_mode             = in_cmd;

    last_mode_ = mode;
    last_gear_ = gear;
}

double RC_Receiver::getDesiredThrust() const
{
    // Map throttle ch[2] ∈ [-1, 1] → thrust ∈ [-0.9, 0.0] (PX4: negative = up)
    const double t = std::clamp(ch[2], -1.0, 1.0);
    return std::clamp(-0.45 * (t + 1.0), -0.9, 0.0);
}

types::Matrix3 RC_Receiver::getDesiredRotationMatrix()
{
    constexpr double MAX_ANGLE = 0.5;     // ~30°
    constexpr double MAX_YAW_RATE = 1.0;  // rad/s

    const double roll  = std::clamp( ch[0] * MAX_ANGLE, -MAX_ANGLE, MAX_ANGLE);
    const double pitch = std::clamp(-ch[1] * MAX_ANGLE, -MAX_ANGLE, MAX_ANGLE);

    const auto now = rclcpp::Clock(RCL_ROS_TIME).now();
    const double dt = std::clamp((now - last_yaw_update_stamp_).seconds(), 0.0, 0.5);
    last_yaw_update_stamp_ = now;

    // Reset yaw SP from current attitude when throttle is at minimum, or first time
    if (ch[2] < -0.9 || !yaw_sp_initialized_)
    {
        manual_yaw_sp_ = std::atan2(state_.rotation(1, 0), state_.rotation(0, 0));
        yaw_sp_initialized_ = true;
    }
    else if (dt > 0.0)
    {
        const double rate = std::clamp(ch[3], -1.0, 1.0) * MAX_YAW_RATE;
        manual_yaw_sp_ = std::atan2(std::sin(manual_yaw_sp_ + rate * dt),
                                    std::cos(manual_yaw_sp_ + rate * dt));
    }

    // Single Eigen chain: Rz(yaw) * Ry(pitch) * Rx(roll)
    return (Eigen::AngleAxisd(manual_yaw_sp_, types::Vector3::UnitZ())
          * Eigen::AngleAxisd(pitch, types::Vector3::UnitY())
          * Eigen::AngleAxisd(roll, types::Vector3::UnitX())).toRotationMatrix();
}

//==============================================================================
// IMU_Reader
//==============================================================================

IMU_Reader::IMU_Reader(types::UAVState& state)
    : state_(state)
{
}

void IMU_Reader::feedAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
    const auto now = rclcpp::Clock(RCL_ROS_TIME).now();
    stamp_.update(0, now);
    state_.rotation = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]).toRotationMatrix();
    state_.timestamp = now.seconds();
}

void IMU_Reader::feedAngularVelocity(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
{
    const auto now = rclcpp::Clock(RCL_ROS_TIME).now();
    stamp_.update(0, now);
    state_.angular_velocity << msg->xyz[0], msg->xyz[1], msg->xyz[2];
    state_.timestamp = now.seconds();
}

//==============================================================================
// Position_Reader
//==============================================================================

Position_Reader::Position_Reader(types::UAVState& state)
    : state_(state)
{
}

void Position_Reader::feedGlobalPosition(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    if (!msg->lat_lon_valid || !origin_.initialized)
        return;

    const auto now = rclcpp::Clock(RCL_ROS_TIME).now();
    // x/y 直接由当前经纬度相对固定 GPS 原点换算得到，因此所有飞机天然落在同一个公共 NED 原点下。
    const auto ned_xy = geo::lla_to_ned(msg->lat, msg->lon, origin_.alt_m, origin_);
    state_.position.x() = ned_xy.x();
    state_.position.y() = ned_xy.y();
    state_.timestamp = now.seconds();
    horizontal_position_.update(0, now);
}

void Position_Reader::feedLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    const auto now = rclcpp::Clock(RCL_ROS_TIME).now();
    bool updated = false;

    if (msg->z_valid && msg->z_global)
    {
        // PX4 本地 z 的原点是本机 EKF2 启动点，不能直接跨机比较。
        // 这里先用 ref_alt - z 还原当前 EKF2 融合后的 AMSL 高度，再减去固定 GPS 原点高度，
        // 这样得到的 z 就重新落回所有飞机共享的公共 NED 原点下。
        const double ekf_alt_msl = static_cast<double>(msg->ref_alt) - static_cast<double>(msg->z);
        state_.position.z() = origin_.alt_m - ekf_alt_msl;
        vertical_position_.update(0, now);
        updated = true;
    }

    if (msg->v_xy_valid && msg->v_z_valid)
    {
        const types::Vector3 vel(msg->vx, msg->vy, msg->vz);
        state_.velocity = vel;
        velocity_.update(vel, now);
        updated = true;
    }

    if (updated)
        state_.timestamp = now.seconds();
}

}  // namespace input
}  // namespace fsmpx4
