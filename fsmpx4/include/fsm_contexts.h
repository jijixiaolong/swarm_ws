#ifndef FSM_CONTEXTS_H_
#define FSM_CONTEXTS_H_

#include <rclcpp/rclcpp.hpp>
#include "types.h"

namespace fsmpx4 {

/// Per-state context: AUTO_TAKEOFF
struct TakeoffCtx
{
    types::Vector3 start_pos{types::Vector3::Zero()};
    double start_time{0.0};
    double yaw{0.0};
};

/// Per-state context: AUTO_HOVER
struct HoverCtx
{
    bool target_locked{false};
    types::Vector3 position{types::Vector3::Zero()};
    double yaw{0.0};

    void reset()
    {
        target_locked = false;
    }
};

/// Per-state context: AUTO_LAND
struct LandCtx
{
    bool command_sent{false};
    bool disarm_active{false};
    bool completed{false};
    rclcpp::Time command_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_command_sent_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time disarm_start_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_disarm_sent_stamp{0, 0, RCL_ROS_TIME};

    void reset()
    {
        command_sent = false;
        disarm_active = false;
        completed = false;
        command_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
        last_command_sent_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
        disarm_start_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
        last_disarm_sent_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
};

/// Per-state context: CMD_CTRL
struct CmdCtx
{};

}  // namespace fsmpx4

#endif  // FSM_CONTEXTS_H_
