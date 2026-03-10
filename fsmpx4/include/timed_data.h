#ifndef TIMED_DATA_H_
#define TIMED_DATA_H_

#include <rclcpp/rclcpp.hpp>

namespace fsmpx4 {

/// Lightweight wrapper: value + timestamp + freshness check.
/// Replaces scattered rcv_stamp / xxx_valid / is_received() patterns.
template <typename T>
struct TimedData
{
    T value{};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};

    void update(const T& v, const rclcpp::Time& now)
    {
        value = v;
        stamp = now;
    }

    bool fresh(const rclcpp::Time& now, double timeout_s) const
    {
        return (now - stamp).seconds() <= timeout_s;
    }
};

}  // namespace fsmpx4

#endif  // TIMED_DATA_H_
