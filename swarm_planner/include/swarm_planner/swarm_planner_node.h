#ifndef SWARM_PLANNER_NODE_H_
#define SWARM_PLANNER_NODE_H_

#include <array>
#include <string>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "swarm_planner/msg/swarm_planner_debug.hpp"
#include "swarm_planner_core.h"
#include "swarm_state_aggregator.h"
#include "types.h"

namespace swarm_planner {

class SwarmPlannerNode : public rclcpp::Node
{
public:
    explicit SwarmPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void initializePublishers(const std::string& output_topic);
    void initializeSubscriptions(const std::string& payload_navsat_topic);
    void computeAndPublish();
    void publishDebug(
        const rclcpp::Time& now,
        const control::SwarmPlannerCore::Output& output);

    control::SwarmPlannerCore core_;
    SwarmStateAggregator aggregator_;

    std::string px4_ns_;
    int self_index_{-1};
    double mass_{2.0};
    double gravity_{9.81};
    double compute_hz_{200.0};
    Vector3 target_ned_{Vector3::Zero()};

    rclcpp::Time last_compute_stamp_{0, 0, RCL_ROS_TIME};
    Vector3 last_thrust_vector_{Vector3::Zero()};

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr desired_acc_pub_;
    rclcpp::Publisher<swarm_planner::msg::SwarmPlannerDebug>::SharedPtr debug_pub_;

    std::array<rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr, kNumUavs>
        peer_global_subs_{};
    std::array<rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr, kNumUavs>
        peer_local_subs_{};
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr payload_navsat_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace swarm_planner

#endif  // SWARM_PLANNER_NODE_H_
