#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "swarm_planner/swarm_planner_node.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<swarm_planner::SwarmPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
