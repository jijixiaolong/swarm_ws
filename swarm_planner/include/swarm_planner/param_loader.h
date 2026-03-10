#ifndef SWARM_PLANNER_PARAM_LOADER_H_
#define SWARM_PLANNER_PARAM_LOADER_H_

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "swarm_planner_core.h"
#include "swarm_state_aggregator.h"
#include "types.h"

namespace swarm_planner {
namespace param_loader {

struct PlannerNodeParams
{
    std::string px4_ns{};
    int self_index{-1};
    double mass{2.0};
    double gravity{9.81};
    double compute_hz{200.0};
    Vector3 target_ned{Vector3(0.0, 0.0, -2.0)};
    std::string output_topic{};
    std::string payload_navsat_topic{"/payload/navsat"};
    control::SwarmPlannerCore::Config core{};
    SwarmStateAggregator::Config aggregator{};
};

PlannerNodeParams load_from_node(rclcpp::Node& node);

}  // namespace param_loader
}  // namespace swarm_planner

#endif  // SWARM_PLANNER_PARAM_LOADER_H_
