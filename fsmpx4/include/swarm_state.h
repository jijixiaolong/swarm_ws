#ifndef FSMPX4_SWARM_STATE_H_
#define FSMPX4_SWARM_STATE_H_

#include <array>

#include "swarm_planner/planner_types.h"

namespace fsmpx4 {

struct Kinematics
{
    swarm_planner::Vector3 pos{swarm_planner::Vector3::Zero()};
    swarm_planner::Vector3 vel{swarm_planner::Vector3::Zero()};
};

struct SwarmState
{
    std::array<swarm_planner::TimedData<Kinematics>, swarm_planner::kNumUavs> uavs{};
    swarm_planner::TimedData<Kinematics> load{};
};

}  // namespace fsmpx4

#endif  // FSMPX4_SWARM_STATE_H_
