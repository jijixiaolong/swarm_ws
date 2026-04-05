#ifndef FSMPX4_UTILS_H_
#define FSMPX4_UTILS_H_

#include <cstdint>
#include <string_view>

#include <geometry_msgs/msg/vector3.hpp>

#include "fsmpx4.h"

namespace fsmpx4
{

std::string_view toString(FSMPX4::State state);
uint8_t toDebugState(FSMPX4::State state);
std::string_view toString(CmdPhase phase);
geometry_msgs::msg::Vector3 toVector3Msg(const swarm_planner::Vector3& v);

}  // namespace fsmpx4

#endif  // FSMPX4_UTILS_H_
