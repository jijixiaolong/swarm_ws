#include "fsmpx4_utils.h"

namespace fsmpx4
{

std::string_view toString(FSMPX4::State state)
{
    switch (state)
    {
        case FSMPX4::State::MANUAL_CTRL: return "MANUAL_CTRL";
        case FSMPX4::State::OFFBOARD_STABILIZED: return "OFFBOARD_STABILIZED";
        case FSMPX4::State::AUTO_TAKEOFF: return "AUTO_TAKEOFF";
        case FSMPX4::State::AUTO_HOVER: return "AUTO_HOVER";
        case FSMPX4::State::AUTO_LAND: return "AUTO_LAND";
        case FSMPX4::State::CMD_CTRL: return "CMD_CTRL";
    }
    return "UNKNOWN";
}

uint8_t toDebugState(FSMPX4::State state)
{
    switch (state)
    {
        case FSMPX4::State::MANUAL_CTRL: return fsmpx4::msg::FSMDebug::STATE_MANUAL_CTRL;
        case FSMPX4::State::OFFBOARD_STABILIZED: return fsmpx4::msg::FSMDebug::STATE_OFFBOARD_STABILIZED;
        case FSMPX4::State::AUTO_TAKEOFF: return fsmpx4::msg::FSMDebug::STATE_AUTO_TAKEOFF;
        case FSMPX4::State::AUTO_HOVER: return fsmpx4::msg::FSMDebug::STATE_AUTO_HOVER;
        case FSMPX4::State::AUTO_LAND: return fsmpx4::msg::FSMDebug::STATE_AUTO_LAND;
        case FSMPX4::State::CMD_CTRL: return fsmpx4::msg::FSMDebug::STATE_CMD_CTRL;
    }
    return fsmpx4::msg::FSMDebug::STATE_MANUAL_CTRL;
}

std::string_view toString(CmdPhase phase)
{
    switch (phase)
    {
        case CmdPhase::FORM_HOLD: return "FORM_HOLD";
        case CmdPhase::PAYLOAD_TRACK: return "PAYLOAD_TRACK";
    }
    return "UNKNOWN";
}

geometry_msgs::msg::Vector3 toVector3Msg(const swarm_planner::Vector3& v)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
}

}  // namespace fsmpx4
