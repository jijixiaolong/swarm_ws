#ifndef FSMPX4_H_
#define FSMPX4_H_

#include <array>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fsmpx4/msg/fsm_debug.hpp>
#include <memory>
#include <string>

#include "control.h"
#include "fsm_contexts.h"
#include "input.h"
#include "param_loader.h"
#include "swarm_planner/msg/swarm_planner_debug.hpp"
#include "swarm_planner/planner_core.h"
#include "swarm_state.h"
#include "types.h"

namespace fsmpx4
{

class FSMPX4 : public rclcpp::Node
{
public:
    explicit FSMPX4(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    enum class State
    {
        MANUAL_CTRL,
        OFFBOARD_STABILIZED,
        AUTO_TAKEOFF,
        AUTO_HOVER,
        AUTO_LAND,
        CMD_CTRL
    };

    void process();
    State currentState() const { return state_; }

private:
    // ── Initialization ──
    void initializePublishers();
    void initializeSubscribers();
    void ControlLoop(double frequency_hz);

    // ── FSM core: Guard / Action separation ──
    void checkTransitions(const rclcpp::Time& now);
    void executeState(const rclcpp::Time& now);
    void enterState(State next_state);

    // ── Readiness queries ──
    bool rcReady(const rclcpp::Time& now);
    bool imuReady(const rclcpp::Time& now);
    bool positionReady(const rclcpp::Time& now);
    bool buildSwarmInput(
        const rclcpp::Time& now,
        swarm_planner::control::SwarmPlannerCore::Input& input) const;
    bool landDetectedReady(const rclcpp::Time& now) const;
    void fallbackToManual(const char* reason);
    /// 检查编队质量是否满足切换到 PAYLOAD_TRACK 的条件
    bool checkFormationQuality(
        const swarm_planner::control::SwarmPlannerCore::DebugState& dbg,
        const rclcpp::Time& now);

    // ── PX4 communication ──
    void publishOffboardMode(bool use_attitude);
    void publishAttitudeCommand(const types::Quaternion& attitude, double thrust, const rclcpp::Time& stamp);
    void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
    bool toggleOffboardMode(bool on_off);
    void publishDebugMessage(const rclcpp::Time& stamp);
    void publishSwarmDebugMessage(
        const rclcpp::Time& stamp,
        const swarm_planner::control::SwarmPlannerCore::DebugState& debug);

    // ── Utility ──
    void logLoadedParams() const;

    // ── Parameters ──
    param_loader::FSMParams params_;
    std::string px4_ns_prefix_;

    // ── FSM state + per-state contexts ──
    State state_;
    TakeoffCtx takeoff_ctx_;
    HoverCtx hover_ctx_;
    LandCtx land_ctx_;
    CmdCtx cmd_ctx_;

    // ── Sensor readiness ──
    bool land_detected_received_{false};
    bool land_detected_landed_{true};
    rclcpp::Time land_detected_stamp_{0, 0, RCL_ROS_TIME};
    bool land_request_pending_{false};

    // ── Controller & state ──
    control::PositionAttitudeController controller_;
    std::shared_ptr<types::UAVState> current_state_;
    types::ControlOutput output_;
    types::UAVCommand cmd_;
    SwarmState swarm_state_{};
    swarm_planner::control::SwarmPlannerCore swarm_core_{};

    // ── Input modules (reference-injected with *current_state_) ──
    input::RC_Receiver rc_input_{*current_state_};
    input::IMU_Reader imu_input_{*current_state_};
    input::Position_Reader position_input_{*current_state_};

    // ── PX4 publishers ──
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_pub_;
    rclcpp::Publisher<fsmpx4::msg::FSMDebug>::SharedPtr debug_pub_;
    rclcpp::Publisher<swarm_planner::msg::SwarmPlannerDebug>::SharedPtr swarm_debug_pub_;

    // ── Subscribers ──
    rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr rc_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_vel_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr gps_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr land_trigger_sub_;
    std::array<rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr,
               swarm_planner::kNumUavs - 1> uav_global_subs_{};
    std::array<rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr,
               swarm_planner::kNumUavs - 1> uav_local_subs_{};
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr load_global_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr load_local_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ── Offboard toggle helper ──
    bool offboard_toggle_active_{false};
    bool offboard_last_command_state_{false};
    bool offboard_has_last_command_{false};
    rclcpp::Time offboard_toggle_start_time_{};
    rclcpp::Duration offboard_toggle_duration_{rclcpp::Duration::from_seconds(0.5)};

    // ── Debug publish throttling ──
    // 控制回路运行在高频（如 200Hz），debug 消息以更低频率发布以减少数据量。
    swarm_planner::control::SwarmPlannerCore::DebugState latest_swarm_debug_{};
    bool swarm_debug_ready_{false};
    uint32_t debug_publish_counter_{0};
    uint32_t debug_publish_every_n_{10};  // 根据控制频率换算为约 20Hz，200Hz 时为 10
};

}  // namespace fsmpx4

#endif  // FSMPX4_H_
