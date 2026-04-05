#include "fsmpx4.h"
#include "fsmpx4_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace fsmpx4
{
namespace
{
constexpr double kFormHoldHeightOffsetM = 0.1;
}  // namespace

//==============================================================================
// Constructor
//==============================================================================


// 构造函数 初始化ros2节点，fsm手动模式  无人机状态的初始值
FSMPX4::FSMPX4(const rclcpp::NodeOptions& options)
    : rclcpp::Node("fsmpx4_fsm", options),
      state_(State::MANUAL_CTRL),
      current_state_(std::make_shared<types::UAVState>())
{
    const bool param_loaded = params_.load_from_node(*this);
    if (!param_loaded)
        RCLCPP_ERROR(get_logger(), "参数加载失败，fsmpx4 将使用默认配置");

    px4_ns_prefix_ = params_.basic.px4_ns;
    if (param_loaded) logLoadedParams();

    // GPS Origin
    position_input_.setGpsOrigin(
        params_.position.gps_origin_latitude_deg,
        params_.position.gps_origin_longitude_deg,
        params_.position.gps_origin_altitude_m);
    if (std::abs(params_.position.gps_origin_latitude_deg) < 1e-9 &&
        std::abs(params_.position.gps_origin_longitude_deg) < 1e-9 &&
        std::abs(params_.position.gps_origin_altitude_m) < 1e-6)
    {
        RCLCPP_WARN(get_logger(), "GPS原点仍为默认值(0,0,0)，请在 YAML 中填写实际参考点");
    }

    // Hover Thrust
    current_state_->hover_thrust = params_.thr_map.hover_percentage;

    // Controller
    control::PositionAttitudeController::Config ctrl_cfg;
    ctrl_cfg.load_from_params(params_);
    if (!controller_.initialize(ctrl_cfg))
        RCLCPP_WARN(get_logger(), "Controller initialization failed");
    if (!swarm_core_.initialize(params_.swarm.core))
        RCLCPP_WARN(get_logger(), "SwarmPlannerCore initialization failed");

    initializePublishers();
    initializeSubscribers();
    ControlLoop(params_.basic.ctrl_freq_max);
    RCLCPP_INFO(get_logger(), "FSM initialized at %.1f Hz", params_.basic.ctrl_freq_max);
}


//==============================================================================
// Publishers & Subscribers
//==============================================================================

void FSMPX4::initializePublishers()
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile);

    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        px4_ns_prefix_.empty() ? "/fmu/in/vehicle_command" : px4_ns_prefix_ + "/fmu/in/vehicle_command", qos);
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        px4_ns_prefix_.empty() ? "/fmu/in/offboard_control_mode" : px4_ns_prefix_ + "/fmu/in/offboard_control_mode", qos);
    attitude_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        px4_ns_prefix_.empty() ? "/fmu/in/vehicle_attitude_setpoint" : px4_ns_prefix_ + "/fmu/in/vehicle_attitude_setpoint", qos);

    debug_pub_ = create_publisher<fsmpx4::msg::FSMDebug>("~/debug",
        rclcpp::QoS(10).reliable());
    swarm_debug_pub_ = create_publisher<swarm_planner::msg::SwarmPlannerDebug>(
        "swarm_planner/debug", rclcpp::QoS(10).reliable());
}

void FSMPX4::initializeSubscribers()
{
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile);

    // RC
    const bool use_fmu = params_.basic.use_fmu_manual_topic;
    std::string manual_topic;
    if (use_fmu) manual_topic = px4_ns_prefix_.empty() ? "/fmu/out/manual_control_setpoint" : px4_ns_prefix_ + "/fmu/out/manual_control_setpoint";
    else if (!px4_ns_prefix_.empty()) manual_topic = px4_ns_prefix_ + "/rc/manual_control_setpoint";
    else manual_topic = "/rc/manual_control_setpoint";

    rc_sub_ = create_subscription<px4_msgs::msg::ManualControlSetpoint>(
        manual_topic, px4_qos,
        [this](const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg) { rc_input_.feed(msg); });

    // IMU
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        px4_ns_prefix_.empty() ? "/fmu/out/vehicle_attitude" : px4_ns_prefix_ + "/fmu/out/vehicle_attitude", px4_qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) { imu_input_.feedAttitude(msg); });

    angular_vel_sub_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
        px4_ns_prefix_.empty() ? "/fmu/out/vehicle_angular_velocity" : px4_ns_prefix_ + "/fmu/out/vehicle_angular_velocity", px4_qos,
        [this](const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) { imu_input_.feedAngularVelocity(msg); });

    // Position
    gps_pos_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        px4_ns_prefix_.empty() ? "/fmu/out/vehicle_global_position" : px4_ns_prefix_ + "/fmu/out/vehicle_global_position", px4_qos,
        [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) { position_input_.feedGlobalPosition(msg); });

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        px4_ns_prefix_.empty() ? "/fmu/out/vehicle_local_position" : px4_ns_prefix_ + "/fmu/out/vehicle_local_position", px4_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) { position_input_.feedLocalPosition(msg); });

    // Land detected
    land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
        px4_ns_prefix_.empty() ? "/fmu/out/vehicle_land_detected" : px4_ns_prefix_ + "/fmu/out/vehicle_land_detected", px4_qos,
        [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
            land_detected_received_ = true;
            land_detected_landed_ = msg->landed;
            land_detected_stamp_ = rclcpp::Clock(RCL_ROS_TIME).now();
        });

    // Land trigger
    land_trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/swarm/land_trigger", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            land_request_pending_ = msg->data;
            RCLCPP_INFO(get_logger(), "LAND trigger %s", msg->data ? "latched" : "canceled");
        });

    swarm_planner::geo::GpsOrigin swarm_origin;
    swarm_origin.set(
        params_.position.gps_origin_latitude_deg,
        params_.position.gps_origin_longitude_deg,
        params_.position.gps_origin_altitude_m);

    size_t sub_slot = 0;
    for (size_t i = 0; i < swarm_planner::kNumUavs; ++i)
    {
        if (static_cast<int>(i) == params_.swarm.self_index)
        {
            continue;
        }

        const std::string& ns = params_.swarm.uav_namespaces[i];
        uav_global_subs_[sub_slot] =
            create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
                ns + "/fmu/out/vehicle_global_position",
                px4_qos,
                [this, i, swarm_origin](
                    const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
                    if (!msg->lat_lon_valid || !msg->alt_valid)
                    {
                        return;
                    }

                    Kinematics kinematics = swarm_state_.uavs[i].value;
                    kinematics.pos =
                        swarm_planner::geo::lla_to_ned(msg->lat, msg->lon, msg->alt, swarm_origin);
                    swarm_state_.uavs[i].update(kinematics, get_clock()->now());
                });

        uav_local_subs_[sub_slot] =
            create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                ns + "/fmu/out/vehicle_local_position",
                px4_qos,
                [this, i](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                    if (!msg->v_xy_valid || !msg->v_z_valid)
                    {
                        return;
                    }

                    Kinematics kinematics = swarm_state_.uavs[i].value;
                    kinematics.vel = swarm_planner::Vector3(msg->vx, msg->vy, msg->vz);
                    swarm_state_.uavs[i].update(kinematics, get_clock()->now());
                });
        ++sub_slot;
    }

    if (params_.swarm.payload_global_position_topic.empty() ||
        params_.swarm.payload_local_position_topic.empty())
    {
        RCLCPP_ERROR(
            get_logger(),
            "载荷 PX4 话题未配置: global='%s' local='%s'",
            params_.swarm.payload_global_position_topic.c_str(),
            params_.swarm.payload_local_position_topic.c_str());
    }
    else
    {
        load_global_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            params_.swarm.payload_global_position_topic,
            px4_qos,
            [this, swarm_origin](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
                if (!msg->lat_lon_valid || !msg->alt_valid)
                {
                    return;
                }

                Kinematics kinematics = swarm_state_.load.value;
                kinematics.pos =
                    swarm_planner::geo::lla_to_ned(msg->lat, msg->lon, msg->alt, swarm_origin);
                swarm_state_.load.update(kinematics, get_clock()->now());
            });

        load_local_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            params_.swarm.payload_local_position_topic,
            px4_qos,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                if (!msg->v_xy_valid || !msg->v_z_valid)
                {
                    return;
                }

                Kinematics kinematics = swarm_state_.load.value;
                kinematics.vel = swarm_planner::Vector3(msg->vx, msg->vy, msg->vz);
                swarm_state_.load.update(kinematics, get_clock()->now());
            });
    }
}

//==============================================================================
// Control Loop
//==============================================================================

void FSMPX4::ControlLoop(double frequency_hz)
{
    debug_publish_every_n_ = std::max<uint32_t>(
        1u, static_cast<uint32_t>(std::lround(frequency_hz / 20.0)));
    control_timer_ = create_wall_timer(
        std::chrono::microseconds(static_cast<int64_t>(1'000'000.0 / frequency_hz)),
        [this]() { process(); });
}

void FSMPX4::process()
{
    const auto now = get_clock()->now();

    output_ = {};  output_.timestamp = now.seconds();
    cmd_ = {};     cmd_.timestamp = now.seconds();
    swarm_debug_ready_ = false;
    swarm_state_.uavs[params_.swarm.self_index].update(
        {current_state_->position, current_state_->velocity}, now);

    checkTransitions(now);
    executeState(now);

    // debug 消息降频发布：控制回路保持高频，但 debug bag 数据按 every_n 采样
    ++debug_publish_counter_;
    if (debug_publish_counter_ >= debug_publish_every_n_)
    {
        debug_publish_counter_ = 0;
        publishDebugMessage(now);
        if (swarm_debug_ready_)
        {
            publishSwarmDebugMessage(now, latest_swarm_debug_);
        }
    }
}

//==============================================================================
// FSM Guard: checkTransitions
//==============================================================================

void FSMPX4::checkTransitions(const rclcpp::Time& now)
{
    switch (state_)
    {
    case State::MANUAL_CTRL:
    {
        const bool req = rc_input_.enter_offboard_stabilized_mode || rc_input_.is_offboard_stabilized_mode;
        if (req && imuReady(now))
            enterState(State::OFFBOARD_STABILIZED);
        break;
    }

    case State::OFFBOARD_STABILIZED:
    {
        if (!rcReady(now) || !imuReady(now) || !rc_input_.is_offboard_stabilized_mode)
        {
            fallbackToManual("OFFBOARD_STABILIZED fallback");
            return;
        }
        if (rc_input_.is_hover_mode && positionReady(now))
        {
            if (params_.takeoff.enabled)
            {
                if (!landDetectedReady(now))
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                         "等待 vehicle_land_detected，暂不决定 AUTO_TAKEOFF/AUTO_HOVER");
                    break;
                }
                if (land_detected_landed_)
                {
                    enterState(State::AUTO_TAKEOFF);
                    return;
                }
            }
            enterState(State::AUTO_HOVER);
        }
        break;
    }

    case State::AUTO_TAKEOFF:
    {
        if (!rcReady(now) || !imuReady(now) || !positionReady(now) || !rc_input_.is_offboard_stabilized_mode)
        {
            fallbackToManual("AUTO_TAKEOFF fallback");
            return;
        }
        if (!rc_input_.is_hover_mode)
        {
            enterState(State::OFFBOARD_STABILIZED);
            return;
        }
        const double elapsed = std::max(0.0, now.seconds() - takeoff_ctx_.start_time);
        const double target_h = params_.takeoff.target_height_m;
        const double climbed = takeoff_ctx_.start_pos.z() - current_state_->position.z();
        if (climbed >= target_h || elapsed >= params_.takeoff.timeout_s)
        {
            RCLCPP_INFO(get_logger(), "AUTO_TAKEOFF done: %.2f/%.2fm %.1fs", climbed, target_h, elapsed);
            enterState(State::AUTO_HOVER);
        }
        break;
    }

    case State::AUTO_HOVER:
    {
        if (!rcReady(now) || !imuReady(now) || !positionReady(now) || !rc_input_.is_offboard_stabilized_mode)
        {
            fallbackToManual("AUTO_HOVER fallback");
            return;
        }
        if (rc_input_.is_command_mode)
        {
            swarm_planner::control::SwarmPlannerCore::Input input;
            if (buildSwarmInput(now, input))
            {
                enterState(State::CMD_CTRL);
                return;
            }
            if (rc_input_.enter_command_mode)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "CMD_CTRL blocked: swarm input unavailable");
            }
            break;
        }
        if (!rc_input_.is_hover_mode) { enterState(State::OFFBOARD_STABILIZED); return; }
        if (land_request_pending_) { land_request_pending_ = false; enterState(State::AUTO_LAND); }
        break;
    }

    case State::AUTO_LAND:
    {
        if (!rcReady(now) || !imuReady(now))
            fallbackToManual("AUTO_LAND fallback");
        break;
    }

    case State::CMD_CTRL:
    {
        if (!rcReady(now) || !imuReady(now) || !positionReady(now) || !rc_input_.is_offboard_stabilized_mode)
        {
            fallbackToManual("CMD_CTRL fallback");
            return;
        }
        if (!rc_input_.is_command_mode)
        {
            enterState(State::AUTO_HOVER);
            return;
        }
        // 数据新鲜度由 executeState 的 buildSwarmInput 统一检查，避免重复构建
        break;
    }
    }
}

//==============================================================================
// FSM Action: executeState
//==============================================================================

void FSMPX4::executeState(const rclcpp::Time& now)
{
    switch (state_)
    {
    case State::MANUAL_CTRL:
        toggleOffboardMode(false);
        publishOffboardMode(false);
        break;

    case State::OFFBOARD_STABILIZED:
        toggleOffboardMode(true);
        publishOffboardMode(true);
        output_.Rd = rc_input_.getDesiredRotationMatrix();
        output_.qd = types::Quaternion(output_.Rd);
        output_.qd.normalize();
        output_.thrust = rc_input_.getDesiredThrust();
        output_.valid = true;
        publishAttitudeCommand(output_.qd, output_.thrust, now);
        break;

    case State::AUTO_TAKEOFF:
    {
        toggleOffboardMode(true);
        publishOffboardMode(true);

        const double speed = params_.takeoff.ascent_velocity_m_s;
        const double target_h = params_.takeoff.target_height_m;
        const double idle_s = params_.takeoff.idle_duration_s;
        cmd_.yaw_desired = takeoff_ctx_.yaw;
        cmd_.b1d = types::Vector3(std::cos(cmd_.yaw_desired), std::sin(cmd_.yaw_desired), 0.0);

        if (now.seconds() - takeoff_ctx_.start_time < idle_s)
        {
            // Motor warm-up phase
            const double dt = now.seconds() - takeoff_ctx_.start_time;
            const double g = params_.physical.gravity;
            double des_a_z = g * std::exp(-5.0 * dt / idle_s);

            cmd_.position = takeoff_ctx_.start_pos;
            cmd_.velocity = types::Vector3::Zero();
            cmd_.acceleration = types::Vector3(0, 0, des_a_z);
        }
        else
        {
            // Linear climb phase
            const double elapsed = std::max(0.0, now.seconds() - takeoff_ctx_.start_time - idle_s);
            const double climb = std::min(speed * elapsed, target_h);
            cmd_.position = takeoff_ctx_.start_pos;
            cmd_.position.z() -= climb;  // NED: 向上为负
            cmd_.velocity = types::Vector3(0, 0, climb < target_h ? -speed : 0);
            cmd_.acceleration = types::Vector3::Zero();
        }

        output_ = controller_.computeControl(*current_state_, cmd_);
        if (output_.valid)
            publishAttitudeCommand(output_.qd, output_.thrust, now);
        break;
    }

    case State::AUTO_HOVER:
    {
        toggleOffboardMode(true);
        publishOffboardMode(true);

        if (!hover_ctx_.target_locked)
        {
            hover_ctx_.position = current_state_->position;
            hover_ctx_.yaw = std::atan2(current_state_->rotation(1, 0), current_state_->rotation(0, 0));
            hover_ctx_.target_locked = true;
            RCLCPP_INFO(get_logger(), "AUTO_HOVER locked: (%.2f,%.2f,%.2f) yaw=%.2f",
                        hover_ctx_.position.x(), hover_ctx_.position.y(), hover_ctx_.position.z(), hover_ctx_.yaw);
        }

        // AUTO_HOVER 始终使用 fsmpx4 自身的位置控制器，不再切换到 swarm planner 输出。
        cmd_.position = hover_ctx_.position;
        cmd_.velocity = types::Vector3::Zero();
        cmd_.acceleration = types::Vector3::Zero();
        cmd_.yaw_desired = hover_ctx_.yaw;
        cmd_.b1d = types::Vector3(std::cos(cmd_.yaw_desired), std::sin(cmd_.yaw_desired), 0.0);
        output_ = controller_.computeControl(*current_state_, cmd_);
        if (output_.valid)
            publishAttitudeCommand(output_.qd, output_.thrust, now);
        break;
    }

    case State::AUTO_LAND:
    {
        publishOffboardMode(false);
        if (land_ctx_.completed) break;

        const double land_resend = params_.land.command_resend_interval_s;
        const double disarm_resend = params_.land.disarm_command_resend_interval_s;
        const double disarm_hold = params_.land.disarm_command_hold_s;

        if (!land_ctx_.disarm_active)
        {
            const bool first = !land_ctx_.command_sent;
            const bool resend = land_ctx_.command_sent &&
                (now - land_ctx_.last_command_sent_stamp).seconds() >= land_resend;

            if (first || resend)
            {
                publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
                land_ctx_.last_command_sent_stamp = now;
                if (first)
                {
                    land_ctx_.command_sent = true;
                    land_ctx_.command_stamp = now;
                    RCLCPP_INFO(get_logger(), "AUTO_LAND: LAND sent");
                    break;
                }
            }
            if (landDetectedReady(now) && land_detected_landed_ &&
                (now - land_ctx_.command_stamp).seconds() > 0.8)
            {
                land_ctx_.disarm_active = true;
                land_ctx_.disarm_start_stamp = now;
                publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f, 0.0f);
                land_ctx_.last_disarm_sent_stamp = now;
                RCLCPP_INFO(get_logger(), "AUTO_LAND: DISARM sent");
            }
        }
        else
        {
            if ((now - land_ctx_.last_disarm_sent_stamp).seconds() >= disarm_resend)
            {
                publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f, 0.0f);
                land_ctx_.last_disarm_sent_stamp = now;
            }
            if ((now - land_ctx_.disarm_start_stamp).seconds() >= disarm_hold)
            {
                land_ctx_.completed = true;
                RCLCPP_INFO(get_logger(), "AUTO_LAND complete");
            }
        }
        break;
    }

    case State::CMD_CTRL:
    {
        toggleOffboardMode(true);
        publishOffboardMode(true);

        swarm_planner::control::SwarmPlannerCore::Input input;
        swarm_planner::control::SwarmPlannerCore::Output swarm_output;
        if (!buildSwarmInput(now, input))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "CMD_CTRL swarm input unavailable");
            enterState(State::AUTO_HOVER);
            return;
        }

        // ── 编队收敛门控 ────────────────────────────────────────────────────
        // Phase 0 (FORM_HOLD): 锁定进入 CMD_CTRL 时的 payload 位置作为目标，
        //   并在 NED z 方向上额外抬高 0.1m，避免目标跟着当前载荷持续刷新。
        // Phase 1 (PAYLOAD_TRACK): 使用真实任务目标（input 不变）
        if (cmd_ctx_.phase == CmdPhase::FORM_HOLD)
        {
            input.payload_target_ned = cmd_ctx_.form_hold_target_ned;
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "[CMD_CTRL] FORM_HOLD: locked payload target=(%.2f, %.2f, %.2f)",
                input.payload_target_ned.x(),
                input.payload_target_ned.y(),
                input.payload_target_ned.z());
        }

        if (!swarm_core_.compute(input, swarm_output))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "CMD_CTRL swarm planner compute failed, fallback to AUTO_HOVER");
            enterState(State::AUTO_HOVER);
            return;
        }

        // ── 检查真实几何是否已收敛，可升级到 PAYLOAD_TRACK ────────────────
        if (cmd_ctx_.phase == CmdPhase::FORM_HOLD &&
            checkFormationQuality(swarm_output.debug, now))
        {
            swarm_core_.reset();  // 清积分（payload_position_integral_ 归零）
            cmd_ctx_.phase = CmdPhase::PAYLOAD_TRACK;
            RCLCPP_INFO(
                get_logger(),
                "\033[32m[CMD_CTRL] Formation converged → PAYLOAD_TRACK\033[0m");
        }

        cmd_.acceleration = types::Vector3(
            swarm_output.desired_acceleration.x(),
            swarm_output.desired_acceleration.y(),
            swarm_output.desired_acceleration.z());
        cmd_.yaw_desired = std::atan2(current_state_->rotation(1, 0), current_state_->rotation(0, 0));
        cmd_.b1d = types::Vector3(std::cos(cmd_.yaw_desired), std::sin(cmd_.yaw_desired), 0.0);

        output_ = controller_.computeFromDesiredAcceleration(
            *current_state_, cmd_.acceleration, cmd_.yaw_desired, cmd_.b1d);
        if (output_.valid)
        {
            publishAttitudeCommand(output_.qd, output_.thrust, now);
        }
        latest_swarm_debug_ = swarm_output.debug;
        swarm_debug_ready_ = true;
        break;
    }
    }
}

//==============================================================================
// enterState
//==============================================================================

void FSMPX4::enterState(State next_state)
{
    RCLCPP_INFO(get_logger(), "\033[32mFSM: %.*s -> %.*s\033[0m",
                static_cast<int>(toString(state_).size()), toString(state_).data(),
                static_cast<int>(toString(next_state).size()), toString(next_state).data());

    switch (next_state)
    {
        case State::AUTO_TAKEOFF:
        {
            const auto t_now = get_clock()->now();
            takeoff_ctx_.start_pos = current_state_->position;
            takeoff_ctx_.start_time = t_now.seconds();
            takeoff_ctx_.yaw = std::atan2(current_state_->rotation(1, 0),
                                           current_state_->rotation(0, 0));
            RCLCPP_INFO(get_logger(), "AUTO_TAKEOFF: start=(%.2f,%.2f,%.2f) target=%.1fm",
                        takeoff_ctx_.start_pos.x(), takeoff_ctx_.start_pos.y(),
                        takeoff_ctx_.start_pos.z(), params_.takeoff.target_height_m);
            break;
        }
        case State::AUTO_HOVER:   hover_ctx_.reset();   break;
        case State::AUTO_LAND:    land_ctx_.reset();     break;
        case State::CMD_CTRL:
            swarm_core_.reset();
            cmd_ctx_.reset();
            cmd_ctx_.form_hold_target_ned = swarm_state_.load.value.pos;
            RCLCPP_INFO(
                get_logger(),
                "CMD_CTRL: lock FORM_HOLD target to payload entry pose with %.2fm upward offset: "
                "(%.2f, %.2f, %.2f)",
                kFormHoldHeightOffsetM,
                cmd_ctx_.form_hold_target_ned.x(),
                cmd_ctx_.form_hold_target_ned.y(),
                cmd_ctx_.form_hold_target_ned.z());
            break;
        default: break;
    }
    state_ = next_state;
}

//==============================================================================
// checkFormationQuality
//==============================================================================

bool FSMPX4::checkFormationQuality(
    const swarm_planner::control::SwarmPlannerCore::DebugState& dbg,
    const rclcpp::Time& now)
{
    using Core = swarm_planner::control::SwarmPlannerCore;
    const auto& gate = params_.swarm.formation_gate;

    if (!gate.enabled)
    {
        return true;  // 门控关闭时直接放行
    }

    // ── 指标：真实 4 节点距离矩阵（3 UAV + payload）收敛 ──────────────────
    // UAV-UAV 使用结构参考边长；UAV-payload 使用真实绳长参考。
    double max_relative_error = 0.0;
    std::array<int, Core::kNumUavs + 1> real_node_indices{};
    for (int i = 0; i < Core::kNumUavs; ++i)
    {
        real_node_indices[static_cast<size_t>(i)] = i;
    }
    constexpr int kPayloadNodeIndex = Core::kNumUavs + 1;
    real_node_indices.back() = kPayloadNodeIndex;

    const auto real_node_position =
        [&dbg](int node_idx) -> const swarm_planner::Vector3&
    {
        return (node_idx < Core::kNumUavs)
            ? dbg.uav_positions_ned[static_cast<size_t>(node_idx)]
            : dbg.payload_position_ned;
    };

    bool has_reference_edge = false;
    for (size_t a = 0; a < real_node_indices.size(); ++a)
    {
        for (size_t b = a + 1; b < real_node_indices.size(); ++b)
        {
            const int i = real_node_indices[a];
            const int j = real_node_indices[b];
            const auto& p_i = real_node_position(i);
            const auto& p_j = real_node_position(j);
            const double actual_distance = (p_i - p_j).norm();

            double reference_distance = 0.0;
            const bool payload_edge =
                (i == kPayloadNodeIndex) ^ (j == kPayloadNodeIndex);
            if (payload_edge)
            {
                reference_distance = params_.swarm.rope_length_m;
            }
            else
            {
                reference_distance =
                    dbg.rest_lengths[static_cast<size_t>(i * Core::kNumNodes + j)];
            }
            if (reference_distance <= 1e-6)
            {
                continue;
            }

            has_reference_edge = true;
            const double relative_error =
                std::abs(actual_distance - reference_distance) / reference_distance;
            max_relative_error = std::max(max_relative_error, relative_error);
        }
    }
    if (!has_reference_edge || max_relative_error > gate.struct_err_max)
    {
        cmd_ctx_.phase_ok_since = -1.0;  // 重置计时
        return false;
    }

    // ── 距离误差满足：开始计时 ────────────────────────────────────────────────
    const double now_s = now.seconds();
    if (cmd_ctx_.phase_ok_since < 0.0)
    {
        cmd_ctx_.phase_ok_since = now_s;
        RCLCPP_INFO(
            get_logger(),
            "[CMD_CTRL] Real UAV-payload distance matrix converged (max relative error %.3f), "
            "holding for %.1f s before PAYLOAD_TRACK",
            max_relative_error,
            gate.hold_duration_s);
    }

    return (now_s - cmd_ctx_.phase_ok_since) >= gate.hold_duration_s;
}

//==============================================================================
// Readiness
//==============================================================================

bool FSMPX4::rcReady(const rclcpp::Time& now) { return rc_input_.fresh(now); }
bool FSMPX4::imuReady(const rclcpp::Time& now) { return imu_input_.fresh(now); }
bool FSMPX4::positionReady(const rclcpp::Time& now)
{
    return position_input_.positionFresh(now) && position_input_.velocityFresh(now);
}

bool FSMPX4::buildSwarmInput(
    const rclcpp::Time& now,
    swarm_planner::control::SwarmPlannerCore::Input& input) const
{
    for (size_t i = 0; i < swarm_planner::kNumUavs; ++i)
    {
        if (!swarm_state_.uavs[i].fresh(now, params_.swarm.data_timeout_s))
        {
            return false;
        }

        input.uav_positions_ned[i] = swarm_state_.uavs[i].value.pos;
        input.uav_velocities_ned[i] = swarm_state_.uavs[i].value.vel;
    }

    if (!swarm_state_.load.fresh(now, params_.swarm.data_timeout_s))
    {
        return false;
    }

    input.payload_position_ned = swarm_state_.load.value.pos;
    input.payload_velocity_ned = swarm_state_.load.value.vel;
    input.payload_target_ned = params_.swarm.target_ned;
    input.self_index = params_.swarm.self_index;
    input.mass = params_.swarm.mass;
    return true;
}

bool FSMPX4::landDetectedReady(const rclcpp::Time& now) const
{
    if (!land_detected_received_) return false;
    return (now - land_detected_stamp_).seconds() <= params_.land.detected_ready_timeout_s;
}

void FSMPX4::fallbackToManual(const char* reason)
{
    if (reason)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", reason);
    toggleOffboardMode(false);
    publishOffboardMode(false);
    enterState(State::MANUAL_CTRL);
}

//==============================================================================
// PX4 communication
//==============================================================================

void FSMPX4::publishOffboardMode(bool use_attitude)
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.attitude = use_attitude;
    offboard_mode_pub_->publish(msg);
}

void FSMPX4::publishVehicleCommand(uint16_t command, float param1, float param2)
{
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = get_clock()->now().nanoseconds() / 1000;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.command = command;
    const auto clamp_id = [](int v) -> uint8_t { return static_cast<uint8_t>(std::clamp(v, 0, 255)); };
    cmd.target_system = clamp_id(params_.vehicle_command.target_system_id);
    cmd.target_component = clamp_id(params_.vehicle_command.target_component_id);
    cmd.source_system = clamp_id(params_.vehicle_command.source_system_id);
    cmd.source_component = clamp_id(params_.vehicle_command.source_component_id);
    cmd.from_external = true;
    vehicle_command_pub_->publish(cmd);
}

bool FSMPX4::toggleOffboardMode(bool on_off)
{
    auto now = get_clock()->now();
    if (!offboard_has_last_command_ || on_off != offboard_last_command_state_)
    {
        offboard_has_last_command_ = true;
        offboard_last_command_state_ = on_off;
        offboard_toggle_active_ = true;
        offboard_toggle_start_time_ = now;
    }
    if (!offboard_toggle_active_) return true;

    if ((now - offboard_toggle_start_time_) < offboard_toggle_duration_)
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                              1.0f, on_off ? 6.0f : 7.0f);
    else
        offboard_toggle_active_ = false;
    return true;
}

void FSMPX4::publishAttitudeCommand(
    const types::Quaternion& attitude, double thrust, const rclcpp::Time& stamp)
{
    types::Quaternion q = attitude.normalized();
    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    msg.timestamp = stamp.nanoseconds() / 1000;
    msg.q_d[0] = static_cast<float>(q.w());
    msg.q_d[1] = static_cast<float>(q.x());
    msg.q_d[2] = static_cast<float>(q.y());
    msg.q_d[3] = static_cast<float>(q.z());

    msg.thrust_body[2] = static_cast<float>(thrust);
    attitude_pub_->publish(msg);
}

//==============================================================================
// Debug
//==============================================================================

}  // namespace fsmpx4
