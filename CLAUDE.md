# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is a ROS2 workspace containing two interdependent C++17 packages for autonomous multi-UAV swarm control using PX4 flight controllers:

- **`fsmpx4/`** — Per-UAV FSM flight controller node. Manages a 6-state machine, reads PX4 sensor topics, runs a cascaded position→attitude controller, and integrates with the swarm planner.
- **`swarm_planner/`** — Formation control and payload tracking library. Implements a spring-damper virtual network over 3 UAVs + a payload, computes per-UAV acceleration commands, and estimates rope tension via a cascaded fast observer (CFO).

The mathematical foundation is documented in [fsmpx4/docs/swarm_controller_spec.md](fsmpx4/docs/swarm_controller_spec.md).

---

## Build Commands

```bash
# Full workspace build (run from ~/swarm_ws)
cd ~/swarm_ws && colcon build

# Build specific packages only
colcon build --packages-select fsmpx4 swarm_planner

# Run all tests
colcon test --packages-select fsmpx4 swarm_planner

# Run tests with live output
colcon test --packages-select fsmpx4 --event-handlers console_direct+

# Source the workspace after building
source ~/swarm_ws/install/setup.bash
```

---

## Launching

```bash
# 3-UAV swarm (with rosbag recording and RViz)
ros2 launch fsmpx4 fsmpx4_swarm_3.launch.py \
  record_bag:=true \
  bag_root:=~/swarm_ws/src/data \
  enable_rviz_enu:=true

# Single UAV FSM node
ros2 launch fsmpx4 fsmpx4_fsm.launch.py
```

Key launch arguments for `fsmpx4_swarm_3.launch.py`: `record_bag`, `bag_root`, `bag_name` (auto-timestamped), `record_all_topics`, `enable_rviz_enu`, `rviz_fixed_frame`, `rviz_publish_rate_hz`.

---

## Architecture

### FSM States (`fsmpx4`)

```
MANUAL_CTRL → OFFBOARD_STABILIZED → AUTO_TAKEOFF → AUTO_HOVER → CMD_CTRL → AUTO_LAND
```

- **`CMD_CTRL`** has two internal phases (see `fsm_contexts.h`):
  1. **`FORM_HOLD`** — holds formation until structural error < 30% for 2 s
  2. **`PAYLOAD_TRACK`** — pursues target payload position using swarm planner output

State machine logic lives in `fsmpx4.cpp`: `checkTransitions()`, `executeState()`, `enterState()`. Per-state data is stored in context structs (`TakeoffCtx`, `HoverCtx`, `LandCtx`, `CmdCtx`) in `fsm_contexts.h`.

### Control Pipeline (`control.cpp`)

`PositionAttitudeController::computeControl()` → cascaded:
1. Position error → desired acceleration (PD + optional integral)
2. Desired acceleration → attitude quaternion + collective thrust
3. Output published as `VehicleAttitudeSetpoint` to PX4

Integral reset on state entry: call `resetIntegrator()` when switching into hover or CMD_CTRL.

### Swarm Planner (`swarm_planner_core.cpp`)

`SwarmPlannerCore::compute(Input, Output)` pipeline:
1. Map real UAV positions **p_i** to virtual nodes **q_i** using altitude scaling
2. Compute spring + damping + friction passive forces over the virtual graph (5 nodes: 3 UAVs, virtual platform U, payload L)
3. Add payload tracking PID (error in payload position vs. target)
4. Add rope tension compensation from CFO observer
5. Map virtual accelerations back to per-UAV attitude commands

The swarm planner is a **library** (`swarm_planner_core_lib`), not a ROS node. `fsmpx4` calls it directly via `buildSwarmInput()` / `compute()`.

### Input Module (`input.h`, `input.cpp`)

- `RC_Receiver` — decodes RC mode channel with edge detection and deadzone (0.1)
- `IMU_Reader` — fuses attitude quaternion + angular velocity
- `Position_Reader` — converts GPS (LLA) → NED using a configurable origin; integrates velocity for position estimates

### Parameter Loading (`param_loader.h`, `param_loader.cpp`)

All parameters load from `fsmpx4/config/fsm.yaml` via ROS2 node parameters. The `Params` struct has nested sub-structs: `Gains`, `Physical`, `Limits`, `ThrustMapping`, `Takeoff`, `Land`, `Swarm`, etc. (~100+ tunable values). Key swarm params live under `params_.swarm.*`.

---

## Key Configuration (`fsmpx4/config/fsm.yaml`)

- Control loop: `ctrl_freq_max: 200.0` Hz
- GPS origin: Zurich area (47.397742° N, 8.545594° E, 488 m MSL) — change for new sites
- Formation triangle: `uav_uav_distance_m: 1.2` (equilateral), `h_u_m: 0.5` (platform height)
- Spring-damper: `spring_k: 3.4`, `damping_c1: 0.6`, `friction_c2: 2.0`
- Payload PID: `payload_kp: 2.0`, `payload_ki: 0.1`, `payload_kd: 0.0`
- CFO observer: `observer_l1: 10`, `observer_l2: 10`, `observer_phi: 0.4`
- Formation gate: `max_structural_error: 0.30`, `hold_duration_s: 2.0`

---

## Message Types

- **`fsmpx4/FSMDebug`** — 30 fields: FSM state, UAV pose, desired trajectory, control output. Published at 50 Hz (throttled from 200 Hz loop).
- **`swarm_planner/SwarmPlannerDebug`** — 51 fields: UAV/payload states, force breakdown (spring/damping/friction/tracking P/I/D), rope observer estimates, structure lock status.

---

## Test Structure

Tests use GoogleTest via `ament_cmake_gtest`. Each package has a `test/` directory:

- `fsmpx4/test/test_fsmpx4_planner_cmd.cpp` — integration test: 3 UAVs + payload, validates FSM transitions and swarm planner command generation
- `swarm_planner/test/test_swarm_planner_core.cpp` — unit tests for force computation and virtual state updates
- `swarm_planner/test/test_geo_utils.cpp` — LLA→NED conversion accuracy

---

## Flight Data & Analysis

Recorded rosbag2 runs are stored in `data/` (timestamped directories). Helper scripts:

```bash
# Analyze the most recent bag
./analyze_latest_swarm_bag.sh

# Generate plots from a specific bag
python3 generate_plots.py <bag_path>
```

The `fsmpx4/script/` directory contains: `swarm_enu_rviz.py` (NED→ENU for RViz), `auto_arm.py`, `rc.py`, `repair_payload_static_bias.py`.
