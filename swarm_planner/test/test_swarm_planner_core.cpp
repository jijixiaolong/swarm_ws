#include <cmath>

#include <gtest/gtest.h>

#include "swarm_planner/planner_core.h"

namespace {

swarm_planner::control::SwarmPlannerCore::Input makeInput()
{
    swarm_planner::control::SwarmPlannerCore::Input input;
    input.uav_positions_ned[0] = swarm_planner::Vector3(-1.0, 0.0, -1.0);
    input.uav_positions_ned[1] = swarm_planner::Vector3(1.0, 0.0, -1.0);
    input.uav_positions_ned[2] = swarm_planner::Vector3(0.0, 1.0, -1.0);
    input.uav_velocities_ned[0] = swarm_planner::Vector3::Zero();
    input.uav_velocities_ned[1] = swarm_planner::Vector3::Zero();
    input.uav_velocities_ned[2] = swarm_planner::Vector3::Zero();
    input.payload_position_ned = swarm_planner::Vector3::Zero();
    input.payload_velocity_ned = swarm_planner::Vector3::Zero();
    input.payload_target_ned = swarm_planner::Vector3(0.0, 0.0, -2.0);
    input.previous_thrust_vector = swarm_planner::Vector3::Zero();
    input.self_index = 0;
    input.mass = 2.0;
    input.dt = 0.01;
    return input;
}

swarm_planner::control::SwarmPlannerCore::Config makeConfig()
{
    swarm_planner::control::SwarmPlannerCore::Config cfg;
    cfg.rest_lengths_override = {
        0.0, 0.98, 1.01, 0.55, 1.14,
        0.98, 0.0, 1.03, 0.60, 1.17,
        1.01, 1.03, 0.0, 0.59, 1.16,
        0.55, 0.60, 0.59, 0.0, 1.0,
        1.14, 1.17, 1.16, 1.0, 0.0};
    return cfg;
}

bool finiteVector(const swarm_planner::Vector3& v)
{
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

}  // namespace

TEST(SwarmPlannerCoreTest, RejectsNonPositiveVirtualHeight)
{
    swarm_planner::control::SwarmPlannerCore core;
    auto cfg = makeConfig();
    cfg.h_u_m = 0.0;
    EXPECT_FALSE(core.initialize(cfg));
}

TEST(SwarmPlannerCoreTest, RequiresConfiguredRestLengthsReference)
{
    swarm_planner::control::SwarmPlannerCore core;
    swarm_planner::control::SwarmPlannerCore::Config cfg;
    EXPECT_FALSE(core.initialize(cfg));
}

TEST(SwarmPlannerCoreTest, ProducesFiniteAccelerationAndUsesConfiguredStructure)
{
    swarm_planner::control::SwarmPlannerCore core;
    auto cfg = makeConfig();
    ASSERT_TRUE(core.initialize(cfg));

    auto input = makeInput();
    swarm_planner::control::SwarmPlannerCore::Output output;
    ASSERT_TRUE(core.compute(input, output));
    EXPECT_TRUE(output.valid);
    EXPECT_TRUE(core.structureLocked());
    EXPECT_TRUE(finiteVector(output.desired_acceleration));
}

TEST(SwarmPlannerCoreTest, ResetPreservesConfiguredStructureReference)
{
    swarm_planner::control::SwarmPlannerCore core;
    auto cfg = makeConfig();
    ASSERT_TRUE(core.initialize(cfg));

    auto input = makeInput();
    swarm_planner::control::SwarmPlannerCore::Output output;
    ASSERT_TRUE(core.compute(input, output));
    core.reset();

    EXPECT_TRUE(core.initialized());
    EXPECT_TRUE(core.structureLocked());
    ASSERT_TRUE(core.compute(input, output));
    EXPECT_TRUE(output.valid);
}

TEST(SwarmPlannerCoreTest, UsesConfiguredRestLengthsOverrideWhenProvided)
{
    swarm_planner::control::SwarmPlannerCore core;
    auto cfg = makeConfig();
    cfg.rest_lengths_override = {
        0.0, 3.0, 4.0, 5.0, 6.0,
        3.0, 0.0, 7.0, 8.0, 9.0,
        4.0, 7.0, 0.0, 10.0, 11.0,
        5.0, 8.0, 10.0, 0.0, 1.0,
        6.0, 9.0, 11.0, 1.0, 0.0};
    ASSERT_TRUE(core.initialize(cfg));
    EXPECT_TRUE(core.structureLocked());

    auto input = makeInput();
    swarm_planner::control::SwarmPlannerCore::Output output;
    ASSERT_TRUE(core.compute(input, output));
    EXPECT_DOUBLE_EQ(output.debug.rest_lengths[1], 3.0);
    EXPECT_DOUBLE_EQ(output.debug.rest_lengths[9], 9.0);
    EXPECT_DOUBLE_EQ(output.debug.rest_lengths[19], 1.0);
}

TEST(SwarmPlannerCoreTest, RejectsMalformedRestLengthsOverride)
{
    swarm_planner::control::SwarmPlannerCore core;
    auto cfg = makeConfig();
    cfg.rest_lengths_override = {0.0, 1.0, 2.0};
    EXPECT_FALSE(core.initialize(cfg));
}

TEST(SwarmPlannerCoreTest, CFOOffKeepsUsedFlagFalse)
{
    swarm_planner::control::SwarmPlannerCore core;
    auto cfg = makeConfig();
    cfg.cfo.enable = false;
    ASSERT_TRUE(core.initialize(cfg));

    auto input = makeInput();
    swarm_planner::control::SwarmPlannerCore::Output output;
    ASSERT_TRUE(core.compute(input, output));
    EXPECT_FALSE(output.used_cfo);
}

TEST(SwarmPlannerCoreTest, UsesFixedDtEvenWhenInputDtIsOutOfRange)
{
    swarm_planner::control::SwarmPlannerCore core;
    auto cfg = makeConfig();
    cfg.cfo.enable = true;
    ASSERT_TRUE(core.initialize(cfg));

    auto input = makeInput();
    input.dt = 1.0;
    swarm_planner::control::SwarmPlannerCore::Output output;
    ASSERT_TRUE(core.compute(input, output));
    EXPECT_TRUE(output.valid);
    EXPECT_TRUE(output.used_cfo);
    EXPECT_DOUBLE_EQ(output.debug.dt_input, 1.0);
    EXPECT_DOUBLE_EQ(output.debug.dt_used, 1.0 / 200.0);
    EXPECT_TRUE(output.debug.dt_valid_for_update);
}

TEST(SwarmPlannerCoreTest, MappingScalesVirtualAccelerationByBeta)
{
    swarm_planner::control::SwarmPlannerCore core;
    auto cfg = makeConfig();
    cfg.spring_k = 0.0;
    cfg.damping_c1 = 0.0;
    cfg.friction_c2 = 0.0;
    cfg.vel_pid_kp = 2.0;
    cfg.vel_pid_ki = 0.0;
    cfg.vel_pid_kd = 0.0;
    cfg.payload_kp = 1.2;
    cfg.acc_norm_limit_m_s2 = 10.0;
    ASSERT_TRUE(core.initialize(cfg));

    auto input = makeInput();
    input.payload_position_ned = swarm_planner::Vector3::Zero();
    input.payload_target_ned = swarm_planner::Vector3(1.0, 0.0, 0.0);
    input.payload_velocity_ned = swarm_planner::Vector3::Zero();
    input.uav_positions_ned[0] = swarm_planner::Vector3(0.2, 0.0, -1.5);
    input.uav_positions_ned[1] = swarm_planner::Vector3(0.4, 0.0, -1.5);
    input.uav_positions_ned[2] = swarm_planner::Vector3(-0.2, 0.3, -1.5);
    input.uav_velocities_ned[0] = swarm_planner::Vector3::Zero();
    input.uav_velocities_ned[1] = swarm_planner::Vector3::Zero();
    input.uav_velocities_ned[2] = swarm_planner::Vector3::Zero();

    swarm_planner::control::SwarmPlannerCore::Output output;
    ASSERT_TRUE(core.compute(input, output));
    ASSERT_TRUE(output.valid);

    const swarm_planner::Vector3 desired_payload_velocity =
        -cfg.payload_kp * (input.payload_position_ned - input.payload_target_ned);
    const swarm_planner::Vector3 expected =
        output.debug.beta[0] * (cfg.vel_pid_kp * desired_payload_velocity / input.mass);
    EXPECT_NEAR(output.desired_acceleration.x(), expected.x(), 1e-6);
    EXPECT_NEAR(output.desired_acceleration.y(), expected.y(), 1e-6);
    EXPECT_NEAR(output.desired_acceleration.z(), expected.z(), 1e-6);
}

TEST(SwarmPlannerCoreTest, AppliesAccelerationNormLimit)
{
    swarm_planner::control::SwarmPlannerCore core;
    auto cfg = makeConfig();
    cfg.acc_norm_limit_m_s2 = 0.5;
    ASSERT_TRUE(core.initialize(cfg));

    auto input = makeInput();
    input.payload_target_ned = swarm_planner::Vector3(100.0, 100.0, -50.0);
    swarm_planner::control::SwarmPlannerCore::Output output;
    ASSERT_TRUE(core.compute(input, output));
    EXPECT_LE(output.desired_acceleration.norm(), cfg.acc_norm_limit_m_s2 + 1e-6);
}
