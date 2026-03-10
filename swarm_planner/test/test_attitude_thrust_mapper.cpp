#include <cmath>

#include <gtest/gtest.h>

#include "swarm_planner/attitude_thrust_mapper.h"

namespace {

constexpr double kEps = 1e-6;
constexpr double kPi = 3.14159265358979323846;

}  // namespace

TEST(AttitudeThrustMapperTest, HoverAccelerationProducesHoverThrust)
{
    const auto result = swarm_planner::mapAccelerationToAttitudeThrust(
        Eigen::Vector3d::Zero(), 0.0, 2.0, 9.81, 0.71);

    ASSERT_TRUE(result.valid);
    EXPECT_NEAR(result.normalized_thrust, -0.71, kEps);
    EXPECT_NEAR(result.quaternion.w(), 1.0, kEps);
    EXPECT_NEAR(result.quaternion.x(), 0.0, kEps);
    EXPECT_NEAR(result.quaternion.y(), 0.0, kEps);
    EXPECT_NEAR(result.quaternion.z(), 0.0, kEps);
}

TEST(AttitudeThrustMapperTest, ZeroCollectiveAccelerationReturnsFallback)
{
    Eigen::Quaterniond fallback(Eigen::AngleAxisd(kPi * 0.5, Eigen::Vector3d::UnitZ()));
    const auto result = swarm_planner::mapAccelerationToAttitudeThrust(
        Eigen::Vector3d(0.0, 0.0, 9.81), 0.0, 2.0, 9.81, 0.71, fallback);

    EXPECT_FALSE(result.valid);
    EXPECT_NEAR(result.normalized_thrust, 0.0, kEps);
    EXPECT_NEAR(result.quaternion.w(), fallback.w(), kEps);
    EXPECT_NEAR(result.quaternion.x(), fallback.x(), kEps);
    EXPECT_NEAR(result.quaternion.y(), fallback.y(), kEps);
    EXPECT_NEAR(result.quaternion.z(), fallback.z(), kEps);
}

TEST(AttitudeThrustMapperTest, HoverYawTracksRequestedYaw)
{
    const auto result = swarm_planner::mapAccelerationToAttitudeThrust(
        Eigen::Vector3d::Zero(), kPi * 0.5, 1.0, 9.81, 0.5);

    ASSERT_TRUE(result.valid);
    EXPECT_NEAR(result.quaternion.w(), std::cos(kPi * 0.25), 1e-6);
    EXPECT_NEAR(result.quaternion.z(), std::sin(kPi * 0.25), 1e-6);
}

TEST(AttitudeThrustMapperTest, ForwardAccelerationTiltsAttitude)
{
    const auto result = swarm_planner::mapAccelerationToAttitudeThrust(
        Eigen::Vector3d(1.0, 0.0, 0.0), 0.0, 2.0, 9.81, 0.6);

    ASSERT_TRUE(result.valid);
    EXPECT_LT(std::abs(result.quaternion.y()), 1.0);
    EXPECT_GT(std::abs(result.quaternion.y()), 1e-3);
    EXPECT_LT(result.normalized_thrust, 0.0);
}
