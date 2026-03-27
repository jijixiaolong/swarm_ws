#include <gtest/gtest.h>

#include "swarm_planner/geo_utils.h"

namespace {

void expectVectorNear(
    const swarm_planner::Vector3& actual,
    const swarm_planner::Vector3& expected,
    double tol = 1e-6)
{
    EXPECT_NEAR(actual.x(), expected.x(), tol);
    EXPECT_NEAR(actual.y(), expected.y(), tol);
    EXPECT_NEAR(actual.z(), expected.z(), tol);
}

}  // namespace

TEST(GeoUtilsTest, ConvertsLlaToCommonNedOrigin)
{
    swarm_planner::geo::GpsOrigin origin;
    origin.set(30.0, 120.0, 100.0);

    const auto ned = swarm_planner::geo::lla_to_ned(30.00001, 120.00002, 103.0, origin);
    EXPECT_GT(ned.x(), 0.0);
    EXPECT_GT(ned.y(), 0.0);
    EXPECT_NEAR(ned.z(), -3.0, 1e-9);
}

TEST(GeoUtilsTest, ZeroOffsetMapsToOrigin)
{
    swarm_planner::geo::GpsOrigin origin;
    origin.set(47.397742, 8.545594, 488.0);

    expectVectorNear(
        swarm_planner::geo::lla_to_ned(47.397742, 8.545594, 488.0, origin),
        swarm_planner::Vector3::Zero());
}
