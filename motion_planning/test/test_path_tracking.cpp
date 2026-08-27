#include "motion_planning/path_tracking.hpp"

#include "gtest/gtest.h"

#include <stdexcept>

namespace
{

void add_point(nav_msgs::msg::Path& path, const double x, const double y)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    path.poses.emplace_back(pose);
}

}  // namespace

TEST(PathTracking, ReturnsNoPointForEmptyPath)
{
    const nav_msgs::msg::Path path;

    EXPECT_FALSE(path_tracking::point_at_distance(path, 1.0).has_value());
}

TEST(PathTracking, InterpolatesAtArcLengthAlongACurve)
{
    nav_msgs::msg::Path path;
    add_point(path, 0.0, 0.0);
    add_point(path, 1.0, 0.0);
    add_point(path, 1.0, 1.0);

    const auto target = path_tracking::point_at_distance(path, 1.5);

    ASSERT_TRUE(target.has_value());
    EXPECT_DOUBLE_EQ(1.0, target->x);
    EXPECT_DOUBLE_EQ(0.5, target->y);
}

TEST(PathTracking, InterpolatesBetweenSparseWaypoints)
{
    nav_msgs::msg::Path path;
    add_point(path, 0.0, 0.0);
    add_point(path, 3.0, 4.0);

    const auto target = path_tracking::point_at_distance(path, 2.5);

    ASSERT_TRUE(target.has_value());
    EXPECT_DOUBLE_EQ(1.5, target->x);
    EXPECT_DOUBLE_EQ(2.0, target->y);
}

TEST(PathTracking, IgnoresZeroLengthSegments)
{
    nav_msgs::msg::Path path;
    add_point(path, 0.0, 0.0);
    add_point(path, 0.0, 0.0);
    add_point(path, 2.0, 0.0);

    const auto target = path_tracking::point_at_distance(path, 1.0);

    ASSERT_TRUE(target.has_value());
    EXPECT_DOUBLE_EQ(1.0, target->x);
    EXPECT_DOUBLE_EQ(0.0, target->y);
}

TEST(PathTracking, UsesLastPointWhenPathIsShorterThanLookahead)
{
    nav_msgs::msg::Path path;
    add_point(path, 0.0, 0.0);
    add_point(path, 1.0, 0.0);

    const auto target = path_tracking::point_at_distance(path, 2.0);

    ASSERT_TRUE(target.has_value());
    EXPECT_DOUBLE_EQ(1.0, target->x);
    EXPECT_DOUBLE_EQ(0.0, target->y);
}

TEST(PathTracking, ComputesAndClampsPurePursuitSteering)
{
    EXPECT_DOUBLE_EQ(
        0.25, path_tracking::pure_pursuit_steering(0.5, 1.0, 0.25, 0.41));
    EXPECT_DOUBLE_EQ(
        0.41, path_tracking::pure_pursuit_steering(2.0, 1.0, 0.25, 0.41));
    EXPECT_DOUBLE_EQ(
        -0.41, path_tracking::pure_pursuit_steering(-2.0, 1.0, 0.25, 0.41));
}

TEST(PathTracking, SelectsSpeedFromSteeringMagnitude)
{
    constexpr double pi = 3.14159265358979323846;
    path_tracking::SpeedProfile profile;
    profile.straight_speed = 2.0;
    profile.medium_turn_speed = 1.0;
    profile.sharp_turn_speed = 0.5;
    EXPECT_DOUBLE_EQ(
        2.0, path_tracking::speed_for_steering(5.0 * pi / 180.0, profile));
    EXPECT_DOUBLE_EQ(
        1.0, path_tracking::speed_for_steering(15.0 * pi / 180.0, profile));
    EXPECT_DOUBLE_EQ(
        0.5, path_tracking::speed_for_steering(25.0 * pi / 180.0, profile));
}

TEST(PathTracking, CapsSpeedFromBlockedPathDistance)
{
    EXPECT_DOUBLE_EQ(
        0.0, path_tracking::blocked_path_speed_limit(0.4, 0.5, 2.0));
    EXPECT_DOUBLE_EQ(
        0.0, path_tracking::blocked_path_speed_limit(0.5, 0.5, 2.0));
    EXPECT_DOUBLE_EQ(
        2.0, path_tracking::blocked_path_speed_limit(1.5, 0.5, 2.0));
    EXPECT_THROW(
        path_tracking::blocked_path_speed_limit(1.0, 0.5, 0.0),
        std::invalid_argument);
}
