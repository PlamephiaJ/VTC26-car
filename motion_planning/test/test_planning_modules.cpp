#include "motion_planning/dynamic_obstacle_map.hpp"
#include "motion_planning/occupancy_grid.hpp"
#include "motion_planning/optimal_trajectory.hpp"
#include "motion_planning/path_tracking.hpp"
#include "motion_planning/reference_path_manager.hpp"
#include "motion_planning/rrt_star_planner.hpp"
#include "motion_planning/rrt_tree.hpp"

#include "gtest/gtest.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

nav_msgs::msg::OccupancyGrid make_free_grid()
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = 20;
    grid.info.height = 20;
    grid.info.resolution = 0.5;
    grid.info.origin.position.x = -5.0;
    grid.info.origin.position.y = -5.0;
    grid.data.resize(grid.info.width * grid.info.height, 0);
    return grid;
}

nav_msgs::msg::OccupancyGrid make_precise_grid()
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = 100;
    grid.info.height = 100;
    grid.info.resolution = 0.1;
    grid.info.origin.position.x = -2.0;
    grid.info.origin.position.y = -2.0;
    grid.data.resize(grid.info.width * grid.info.height, 0);
    return grid;
}

geometry_msgs::msg::Point point(const double x, const double y)
{
    geometry_msgs::msg::Point result;
    result.x = x;
    result.y = y;
    return result;
}

}  // namespace

TEST(RrtTree, SteersByAtMostConfiguredStep)
{
    rrt_star::Node source;
    const rrt_star::Node result =
        rrt_star::steer_towards(source, {3.0, 4.0}, 2.5);

    EXPECT_DOUBLE_EQ(1.5, result.x);
    EXPECT_DOUBLE_EQ(2.0, result.y);
}

TEST(RrtTree, NearIndicesUsesStrictRadius)
{
    rrt_star::Tree tree(4);
    tree.at(1).x = 0.5;
    tree.at(2).x = 0.6;
    tree.at(2).y = 0.8;
    tree.at(3).x = 1.001;

    const auto result = rrt_star::near_indices(tree, {}, 1.0);

    EXPECT_EQ((std::vector<std::size_t>{0, 1}), result);
}

TEST(RrtTree, ReparentMaintainsLinksAndDescendantCosts)
{
    rrt_star::Tree tree(5);
    tree.at(0).is_root = true;
    tree.at(0).children = {1, 2};
    tree.at(1).x = 1.0;
    tree.at(1).parent = 0;
    tree.at(1).cost = 1.0;
    tree.at(1).children = {3};
    tree.at(2).y = 1.0;
    tree.at(2).parent = 0;
    tree.at(2).cost = 1.0;
    tree.at(3).x = 2.0;
    tree.at(3).parent = 1;
    tree.at(3).cost = 2.0;
    tree.at(3).children = {4};
    tree.at(4).x = 3.0;
    tree.at(4).parent = 3;
    tree.at(4).cost = 3.0;

    rrt_star::reparent_node(tree, 3, 2, 2.5);

    EXPECT_TRUE(tree.at(1).children.empty());
    EXPECT_NE(
        tree.at(2).children.end(),
        std::find(tree.at(2).children.begin(), tree.at(2).children.end(), 3));
    EXPECT_EQ(2u, tree.at(3).parent);
    EXPECT_DOUBLE_EQ(2.5, tree.at(3).cost);
    EXPECT_DOUBLE_EQ(3.5, tree.at(4).cost);
}

TEST(PathTracking, ResamplingBoundsSpacingAndPreservesEndpoints)
{
    const std::vector<geometry_msgs::msg::Point> input = {
        point(0.0, 0.0), point(1.0, 0.0)};

    const auto result = path_tracking::resample_polyline(input, 0.3);

    ASSERT_EQ(5u, result.size());
    EXPECT_DOUBLE_EQ(0.0, result.front().x);
    EXPECT_DOUBLE_EQ(1.0, result.back().x);
    for (std::size_t i = 1; i < result.size(); ++i)
    {
        EXPECT_LE(result.at(i).x - result.at(i - 1).x, 0.3);
    }
}

TEST(DynamicObstacles, FiltersInvalidAndFarScanRanges)
{
    constexpr double half_pi = 1.57079632679489661923;
    sensor_msgs::msg::LaserScan scan;
    scan.angle_min = 0.0;
    scan.angle_increment = static_cast<float>(half_pi);
    scan.ranges = {
        1.0F, std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::quiet_NaN(), 5.0F};

    const auto hits = dynamic_obstacles::valid_hit_points(scan, 2.0);

    ASSERT_EQ(1u, hits.size());
    EXPECT_NEAR(1.0, hits.front().x, 1e-6);
    EXPECT_NEAR(0.0, hits.front().y, 1e-6);
}

TEST(DynamicObstacleMap, ClearingObservationRestoresStaticLayer)
{
    dynamic_obstacles::MapLayer layer;
    const auto grid = make_free_grid();
    layer.initialize(grid, 0.0);

    EXPECT_GT(layer.add_observation(point(0.0, 0.0), 0.5), 0u);
    EXPECT_GT(layer.collision_map().data.at(210), 50);

    layer.clear_observations();

    EXPECT_EQ(0, layer.collision_map().data.at(210));
}

TEST(DynamicObstacleMap, RebuildProfilesAndReplacesObservationSet)
{
    dynamic_obstacles::MapLayer layer;
    layer.initialize(make_free_grid(), 0.0);

    const auto first = layer.rebuild_observations({point(0.0, 0.0)}, 0.5);
    EXPECT_EQ(1u, first.observation_count);
    EXPECT_GT(first.changed_cell_count, 0u);
    EXPECT_GT(first.total.count(), 0);

    layer.rebuild_observations({point(2.0, 0.0)}, 0.0);
    EXPECT_EQ(0, layer.collision_map().data.at(210));
    EXPECT_GT(layer.collision_map().data.at(214), 50);
}

TEST(OptimalTrajectory, ProjectsSamplesAndSlicesByArcLength)
{
    const std::vector<geometry_msgs::msg::Point> waypoints = {
        point(0.0, 0.0), point(10.0, 0.0),
        point(10.0, 10.0), point(0.0, 10.0)};
    optimal_trajectory::Trajectory trajectory(waypoints);

    const auto projection = trajectory.project(
        point(2.0, 1.0), std::nullopt, 2.0, 8.0, 2.5);
    const auto sampled = trajectory.point_at(12.0);
    const auto sliced = trajectory.slice(9.0, 3.0);

    EXPECT_DOUBLE_EQ(40.0, trajectory.total_length());
    EXPECT_DOUBLE_EQ(2.0, projection.progress);
    EXPECT_DOUBLE_EQ(2.0, projection.point.x);
    EXPECT_DOUBLE_EQ(0.0, projection.point.y);
    EXPECT_DOUBLE_EQ(1.0, projection.distance);
    EXPECT_DOUBLE_EQ(10.0, sampled.x);
    EXPECT_DOUBLE_EQ(2.0, sampled.y);
    ASSERT_EQ(3u, sliced.size());
    EXPECT_DOUBLE_EQ(9.0, sliced.front().x);
    EXPECT_DOUBLE_EQ(10.0, sliced.at(1).x);
    EXPECT_DOUBLE_EQ(2.0, sliced.back().y);
}

TEST(OptimalTrajectory, SliceWrapsAcrossLapBoundary)
{
    const optimal_trajectory::Trajectory trajectory({
        point(0.0, 0.0), point(10.0, 0.0),
        point(10.0, 10.0), point(0.0, 10.0)});

    const auto sliced = trajectory.slice(39.0, 3.0);

    ASSERT_EQ(3u, sliced.size());
    EXPECT_DOUBLE_EQ(0.0, sliced.front().x);
    EXPECT_DOUBLE_EQ(1.0, sliced.front().y);
    EXPECT_DOUBLE_EQ(0.0, sliced.at(1).x);
    EXPECT_DOUBLE_EQ(0.0, sliced.at(1).y);
    EXPECT_DOUBLE_EQ(2.0, sliced.back().x);
    EXPECT_DOUBLE_EQ(0.0, sliced.back().y);
}

TEST(ReferencePathManager, UsesOptimalReferenceWhenForwardArcIsClear)
{
    reference_path::ManagerConfig config;
    config.global_goal_distance = 3.0;
    reference_path::Manager manager({
        point(0.0, 0.0), point(4.0, 0.0),
        point(4.0, 4.0), point(0.0, 4.0)}, config);

    const auto decision = manager.update(
        point(0.0, 0.0), make_precise_grid(), 0.0);

    EXPECT_EQ(reference_path::Mode::optimal_reference, decision.mode);
    EXPECT_FALSE(decision.mode_changed);
    EXPECT_TRUE(decision.optimal_arc_clear);
    ASSERT_FALSE(decision.local_optimal_reference.empty());
    EXPECT_DOUBLE_EQ(0.0, decision.local_optimal_reference.front().x);
    EXPECT_DOUBLE_EQ(3.0, decision.local_optimal_reference.back().x);
    EXPECT_DOUBLE_EQ(3.0, decision.global_goal.x);
}

TEST(ReferencePathManager, UsesRrtOnlyUntilSafeOptimalRejoin)
{
    reference_path::ManagerConfig config;
    config.global_goal_distance = 3.0;
    config.rejoin_distance = 0.5;
    config.progress_search_backward = 2.0;
    config.progress_search_forward = 8.0;
    config.projection_fallback_distance = 2.5;
    config.rejoin_clearance_time = 0.5;
    reference_path::Manager manager({
        point(0.0, 0.0), point(4.0, 0.0),
        point(4.0, 4.0), point(0.0, 4.0)}, config);
    auto grid = make_precise_grid();

    occupancy_grid::set_xy_coord_occupied(grid, 2.0, 0.0);
    const auto blocked = manager.update(point(0.0, 0.0), grid, 0.0);
    EXPECT_EQ(reference_path::Mode::rrt_detour, blocked.mode);
    EXPECT_TRUE(blocked.mode_changed);
    EXPECT_FALSE(blocked.optimal_arc_clear);
    EXPECT_DOUBLE_EQ(3.0, blocked.global_goal.x);
    EXPECT_DOUBLE_EQ(0.0, blocked.global_goal.y);

    std::fill(grid.data.begin(), grid.data.end(), 0);
    const auto too_far = manager.update(point(2.0, 2.0), grid, 0.1);
    EXPECT_EQ(reference_path::Mode::rrt_detour, too_far.mode);
    EXPECT_TRUE(too_far.optimal_arc_clear);
    EXPECT_FALSE(too_far.vehicle_near_optimal);

    occupancy_grid::set_xy_coord_occupied(grid, 2.0, 0.2);
    const auto unsafe_connector = manager.update(point(2.0, 0.4), grid, 0.2);
    EXPECT_EQ(reference_path::Mode::rrt_detour, unsafe_connector.mode);
    EXPECT_TRUE(unsafe_connector.optimal_arc_clear);
    EXPECT_TRUE(unsafe_connector.vehicle_near_optimal);
    EXPECT_FALSE(unsafe_connector.rejoin_connector_clear);
    EXPECT_DOUBLE_EQ(4.0, unsafe_connector.global_goal.x);
    EXPECT_DOUBLE_EQ(1.0, unsafe_connector.global_goal.y);

    std::fill(grid.data.begin(), grid.data.end(), 0);
    const auto first_clear = manager.update(point(2.0, 0.4), grid, 0.3);
    EXPECT_EQ(reference_path::Mode::rrt_detour, first_clear.mode);
    EXPECT_FALSE(first_clear.mode_changed);
    EXPECT_DOUBLE_EQ(0.0, first_clear.rejoin_clearance_elapsed);

    const auto still_waiting = manager.update(point(2.0, 0.4), grid, 0.7);
    EXPECT_EQ(reference_path::Mode::rrt_detour, still_waiting.mode);
    EXPECT_FALSE(still_waiting.mode_changed);
    EXPECT_NEAR(0.4, still_waiting.rejoin_clearance_elapsed, 1e-9);

    const auto rejoined = manager.update(point(2.0, 0.4), grid, 0.81);
    EXPECT_EQ(reference_path::Mode::optimal_reference, rejoined.mode);
    EXPECT_TRUE(rejoined.mode_changed);
    EXPECT_NEAR(0.51, rejoined.rejoin_clearance_elapsed, 1e-9);
    EXPECT_TRUE(rejoined.optimal_arc_clear);
    EXPECT_TRUE(rejoined.vehicle_near_optimal);
    EXPECT_TRUE(rejoined.rejoin_connector_clear);
}

TEST(RrtStarPlanner, FindsDirectPathInFreeMap)
{
    rrt_star::PlannerConfig config;
    config.minimum_iterations = 1;
    config.maximum_iterations = 2;
    config.sample_standard_deviation = 1.0;
    config.step_size = 1.0;
    config.near_radius = 1.5;
    config.goal_tolerance = 0.2;
    config.goal_sample_rate = 1.0;
    config.static_margin = 0.0;
    config.dynamic_margin = 0.0;
    rrt_star::Planner planner(config, 7u);
    const auto grid = make_free_grid();

    const auto result = planner.plan({0.0, 0.0}, {1.0, 0.0}, grid, grid);

    ASSERT_TRUE(result.success);
    ASSERT_EQ(2u, result.path.size());
    EXPECT_DOUBLE_EQ(0.0, result.path.front().x);
    EXPECT_DOUBLE_EQ(1.0, result.path.back().x);
    EXPECT_GT(result.profile.total.count(), 0);
    EXPECT_LE(result.profile.sampling, result.profile.total);
    EXPECT_LE(result.profile.nearest, result.profile.total);
    EXPECT_LE(result.profile.initial_collision, result.profile.total);
    EXPECT_LE(result.profile.near, result.profile.total);
    EXPECT_LE(result.profile.parent_collision, result.profile.total);
    EXPECT_LE(result.profile.rewiring, result.profile.total);
}
