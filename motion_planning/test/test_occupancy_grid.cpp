#include "motion_planning/occupancy_grid.hpp"

#include "gtest/gtest.h"

#include <algorithm>

nav_msgs::msg::OccupancyGrid create_grid(const int width, const int height)
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.resolution = 1.0;
    grid.info.origin.position.x = -1.0;
    grid.info.origin.position.y = -2.0;
    grid.data.resize(width * height, 0);
    return grid;
}

TEST(OccupancyGrid, ConvertsCoordinatesWithinBounds)
{
    auto grid = create_grid(4, 3);

    EXPECT_EQ(0, occupancy_grid::xy_coord_to_array_index(grid, -1.0, -2.0));
    EXPECT_EQ(1, occupancy_grid::xy_coord_to_array_index(grid, 0.0, -2.0));
    EXPECT_EQ(11, occupancy_grid::xy_coord_to_array_index(grid, 2.9, 0.9));
}

TEST(OccupancyGrid, RejectsCoordinatesOutsideBounds)
{
    auto grid = create_grid(4, 3);

    EXPECT_EQ(-1, occupancy_grid::xy_coord_to_array_index(grid, -1.01, -2.0));
    EXPECT_EQ(-1, occupancy_grid::xy_coord_to_array_index(grid, 3.0, -2.0));
    EXPECT_EQ(-1, occupancy_grid::xy_coord_to_array_index(grid, -1.0, -2.01));
    EXPECT_EQ(-1, occupancy_grid::xy_coord_to_array_index(grid, -1.0, 1.0));
    EXPECT_TRUE(occupancy_grid::is_xy_coord_occupied(grid, -1.01, -2.0));
}

TEST(OccupancyGrid, InflatesSymmetrically)
{
    auto grid = create_grid(5, 5);
    const auto changes = occupancy_grid::inflate_cell(grid, 12, 1.0, 100);

    EXPECT_EQ(9u, changes.size());
    for (int y = 1; y <= 3; y++)
    {
        for (int x = 1; x <= 3; x++)
        {
            EXPECT_EQ(100, grid.data.at(y * 5 + x));
        }
    }
}

TEST(OccupancyGrid, InflatesLastRowAndColumn)
{
    auto grid = create_grid(5, 5);
    const auto changes = occupancy_grid::inflate_cell(grid, 24, 1.0, 100);

    EXPECT_EQ(4u, changes.size());
    EXPECT_EQ(100, grid.data.at(18));
    EXPECT_EQ(100, grid.data.at(19));
    EXPECT_EQ(100, grid.data.at(23));
    EXPECT_EQ(100, grid.data.at(24));
}

TEST(OccupancyGrid, SegmentCollisionSupportsPlannerRootEscape)
{
    auto static_grid = create_grid(5, 5);
    auto dynamic_grid = static_grid;
    geometry_msgs::msg::Point start;
    start.x = 1.0;
    start.y = 0.0;
    geometry_msgs::msg::Point end;
    end.x = 2.0;
    end.y = 0.0;
    occupancy_grid::set_xy_coord_occupied(dynamic_grid, start.x, start.y);

    EXPECT_TRUE(occupancy_grid::segment_is_blocked(dynamic_grid, start, end));

    occupancy_grid::SegmentCheckOptions options;
    options.escape_reference_map = &static_grid;
    options.start_escape_distance = 1.1;
    EXPECT_FALSE(occupancy_grid::segment_is_blocked(
        dynamic_grid, start, end, options));
}

TEST(OccupancyGrid, DdaVisitsEveryCrossedCell)
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = 4;
    grid.info.height = 3;
    grid.info.resolution = 1.0;
    grid.data.resize(12, 0);
    grid.data.at(2) = 100;
    geometry_msgs::msg::Point start;
    start.x = 0.1;
    start.y = 0.1;
    geometry_msgs::msg::Point end;
    end.x = 2.9;
    end.y = 1.1;

    EXPECT_TRUE(occupancy_grid::segment_is_blocked(grid, start, end));
}

TEST(OccupancyGrid, PolylineCollisionChecksEverySegment)
{
    auto grid = create_grid(5, 5);
    occupancy_grid::set_xy_coord_occupied(grid, 1.0, 0.0);
    geometry_msgs::msg::Point first;
    first.x = 0.0;
    first.y = 0.0;
    geometry_msgs::msg::Point middle;
    middle.x = 1.0;
    middle.y = 0.0;
    geometry_msgs::msg::Point last;
    last.x = 2.0;
    last.y = 0.0;

    EXPECT_TRUE(occupancy_grid::polyline_is_blocked(
        grid, {first, middle, last}));
}

TEST(OccupancyGrid, MeasuresDistanceToFirstPolylineCollision)
{
    auto grid = create_grid(7, 5);
    occupancy_grid::set_xy_coord_occupied(grid, 2.0, 0.0);
    geometry_msgs::msg::Point first;
    first.x = 0.0;
    first.y = 0.0;
    geometry_msgs::msg::Point middle;
    middle.x = 1.0;
    middle.y = 0.0;
    geometry_msgs::msg::Point last;
    last.x = 4.0;
    last.y = 0.0;

    const auto distance = occupancy_grid::distance_to_first_collision(
        grid, {first, middle, last});

    ASSERT_TRUE(distance.has_value());
    EXPECT_DOUBLE_EQ(2.0, *distance);
    std::fill(grid.data.begin(), grid.data.end(), 0);
    EXPECT_FALSE(occupancy_grid::distance_to_first_collision(
        grid, {first, middle, last}).has_value());
}
