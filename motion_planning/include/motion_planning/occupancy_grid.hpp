#ifndef MOTION_PLANNING__OCCUPANCY_GRID_HPP_
#define MOTION_PLANNING__OCCUPANCY_GRID_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <optional>
#include <utility>
#include <vector>

/** Coordinate conversion, occupancy queries, and inflation primitives. */
namespace occupancy_grid
{

constexpr int OCCUPIED_THRESHOLD = 50;

/** Optional exception used when a planner root must exit an inflation bubble. */
struct SegmentCheckOptions
{
    const nav_msgs::msg::OccupancyGrid* escape_reference_map = nullptr;
    double start_escape_distance = 0.0;
};

/**
 * Input: grid and integer cell coordinates.
 * Return: row-major data index, or -1 when the cell is outside/invalid.
 */
int xy_index_to_array_index(
    const nav_msgs::msg::OccupancyGrid& grid, int x_index, int y_index);

/**
 * Input: grid and continuous map-frame coordinate.
 * Return: containing cell's row-major data index, or -1 when outside/invalid.
 */
int xy_coord_to_array_index(
    const nav_msgs::msg::OccupancyGrid& grid, float x, float y);

/**
 * Input: grid and valid row-major data index.
 * Return: integer x/y cell indices. Callers must validate the input index.
 */
std::pair<int, int> array_index_to_xy_index(
    const nav_msgs::msg::OccupancyGrid& grid, int array_index);

/** Return the map-frame x coordinate of a valid data index's cell origin. */
float array_index_to_x_coord(
    const nav_msgs::msg::OccupancyGrid& grid, int array_index);

/** Return the map-frame y coordinate of a valid data index's cell origin. */
float array_index_to_y_coord(
    const nav_msgs::msg::OccupancyGrid& grid, int array_index);

/**
 * Input: grid and map-frame coordinate.
 * Return: true for occupied cells and for coordinates outside the grid. This
 * fail-closed behavior prevents the planner from leaving the known map.
 */
bool is_xy_coord_occupied(
    const nav_msgs::msg::OccupancyGrid& grid, float x, float y);

/** Mark the containing cell fully occupied; outside coordinates do nothing. */
void set_xy_coord_occupied(
    nav_msgs::msg::OccupancyGrid& grid, float x, float y);

/**
 * Inflate one valid cell by a square margin.
 *
 * Input: mutable grid, center data index, non-negative metric margin, and
 * value to write into previously free cells.
 * Return: indices whose values were changed. Invalid input returns an empty
 * list and leaves the grid unchanged.
 */
std::vector<int> inflate_cell(
    nav_msgs::msg::OccupancyGrid& grid, int array_index, float margin, int value);

/**
 * Inflate every occupied source cell in place by a non-negative metric margin.
 * Input/output: the supplied grid is permanently modified.
 */
void inflate_map(nav_msgs::msg::OccupancyGrid& grid, float margin);

/**
 * Test one map-frame segment using DDA cell traversal.
 *
 * Input: collision map, endpoints, and optional root-escape settings.
 * Return: true when any traversed cell is occupied/outside or the map is invalid.
 * When an escape reference map is supplied, occupied cells near the segment
 * start are allowed only if the same cells are free in that reference map.
 */
bool segment_is_blocked(
    const nav_msgs::msg::OccupancyGrid& collision_map,
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& end,
    const SegmentCheckOptions& options = SegmentCheckOptions{});

/**
 * Test every adjacent segment of an ordered map-frame polyline.
 * Empty input is blocked; a single point checks that point directly.
 */
bool polyline_is_blocked(
    const nav_msgs::msg::OccupancyGrid& collision_map,
    const std::vector<geometry_msgs::msg::Point>& points);

/**
 * Return arc length from the first polyline point to its first occupied cell.
 * A clear or empty polyline returns std::nullopt. Invalid maps fail closed and
 * return zero.
 */
std::optional<double> distance_to_first_collision(
    const nav_msgs::msg::OccupancyGrid& collision_map,
    const std::vector<geometry_msgs::msg::Point>& points);

}  // namespace occupancy_grid

#endif  // MOTION_PLANNING__OCCUPANCY_GRID_HPP_
