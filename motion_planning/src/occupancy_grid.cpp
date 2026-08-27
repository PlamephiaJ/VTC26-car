/*
 * @Author: Yuhao Chen
 * @Date: 2024-06-23 11:22:01
 * @LastEditors: Yuhao Chen
 * @LastEditTime: 2024-06-25 11:22:09
 * @Description: Occupancy grid releated functions source file.
 */

#include "../include/motion_planning/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

bool cell_is_occupied(
    const nav_msgs::msg::OccupancyGrid& grid, const int x, const int y)
{
    const int index = occupancy_grid::xy_index_to_array_index(grid, x, y);
    return index < 0 ||
        static_cast<int>(grid.data[static_cast<std::size_t>(index)]) >
            occupancy_grid::OCCUPIED_THRESHOLD;
}

bool segment_is_blocked_dda(
    const nav_msgs::msg::OccupancyGrid& collision_map,
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& end,
    const occupancy_grid::SegmentCheckOptions& options)
{
    const double resolution = collision_map.info.resolution;
    const double origin_x = collision_map.info.origin.position.x;
    const double origin_y = collision_map.info.origin.position.y;
    int cell_x = static_cast<int>(std::floor((start.x - origin_x) / resolution));
    int cell_y = static_cast<int>(std::floor((start.y - origin_y) / resolution));
    const int end_x = static_cast<int>(std::floor((end.x - origin_x) / resolution));
    const int end_y = static_cast<int>(std::floor((end.y - origin_y) / resolution));
    if (occupancy_grid::xy_index_to_array_index(collision_map, cell_x, cell_y) < 0 ||
        occupancy_grid::xy_index_to_array_index(collision_map, end_x, end_y) < 0)
    {
        return true;
    }

    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const int step_x = (dx > 0.0) - (dx < 0.0);
    const int step_y = (dy > 0.0) - (dy < 0.0);
    const double infinity = std::numeric_limits<double>::infinity();
    const double t_delta_x = step_x == 0 ? infinity : resolution / std::abs(dx);
    const double t_delta_y = step_y == 0 ? infinity : resolution / std::abs(dy);
    const double next_x = origin_x + (cell_x + (step_x > 0 ? 1 : 0)) * resolution;
    const double next_y = origin_y + (cell_y + (step_y > 0 ? 1 : 0)) * resolution;
    double t_max_x = step_x == 0 ? infinity : (next_x - start.x) / dx;
    double t_max_y = step_y == 0 ? infinity : (next_y - start.y) / dy;
    t_max_x = std::max(0.0, t_max_x);
    t_max_y = std::max(0.0, t_max_y);

    const bool escape_enabled = options.escape_reference_map != nullptr &&
        options.start_escape_distance > 0.0 &&
        cell_is_occupied(collision_map, cell_x, cell_y) &&
        !cell_is_occupied(*options.escape_reference_map, cell_x, cell_y);
    const double escape_distance_squared =
        options.start_escape_distance * options.start_escape_distance;
    double entry_t = 0.0;

    while (true)
    {
        if (cell_is_occupied(collision_map, cell_x, cell_y))
        {
            const double x = start.x + std::min(1.0, entry_t) * dx;
            const double y = start.y + std::min(1.0, entry_t) * dy;
            const double escape_dx = x - start.x;
            const double escape_dy = y - start.y;
            const bool inside_escape_region =
                escape_dx * escape_dx + escape_dy * escape_dy <=
                escape_distance_squared;
            if (!(escape_enabled && inside_escape_region &&
                  !cell_is_occupied(
                      *options.escape_reference_map, cell_x, cell_y)))
            {
                return true;
            }
        }
        if (cell_x == end_x && cell_y == end_y)
        {
            return false;
        }

        if (t_max_x < t_max_y)
        {
            entry_t = t_max_x;
            t_max_x += t_delta_x;
            cell_x += step_x;
        }
        else if (t_max_y < t_max_x)
        {
            entry_t = t_max_y;
            t_max_y += t_delta_y;
            cell_y += step_y;
        }
        else
        {
            entry_t = t_max_x;
            t_max_x += t_delta_x;
            t_max_y += t_delta_y;
            cell_x += step_x;
            cell_y += step_y;
        }
    }
}

}  // namespace

int occupancy_grid::xy_index_to_array_index(const nav_msgs::msg::OccupancyGrid& grid, const int i_x, const int i_y)
{
    if (i_x < 0 || i_y < 0 || i_x >= int(grid.info.width) || i_y >= int(grid.info.height))
    {
        return -1;
    }

    const int array_index = i_y * int(grid.info.width) + i_x;
    if (array_index >= int(grid.data.size()))
    {
        return -1;
    }
    return array_index;
}

int occupancy_grid::xy_coord_to_array_index(const nav_msgs::msg::OccupancyGrid& grid, const float x, const float y)
{
    if (grid.info.resolution <= 0.0)
    {
        return -1;
    }

    int i_x = static_cast<int>(std::floor((x - grid.info.origin.position.x) / grid.info.resolution));
    int i_y = static_cast<int>(std::floor((y - grid.info.origin.position.y) / grid.info.resolution));
    return xy_index_to_array_index(grid, i_x, i_y);
}

std::pair<int, int> occupancy_grid::array_index_to_xy_index(const nav_msgs::msg::OccupancyGrid& grid, const int i)
{
    int i_y = i / grid.info.width;
    int i_x = i - i_y * grid.info.width;
    return {i_x, i_y};
}

float occupancy_grid::array_index_to_x_coord(const nav_msgs::msg::OccupancyGrid& grid, const int i)
{
    return grid.info.origin.position.x + array_index_to_xy_index(grid, i).first * grid.info.resolution;
}

float occupancy_grid::array_index_to_y_coord(const nav_msgs::msg::OccupancyGrid& grid, const int i)
{
    return grid.info.origin.position.y + array_index_to_xy_index(grid, i).second * grid.info.resolution;
}

bool occupancy_grid::is_xy_coord_occupied(const nav_msgs::msg::OccupancyGrid& grid, const float x, const float y)
{
    const int array_index = xy_coord_to_array_index(grid, x, y);
    if (array_index < 0)
    {
        return true;
    }
    return int(grid.data.at(array_index)) > OCCUPIED_THRESHOLD;
}

void occupancy_grid::set_xy_coord_occupied(nav_msgs::msg::OccupancyGrid& grid, const float x, const float y)
{
    const int array_index = xy_coord_to_array_index(grid, x, y);
    if (array_index >= 0)
    {
        grid.data.at(array_index) = 100;
    }
}

std::vector<int> occupancy_grid::inflate_cell(nav_msgs::msg::OccupancyGrid &grid, const int i, const float margin, const int val)
{
    std::vector<int> changes;
    if (i < 0 || i >= int(grid.data.size()) || grid.info.width == 0 || grid.info.height == 0 || grid.info.resolution <= 0.0 || margin < 0.0)
    {
        return changes;
    }

    int margin_cell = static_cast<int>(std::ceil(margin / grid.info.resolution));
    std::pair<int, int> xy_index = array_index_to_xy_index(grid, i);
    for (int x = std::max(0, xy_index.first - margin_cell); x <= std::min(int(grid.info.width) - 1, xy_index.first + margin_cell); x++)
    {
        for (int y = std::max(0, xy_index.second - margin_cell); y <= std::min(int(grid.info.height) - 1, xy_index.second + margin_cell); y++)
        {
            const int array_index = xy_index_to_array_index(grid, x, y);
            if (array_index < 0)
            {
                continue;
            }
            if (grid.data.at(array_index) < OCCUPIED_THRESHOLD && grid.data.at(array_index) != val)
            {
                grid.data.at(array_index) = val;
                changes.emplace_back(array_index);
            }
        }
    }
    return changes;
}

void occupancy_grid::inflate_map(nav_msgs::msg::OccupancyGrid& grid, const float margin)
{
    std::vector<int> occupied_indices;
    int size_grid_data = grid.data.size();
    for (int i = 0; i < size_grid_data; i++)
    {
        if (grid.data.at(i) > OCCUPIED_THRESHOLD)
        {
            occupied_indices.emplace_back(i);
        }
    }
    
    int size_occupied_indices = occupied_indices.size();
    for (int i = 0; i < size_occupied_indices; i++)
    {
        inflate_cell(grid, occupied_indices.at(i), margin, 100);
    }
}

bool occupancy_grid::segment_is_blocked(
    const nav_msgs::msg::OccupancyGrid& collision_map,
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& end,
    const SegmentCheckOptions& options)
{
    if (collision_map.info.resolution <= 0.0)
    {
        return true;
    }

    return segment_is_blocked_dda(collision_map, start, end, options);
}

bool occupancy_grid::polyline_is_blocked(
    const nav_msgs::msg::OccupancyGrid& collision_map,
    const std::vector<geometry_msgs::msg::Point>& points)
{
    if (points.empty())
    {
        return true;
    }
    if (points.size() == 1)
    {
        return is_xy_coord_occupied(
            collision_map, points.front().x, points.front().y);
    }
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        if (segment_is_blocked(collision_map, points.at(i - 1), points.at(i)))
        {
            return true;
        }
    }
    return false;
}

std::optional<double> occupancy_grid::distance_to_first_collision(
    const nav_msgs::msg::OccupancyGrid& collision_map,
    const std::vector<geometry_msgs::msg::Point>& points)
{
    if (points.empty())
    {
        return std::nullopt;
    }
    if (collision_map.info.resolution <= 0.0)
    {
        return 0.0;
    }
    if (is_xy_coord_occupied(
            collision_map, points.front().x, points.front().y))
    {
        return 0.0;
    }

    double accumulated_distance = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        const auto& start = points.at(i - 1);
        const auto& end = points.at(i);
        const double segment_length = std::hypot(
            end.x - start.x, end.y - start.y);
        const int sample_count = std::max(
            1, static_cast<int>(
                std::ceil(segment_length / collision_map.info.resolution)));
        for (int sample = 1; sample <= sample_count; ++sample)
        {
            const double ratio =
                static_cast<double>(sample) / sample_count;
            const double x = start.x + ratio * (end.x - start.x);
            const double y = start.y + ratio * (end.y - start.y);
            if (is_xy_coord_occupied(collision_map, x, y))
            {
                return accumulated_distance + ratio * segment_length;
            }
        }
        accumulated_distance += segment_length;
    }
    return std::nullopt;
}
