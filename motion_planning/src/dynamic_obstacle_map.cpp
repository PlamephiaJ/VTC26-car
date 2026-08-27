#include "motion_planning/dynamic_obstacle_map.hpp"

#include "motion_planning/occupancy_grid.hpp"

#include <cmath>
#include <chrono>
#include <stdexcept>

namespace dynamic_obstacles
{

std::vector<geometry_msgs::msg::Point> valid_hit_points(
    const sensor_msgs::msg::LaserScan& scan, const double maximum_range)
{
    if (maximum_range <= 0.0)
    {
        throw std::invalid_argument("maximum_range must be positive");
    }

    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(scan.ranges.size());
    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const double range = scan.ranges.at(i);
        if (!std::isfinite(range) || range > maximum_range)
        {
            continue;
        }
        const double angle = scan.angle_min + scan.angle_increment * i;
        geometry_msgs::msg::Point point;
        point.x = range * std::cos(angle);
        point.y = range * std::sin(angle);
        points.emplace_back(point);
    }
    return points;
}

void MapLayer::initialize(
    const nav_msgs::msg::OccupancyGrid& map, const double static_margin)
{
    if (static_margin < 0.0)
    {
        throw std::invalid_argument("static_margin must be non-negative");
    }
    base_map_ = map;
    static_collision_map_ = map;
    occupancy_grid::inflate_map(static_collision_map_, static_margin);
    collision_map_ = static_collision_map_;
    changed_indices_.clear();
    initialized_ = true;
}

bool MapLayer::initialized() const
{
    return initialized_;
}

const nav_msgs::msg::OccupancyGrid& MapLayer::base_map() const
{
    if (!initialized_)
    {
        throw std::logic_error("MapLayer is not initialized");
    }
    return base_map_;
}

const nav_msgs::msg::OccupancyGrid& MapLayer::collision_map() const
{
    if (!initialized_)
    {
        throw std::logic_error("MapLayer is not initialized");
    }
    return collision_map_;
}

std::size_t MapLayer::add_observation(
    const geometry_msgs::msg::Point& map_point,
    const double inflation_margin)
{
    if (!initialized_)
    {
        throw std::logic_error("MapLayer is not initialized");
    }
    if (inflation_margin < 0.0)
    {
        throw std::invalid_argument("inflation_margin must be non-negative");
    }
    if (occupancy_grid::is_xy_coord_occupied(
            base_map_, map_point.x, map_point.y))
    {
        return 0;
    }

    const int center_index = occupancy_grid::xy_coord_to_array_index(
        collision_map_, map_point.x, map_point.y);
    const std::vector<int> changes = occupancy_grid::inflate_cell(
        collision_map_, center_index, inflation_margin, 100);
    changed_indices_.insert(
        changed_indices_.end(), changes.begin(), changes.end());
    return changes.size();
}

RebuildProfile MapLayer::rebuild_observations(
    const std::vector<geometry_msgs::msg::Point>& map_points,
    const double inflation_margin)
{
    using Clock = std::chrono::steady_clock;
    RebuildProfile profile;
    profile.observation_count = map_points.size();
    const auto total_start = Clock::now();
    const auto clear_start = total_start;
    clear_observations();
    const auto raster_start = Clock::now();
    profile.clear = raster_start - clear_start;
    for (const auto& point : map_points)
    {
        profile.changed_cell_count += add_observation(point, inflation_margin);
    }
    const auto end = Clock::now();
    profile.rasterize_and_inflate = end - raster_start;
    profile.total = end - total_start;
    return profile;
}

void MapLayer::clear_observations()
{
    if (!initialized_)
    {
        return;
    }
    for (const int index : changed_indices_)
    {
        if (index >= 0 && index < static_cast<int>(collision_map_.data.size()) &&
            index < static_cast<int>(static_collision_map_.data.size()))
        {
            collision_map_.data.at(index) = static_collision_map_.data.at(index);
        }
    }
    changed_indices_.clear();
}

}  // namespace dynamic_obstacles
