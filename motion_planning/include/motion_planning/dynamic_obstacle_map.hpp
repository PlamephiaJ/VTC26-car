#ifndef MOTION_PLANNING__DYNAMIC_OBSTACLE_MAP_HPP_
#define MOTION_PLANNING__DYNAMIC_OBSTACLE_MAP_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <vector>
#include <chrono>

namespace dynamic_obstacles
{

/** Measurements for one clear/rasterize/inflate dynamic-map rebuild. */
struct RebuildProfile
{
    std::chrono::nanoseconds clear{0};
    std::chrono::nanoseconds rasterize_and_inflate{0};
    std::chrono::nanoseconds total{0};
    std::size_t observation_count = 0;
    std::size_t changed_cell_count = 0;
};

/**
 * Convert valid planar laser ranges into points in the laser frame.
 *
 * Input: LaserScan message and positive maximum accepted range.
 * Return: Cartesian hit points for finite ranges at or below the limit.
 * NaN, infinity, and farther readings are omitted.
 */
std::vector<geometry_msgs::msg::Point> valid_hit_points(
    const sensor_msgs::msg::LaserScan& scan, double maximum_range);

/**
 * Owns the immutable base map and the mutable live-obstacle collision layer.
 *
 * Together with valid_hit_points(), this module owns the complete scan-to-map
 * obstacle pipeline except TF, which remains in the ROS node.
 */
class MapLayer
{
public:
    /**
     * Initialize from a newly received base map.
     *
     * Input: source occupancy grid and non-negative static inflation margin.
     * Operation: stores the source map, creates an inflated collision map, and
     * clears all previously recorded live obstacles.
     */
    void initialize(
        const nav_msgs::msg::OccupancyGrid& map, double static_margin);

    /** Return true after initialize() has supplied a usable map. */
    bool initialized() const;

    /** Return the unmodified source map. Throws if not initialized. */
    const nav_msgs::msg::OccupancyGrid& base_map() const;

    /** Return the inflated map including current live obstacles. */
    const nav_msgs::msg::OccupancyGrid& collision_map() const;

    /**
     * Insert one observed obstacle.
     *
     * Input: hit point in map coordinates and non-negative inflation margin.
     * Operation: if the base map cell is free, marks/inflates the corresponding
     * collision-map cell and records changed indices for later restoration.
     * Return: number of collision-map cells newly changed by this observation.
     */
    std::size_t add_observation(
        const geometry_msgs::msg::Point& map_point, double inflation_margin);

    /**
     * Rebuild the live layer from a complete fixed-rate observation window.
     * Return: independently measured clear, rasterize/inflate, and total time.
     */
    RebuildProfile rebuild_observations(
        const std::vector<geometry_msgs::msg::Point>& map_points,
        double inflation_margin);

    /**
     * Remove every live obstacle added since the previous clear.
     *
     * Operation: restores recorded cells from the statically inflated map and
     * empties the change list. Static obstacles remain unchanged.
     */
    void clear_observations();

private:
    nav_msgs::msg::OccupancyGrid base_map_;
    nav_msgs::msg::OccupancyGrid collision_map_;
    nav_msgs::msg::OccupancyGrid static_collision_map_;
    std::vector<int> changed_indices_;
    bool initialized_ = false;
};

}  // namespace dynamic_obstacles

#endif  // MOTION_PLANNING__DYNAMIC_OBSTACLE_MAP_HPP_
