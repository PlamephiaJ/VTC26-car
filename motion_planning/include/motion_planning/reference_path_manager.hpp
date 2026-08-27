#ifndef MOTION_PLANNING__REFERENCE_PATH_MANAGER_HPP_
#define MOTION_PLANNING__REFERENCE_PATH_MANAGER_HPP_

#include "motion_planning/optimal_trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <optional>
#include <vector>

namespace reference_path
{

/** Active source of the local path followed by the controller. */
enum class Mode
{
    optimal_reference,
    rrt_detour,
};

/** Parameters governing progress tracking and safe mode transitions. */
struct ManagerConfig
{
    double global_goal_distance = 3.5;
    double rejoin_distance = 0.5;
    double rejoin_clearance_time = 0.75;
    double progress_search_backward = 2.0;
    double progress_search_forward = 8.0;
    double projection_fallback_distance = 2.5;
};

/** Complete reference-path decision for one vehicle update. */
struct Decision
{
    Mode mode = Mode::optimal_reference;
    bool mode_changed = false;
    optimal_trajectory::Projection projection;
    geometry_msgs::msg::Point global_goal;
    std::vector<geometry_msgs::msg::Point> local_optimal_reference;
    bool optimal_arc_clear = false;
    bool vehicle_near_optimal = false;
    bool rejoin_connector_clear = false;
    double rejoin_clearance_elapsed = 0.0;
};

/**
 * Selects between the precomputed optimal trajectory and an RRT* detour.
 *
 * This class owns the cyclic progress hint and mode state, but does not run
 * RRT*, publish ROS messages, or modify the collision map. The ROS node uses a
 * decision in `rrt_detour` mode to invoke the separate RRT* planner with the
 * returned `global_goal`.
 */
class Manager
{
public:
    /**
     * Input: precomputed closed optimal trajectory and validated thresholds.
     * Effect: starts in optimal-reference mode with no progress hint.
     */
    Manager(
        const std::vector<geometry_msgs::msg::Point>& optimal_waypoints,
        const ManagerConfig& config);

    /**
     * Update progress, global goal, collision state, and reference mode.
     *
     * Input: current vehicle position and current inflated collision map.
     * Return:
     * - global goal at `progress + global_goal_distance` in both modes;
     * - optimal trajectory slice from current projection to that goal;
     * - mode and individual safety-condition results.
     *
     * Transition rules:
     * - optimal -> RRT when the optimal arc to the global goal is blocked;
     * - RRT -> optimal only when that arc and the short connector remain clear
     *   for the configured clearance time while the vehicle stays within the
     *   rejoin distance.
     */
    Decision update(
        const geometry_msgs::msg::Point& vehicle_position,
        const nav_msgs::msg::OccupancyGrid& collision_map,
        double current_time_seconds);

    /** Return the currently active reference source without updating state. */
    Mode mode() const;

    /** Clear progress history and restore the default optimal-reference mode. */
    void reset();

private:
    optimal_trajectory::Trajectory trajectory_;
    ManagerConfig config_;
    Mode mode_ = Mode::optimal_reference;
    std::optional<double> progress_hint_;
    std::optional<double> rejoin_clear_since_seconds_;
};

}  // namespace reference_path

#endif  // MOTION_PLANNING__REFERENCE_PATH_MANAGER_HPP_
