#include "motion_planning/reference_path_manager.hpp"

#include "motion_planning/occupancy_grid.hpp"

#include <stdexcept>

namespace reference_path
{

Manager::Manager(
    const std::vector<geometry_msgs::msg::Point>& optimal_waypoints,
    const ManagerConfig& config)
    : trajectory_(optimal_waypoints), config_(config)
{
    if (config_.global_goal_distance <= 0.0 || config_.rejoin_distance < 0.0 ||
        config_.rejoin_clearance_time < 0.0 ||
        config_.progress_search_backward < 0.0 ||
        config_.progress_search_forward < 0.0 ||
        config_.projection_fallback_distance < 0.0)
    {
        throw std::invalid_argument("Invalid reference-path manager configuration");
    }
}

Mode Manager::mode() const
{
    return mode_;
}

void Manager::reset()
{
    mode_ = Mode::optimal_reference;
    progress_hint_.reset();
    rejoin_clear_since_seconds_.reset();
}

Decision Manager::update(
    const geometry_msgs::msg::Point& vehicle_position,
    const nav_msgs::msg::OccupancyGrid& collision_map,
    const double current_time_seconds)
{
    Decision decision;
    decision.projection = trajectory_.project(
        vehicle_position, progress_hint_, config_.progress_search_backward,
        config_.progress_search_forward,
        config_.projection_fallback_distance);
    progress_hint_ = decision.projection.progress;
    decision.global_goal = trajectory_.point_at(
        decision.projection.progress + config_.global_goal_distance);
    decision.local_optimal_reference = trajectory_.slice(
        decision.projection.progress, config_.global_goal_distance);

    decision.optimal_arc_clear = !occupancy_grid::polyline_is_blocked(
        collision_map, decision.local_optimal_reference);
    decision.vehicle_near_optimal =
        decision.projection.distance <= config_.rejoin_distance;
    decision.rejoin_connector_clear = !occupancy_grid::segment_is_blocked(
        collision_map, vehicle_position, decision.projection.point);

    const Mode previous_mode = mode_;
    if (mode_ == Mode::optimal_reference && !decision.optimal_arc_clear)
    {
        mode_ = Mode::rrt_detour;
        rejoin_clear_since_seconds_.reset();
    }
    else if (mode_ == Mode::rrt_detour)
    {
        const bool safe_to_rejoin = decision.optimal_arc_clear &&
            decision.vehicle_near_optimal &&
            decision.rejoin_connector_clear;
        if (!safe_to_rejoin)
        {
            rejoin_clear_since_seconds_.reset();
        }
        else
        {
            if (!rejoin_clear_since_seconds_ ||
                current_time_seconds < *rejoin_clear_since_seconds_)
            {
                rejoin_clear_since_seconds_ = current_time_seconds;
            }
            decision.rejoin_clearance_elapsed =
                current_time_seconds - *rejoin_clear_since_seconds_;
            if (decision.rejoin_clearance_elapsed >=
                config_.rejoin_clearance_time)
            {
                mode_ = Mode::optimal_reference;
                rejoin_clear_since_seconds_.reset();
            }
        }
    }

    decision.mode = mode_;
    decision.mode_changed = previous_mode != mode_;
    return decision;
}

}  // namespace reference_path
