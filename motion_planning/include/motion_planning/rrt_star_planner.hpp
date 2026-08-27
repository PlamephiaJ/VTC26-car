#ifndef MOTION_PLANNING__RRT_STAR_PLANNER_HPP_
#define MOTION_PLANNING__RRT_STAR_PLANNER_HPP_

#include "motion_planning/rrt_tree.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <cstddef>
#include <chrono>
#include <cstdint>
#include <optional>
#include <random>
#include <vector>

namespace rrt_star
{

/** Tunable parameters consumed only by the RRT* algorithm. */
struct PlannerConfig
{
    int minimum_iterations = 1000;
    int maximum_iterations = 1200;
    double sample_standard_deviation = 1.5;
    double step_size = 0.3;
    double near_radius = 1.0;
    double goal_tolerance = 0.1;
    double goal_sample_rate = 0.1;
    double static_margin = 0.2;
    double dynamic_margin = 0.22;
};

/** Exclusive wall-clock segments accumulated inside one plan() call. */
struct PlanProfile
{
    std::chrono::nanoseconds sampling{0};
    std::chrono::nanoseconds nearest{0};
    std::chrono::nanoseconds initial_collision{0};
    std::chrono::nanoseconds near{0};
    std::chrono::nanoseconds parent_collision{0};
    std::chrono::nanoseconds rewiring{0};
    std::chrono::nanoseconds total{0};
};

/** Why a planning call ended without a path. */
enum class PlanFailure
{
    none,
    sampling_failed,
    iteration_limit,
};

/**
 * Complete output of one RRT* planning call.
 *
 * `tree` is always populated for visualization/diagnostics. `path` is ordered
 * root-to-goal-candidate when `success` is true. `goal_candidate_count` helps
 * distinguish exploration failure from final candidate selection.
 */
struct PlanResult
{
    bool success = false;
    PlanFailure failure = PlanFailure::iteration_limit;
    Tree tree;
    std::vector<Node> path;
    std::size_t goal_candidate_count = 0;
    PlanProfile profile;
};

/**
 * ROS-node-independent RRT* planner.
 *
 * The class owns only its random generator and immutable algorithm settings.
 * It does not publish, transform frames, mutate maps, or retain a previous
 * tree, so one `plan()` call corresponds to one local planning cycle.
 */
class Planner
{
public:
    /**
     * Construct a planner.
     *
     * Input: validated configuration and optional deterministic random seed.
     * If no seed is supplied, std::random_device is used.
     */
    explicit Planner(
        const PlannerConfig& config,
        std::optional<std::uint32_t> random_seed = std::nullopt);

    /**
     * Generate one local path.
     *
     * Input: start and goal in the map frame, dynamic collision map, and the
     * original static map used by root-escape collision handling.
     * Return: planning result containing the explored tree and, on success,
     * the root-to-best-candidate path. Input maps are never modified.
     */
    PlanResult plan(
        const Point2D& start, const Point2D& goal,
        const nav_msgs::msg::OccupancyGrid& dynamic_map,
        const nav_msgs::msg::OccupancyGrid& static_map);

private:
    /**
     * Draw a free sample from the goal-biased sampling distribution.
     *
     * Return: a free point, or std::nullopt after 1000 rejected attempts.
     */
    std::optional<Point2D> sample_free_point(
        const Point2D& start, const Point2D& goal,
        const nav_msgs::msg::OccupancyGrid& dynamic_map);

    PlannerConfig config_;
    std::mt19937 random_generator_;
    std::uniform_real_distribution<double> unit_distribution_{0.0, 1.0};
};

}  // namespace rrt_star

#endif  // MOTION_PLANNING__RRT_STAR_PLANNER_HPP_
