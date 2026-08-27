#ifndef MOTION_PLANNING__OPTIMAL_TRAJECTORY_HPP_
#define MOTION_PLANNING__OPTIMAL_TRAJECTORY_HPP_

#include "geometry_msgs/msg/point.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace optimal_trajectory
{

/** Closest-point projection of a vehicle position onto the trajectory. */
struct Projection
{
    geometry_msgs::msg::Point point;
    double progress = 0.0;
    double distance = 0.0;
    std::size_t segment_index = 0;
};

/**
 * Precomputed cyclic optimal trajectory parameterized by planar arc length.
 *
 * Construction removes consecutive duplicate points and precomputes segment
 * lengths/cumulative progress once. Runtime projection, sampling, and slicing
 * therefore do not rebuild trajectory geometry.
 */
class Trajectory
{
public:
    /**
     * Input: ordered map-frame waypoints describing one closed driving lap.
     * Operation: stores a cleaned copy and adds the implicit last-to-first
     * segment. Throws std::invalid_argument if fewer than two distinct points
     * remain or the resulting lap length is zero.
     */
    explicit Trajectory(
        const std::vector<geometry_msgs::msg::Point>& waypoints);

    /** Return the full closed-lap arc length in map units. */
    double total_length() const;

    /** Normalize arbitrary progress into [0, total_length). */
    double normalize_progress(double progress) const;

    /**
     * Project a map-frame point onto the trajectory.
     *
     * Input: query point, optional previous progress hint, backward/forward arc
     * search windows, and a non-negative fallback-distance threshold.
     * Return: closest projection and its cyclic progress. With a hint, only
     * nearby segments are searched first for efficiency and progress
     * continuity. If that result is farther than `fallback_distance`, a global
     * search is performed to support teleports/repositioning.
     */
    Projection project(
        const geometry_msgs::msg::Point& query,
        std::optional<double> progress_hint,
        double backward_search_distance,
        double forward_search_distance,
        double fallback_distance) const;

    /**
     * Sample an exact point at cyclic arc-length progress.
     * Input progress may be negative or greater than one lap.
     */
    geometry_msgs::msg::Point point_at(double progress) const;

    /**
     * Extract a forward map-frame polyline by arc length.
     *
     * Input: start progress and non-negative forward distance.
     * Return: exact start point, crossed original waypoints, and exact end
     * point. Distance is capped at one lap to avoid duplicate laps.
     */
    std::vector<geometry_msgs::msg::Point> slice(
        double start_progress, double forward_distance) const;

private:
    Projection project_globally(
        const geometry_msgs::msg::Point& query) const;
    Projection project_on_segments(
        const geometry_msgs::msg::Point& query,
        const std::vector<std::size_t>& segment_indices) const;
    std::size_t segment_at(double normalized_progress) const;
    std::vector<std::size_t> local_segment_indices(
        double normalized_hint, double backward_distance,
        double forward_distance) const;

    std::vector<geometry_msgs::msg::Point> points_;
    std::vector<double> segment_lengths_;
    std::vector<double> cumulative_lengths_;
    double total_length_ = 0.0;
};

}  // namespace optimal_trajectory

#endif  // MOTION_PLANNING__OPTIMAL_TRAJECTORY_HPP_
