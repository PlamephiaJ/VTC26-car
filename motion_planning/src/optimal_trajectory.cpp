#include "motion_planning/optimal_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace optimal_trajectory
{

namespace
{

constexpr double epsilon = 1e-9;

double distance_between(
    const geometry_msgs::msg::Point& first,
    const geometry_msgs::msg::Point& second)
{
    return std::hypot(first.x - second.x, first.y - second.y);
}

}  // namespace

Trajectory::Trajectory(
    const std::vector<geometry_msgs::msg::Point>& waypoints)
{
    for (const auto& waypoint : waypoints)
    {
        if (points_.empty() ||
            distance_between(points_.back(), waypoint) > epsilon)
        {
            points_.emplace_back(waypoint);
        }
    }
    if (points_.size() > 1 &&
        distance_between(points_.front(), points_.back()) <= epsilon)
    {
        points_.pop_back();
    }
    if (points_.size() < 2)
    {
        throw std::invalid_argument(
            "Optimal trajectory requires at least two distinct points");
    }

    cumulative_lengths_.emplace_back(0.0);
    for (std::size_t i = 0; i < points_.size(); ++i)
    {
        const std::size_t next = (i + 1) % points_.size();
        const double length = distance_between(points_.at(i), points_.at(next));
        if (length <= epsilon)
        {
            throw std::invalid_argument(
                "Optimal trajectory contains a zero-length closing segment");
        }
        segment_lengths_.emplace_back(length);
        total_length_ += length;
        cumulative_lengths_.emplace_back(total_length_);
    }
}

double Trajectory::total_length() const
{
    return total_length_;
}

double Trajectory::normalize_progress(const double progress) const
{
    double normalized = std::fmod(progress, total_length_);
    if (normalized < 0.0)
    {
        normalized += total_length_;
    }
    return normalized;
}

std::size_t Trajectory::segment_at(const double normalized_progress) const
{
    const auto upper = std::upper_bound(
        cumulative_lengths_.begin(), cumulative_lengths_.end(),
        normalized_progress);
    const std::size_t index = static_cast<std::size_t>(
        std::distance(cumulative_lengths_.begin(), upper) - 1);
    return std::min(index, segment_lengths_.size() - 1);
}

geometry_msgs::msg::Point Trajectory::point_at(const double progress) const
{
    const double normalized = normalize_progress(progress);
    const std::size_t segment = segment_at(normalized);
    const double offset = normalized - cumulative_lengths_.at(segment);
    const double ratio = offset / segment_lengths_.at(segment);
    const auto& start = points_.at(segment);
    const auto& end = points_.at((segment + 1) % points_.size());

    geometry_msgs::msg::Point result;
    result.x = start.x + ratio * (end.x - start.x);
    result.y = start.y + ratio * (end.y - start.y);
    result.z = start.z + ratio * (end.z - start.z);
    return result;
}

Projection Trajectory::project_on_segments(
    const geometry_msgs::msg::Point& query,
    const std::vector<std::size_t>& segment_indices) const
{
    Projection best;
    best.distance = std::numeric_limits<double>::max();
    for (const std::size_t segment : segment_indices)
    {
        const auto& start = points_.at(segment);
        const auto& end = points_.at((segment + 1) % points_.size());
        const double dx = end.x - start.x;
        const double dy = end.y - start.y;
        const double length_squared = dx * dx + dy * dy;
        const double unclamped_ratio =
            ((query.x - start.x) * dx + (query.y - start.y) * dy) /
            length_squared;
        const double ratio = std::clamp(unclamped_ratio, 0.0, 1.0);

        geometry_msgs::msg::Point projected;
        projected.x = start.x + ratio * dx;
        projected.y = start.y + ratio * dy;
        projected.z = start.z + ratio * (end.z - start.z);
        const double distance = distance_between(query, projected);
        if (distance < best.distance)
        {
            best.point = projected;
            best.distance = distance;
            best.segment_index = segment;
            best.progress = normalize_progress(
                cumulative_lengths_.at(segment) +
                ratio * segment_lengths_.at(segment));
        }
    }
    return best;
}

Projection Trajectory::project_globally(
    const geometry_msgs::msg::Point& query) const
{
    std::vector<std::size_t> all_segments(segment_lengths_.size());
    for (std::size_t i = 0; i < all_segments.size(); ++i)
    {
        all_segments.at(i) = i;
    }
    return project_on_segments(query, all_segments);
}

std::vector<std::size_t> Trajectory::local_segment_indices(
    const double normalized_hint, const double backward_distance,
    const double forward_distance) const
{
    if (backward_distance + forward_distance >= total_length_)
    {
        std::vector<std::size_t> all_segments(segment_lengths_.size());
        for (std::size_t i = 0; i < all_segments.size(); ++i)
        {
            all_segments.at(i) = i;
        }
        return all_segments;
    }

    const std::size_t hinted_segment = segment_at(normalized_hint);
    std::vector<std::size_t> result{hinted_segment};
    const auto already_selected = [&result](const std::size_t index)
    {
        return std::find(result.begin(), result.end(), index) != result.end();
    };

    double forward_to_boundary =
        cumulative_lengths_.at(hinted_segment + 1) - normalized_hint;
    std::size_t segment = (hinted_segment + 1) % segment_lengths_.size();
    while (forward_to_boundary <= forward_distance &&
           !already_selected(segment))
    {
        result.emplace_back(segment);
        forward_to_boundary += segment_lengths_.at(segment);
        segment = (segment + 1) % segment_lengths_.size();
    }

    double backward_to_boundary =
        normalized_hint - cumulative_lengths_.at(hinted_segment);
    segment = hinted_segment == 0 ?
        segment_lengths_.size() - 1 : hinted_segment - 1;
    while (backward_to_boundary <= backward_distance &&
           !already_selected(segment))
    {
        result.emplace_back(segment);
        backward_to_boundary += segment_lengths_.at(segment);
        segment = segment == 0 ? segment_lengths_.size() - 1 : segment - 1;
    }
    return result;
}

Projection Trajectory::project(
    const geometry_msgs::msg::Point& query,
    const std::optional<double> progress_hint,
    const double backward_search_distance,
    const double forward_search_distance,
    const double fallback_distance) const
{
    if (backward_search_distance < 0.0 || forward_search_distance < 0.0 ||
        fallback_distance < 0.0)
    {
        throw std::invalid_argument("Projection search distances must be non-negative");
    }
    if (!progress_hint)
    {
        return project_globally(query);
    }

    const double normalized_hint = normalize_progress(*progress_hint);
    Projection result = project_on_segments(
        query, local_segment_indices(
            normalized_hint, backward_search_distance,
            forward_search_distance));
    if (result.distance > fallback_distance)
    {
        result = project_globally(query);
    }
    return result;
}

std::vector<geometry_msgs::msg::Point> Trajectory::slice(
    const double start_progress, const double forward_distance) const
{
    if (forward_distance < 0.0)
    {
        throw std::invalid_argument("forward_distance must be non-negative");
    }

    const double capped_distance = std::min(forward_distance, total_length_);
    const double normalized_start = normalize_progress(start_progress);
    std::vector<geometry_msgs::msg::Point> result;
    result.emplace_back(point_at(normalized_start));
    if (capped_distance <= epsilon)
    {
        return result;
    }

    std::size_t segment = segment_at(normalized_start);
    double distance_to_segment_end =
        cumulative_lengths_.at(segment + 1) - normalized_start;
    double remaining = capped_distance;
    while (remaining > epsilon)
    {
        if (remaining < distance_to_segment_end - epsilon)
        {
            result.emplace_back(point_at(start_progress + capped_distance));
            break;
        }

        result.emplace_back(points_.at((segment + 1) % points_.size()));
        remaining -= distance_to_segment_end;
        segment = (segment + 1) % segment_lengths_.size();
        distance_to_segment_end = segment_lengths_.at(segment);
    }
    return result;
}

}  // namespace optimal_trajectory
