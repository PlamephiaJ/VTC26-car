#include "motion_planning/path_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace path_tracking
{

std::vector<geometry_msgs::msg::Point> nodes_to_points(
    const std::vector<rrt_star::Node>& nodes)
{
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(nodes.size());
    for (const auto& node : nodes)
    {
        geometry_msgs::msg::Point point;
        point.x = node.x;
        point.y = node.y;
        point.z = 0.0;
        points.emplace_back(point);
    }
    return points;
}

std::vector<geometry_msgs::msg::Point> resample_polyline(
    const std::vector<geometry_msgs::msg::Point>& points,
    const double maximum_spacing)
{
    if (maximum_spacing <= 0.0)
    {
        throw std::invalid_argument("maximum_spacing must be positive");
    }
    if (points.size() < 2)
    {
        return points;
    }

    std::vector<geometry_msgs::msg::Point> result;
    for (std::size_t i = 0; i + 1 < points.size(); ++i)
    {
        const auto& start = points.at(i);
        const auto& end = points.at(i + 1);
        result.emplace_back(start);
        const double length = std::hypot(end.x - start.x, end.y - start.y);
        if (length < maximum_spacing)
        {
            continue;
        }

        const int segment_count = static_cast<int>(
            std::ceil(length / maximum_spacing));
        for (int j = 1; j < segment_count; ++j)
        {
            const double ratio = static_cast<double>(j) / segment_count;
            geometry_msgs::msg::Point interpolated;
            interpolated.x = start.x + ratio * (end.x - start.x);
            interpolated.y = start.y + ratio * (end.y - start.y);
            interpolated.z = start.z + ratio * (end.z - start.z);
            result.emplace_back(interpolated);
        }
    }
    result.emplace_back(points.back());
    return result;
}

nav_msgs::msg::Path to_path_message(
    const std::vector<geometry_msgs::msg::Point>& points,
    const std::string& frame_id)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.poses.reserve(points.size());
    for (const auto& point : points)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.pose.position = point;
        path.poses.emplace_back(pose);
    }
    return path;
}

std::optional<geometry_msgs::msg::Point> point_at_distance(
    const nav_msgs::msg::Path& path, const double distance)
{
    if (path.poses.empty())
    {
        return std::nullopt;
    }

    if (distance <= 0.0)
    {
        return path.poses.front().pose.position;
    }

    double accumulated_distance = 0.0;
    for (std::size_t i = 1; i < path.poses.size(); ++i)
    {
        const auto& segment_start = path.poses.at(i - 1).pose.position;
        const auto& segment_end = path.poses.at(i).pose.position;
        const double segment_length = std::hypot(
            segment_end.x - segment_start.x,
            segment_end.y - segment_start.y);

        if (segment_length <= std::numeric_limits<double>::epsilon())
        {
            continue;
        }

        if (accumulated_distance + segment_length >= distance)
        {
            const double ratio = std::clamp(
                (distance - accumulated_distance) / segment_length, 0.0, 1.0);
            geometry_msgs::msg::Point target;
            target.x = segment_start.x + ratio * (segment_end.x - segment_start.x);
            target.y = segment_start.y + ratio * (segment_end.y - segment_start.y);
            target.z = segment_start.z + ratio * (segment_end.z - segment_start.z);
            return target;
        }

        accumulated_distance += segment_length;
    }

    return path.poses.back().pose.position;
}

double pure_pursuit_steering(
    const double target_lateral_offset, const double lookahead_distance,
    const double proportional_gain, const double steering_limit)
{
    if (lookahead_distance <= 0.0 || proportional_gain <= 0.0 ||
        steering_limit <= 0.0)
    {
        throw std::invalid_argument(
            "lookahead, gain, and steering limit must be positive");
    }
    const double steering = proportional_gain * 2.0 * target_lateral_offset /
        (lookahead_distance * lookahead_distance);
    return std::clamp(steering, -steering_limit, steering_limit);
}

double speed_for_steering(
    const double steering_angle, const SpeedProfile& profile)
{
    constexpr double pi = 3.14159265358979323846;
    const double degrees = std::abs(steering_angle) * 180.0 / pi;
    if (degrees <= profile.low_steering_threshold_degrees)
    {
        return profile.straight_speed;
    }
    if (degrees <= profile.medium_steering_threshold_degrees)
    {
        return profile.medium_turn_speed;
    }
    return profile.sharp_turn_speed;
}

double blocked_path_speed_limit(
    const double collision_distance, const double stop_distance,
    const double gain)
{
    if (collision_distance < 0.0 || stop_distance < 0.0 || gain <= 0.0)
    {
        throw std::invalid_argument(
            "collision distance and stop distance must be non-negative; "
            "gain must be positive");
    }
    return std::max(0.0, (collision_distance - stop_distance) * gain);
}

}  // namespace path_tracking
