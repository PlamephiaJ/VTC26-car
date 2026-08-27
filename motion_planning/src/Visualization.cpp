/*
 * @Author: Yuhao Chen
 * @Date: 2024-06-24 16:34:41
 * @LastEditors: Yuhao Chen
 * @LastEditTime: 2024-06-25 11:22:58
 * @Description: Visualization source file.
 */

#include "../include/motion_planning/Visualization.hpp"

static int num_visuals = 0;

PointsVisualizer::PointsVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, std::string ns, std::string frame_id, std_msgs::msg::ColorRGBA color, float scale): pub(pub), ns(ns), frame_id(frame_id)
{
    dots.header.frame_id = frame_id;
    dots.ns = ns;
    dots.action = visualization_msgs::msg::Marker::ADD;
    dots.pose.orientation.w = 1.0;
    dots.id = num_visuals;
    dots.type = visualization_msgs::msg::Marker::POINTS;
    dots.scale.x = dots.scale.y = dots.scale.z = scale;
    dots.color = color;
    num_visuals++;
}

void PointsVisualizer::add_point(geometry_msgs::msg::Point p)
{
    dots.points.emplace_back(p);
}

void PointsVisualizer::publish_points(bool clear_after_publish)
{
    pub->publish(dots);
    if (clear_after_publish)
    {
        dots.points.clear();
    }
}

MarkerVisualizer::MarkerVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, std::string ns, std::string frame_id, std_msgs::msg::ColorRGBA color, float scale, int shape): pub(pub), ns(ns), frame_id(frame_id)
{
    dot.header.frame_id = frame_id;
    dot.ns = ns;
    dot.action = visualization_msgs::msg::Marker::ADD;
    dot.id = num_visuals;
    dot.type = shape;
    dot.scale.x = dot.scale.y = dot.scale.z = scale;
    dot.color = color;
    num_visuals++;
}

void MarkerVisualizer::set_pose(geometry_msgs::msg::Pose pose)
{
    dot.pose.orientation = pose.orientation;
    dot.pose.position = pose.position;
}

void MarkerVisualizer::publish_marker()
{
    pub->publish(dot);
}
