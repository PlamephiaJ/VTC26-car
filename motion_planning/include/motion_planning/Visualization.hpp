/*
 * @Author: Yuhao Chen
 * @Date: 2024-06-24 16:27:07
 * @LastEditors: Yuhao Chen
 * @LastEditTime: 2024-07-01 19:18:06
 * @Description: Visualization class.
 */

#pragma once

#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

/** Accumulates points and publishes them as one RViz POINTS marker. */
class PointsVisualizer {
protected:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub;
    visualization_msgs::msg::Marker dots;
    std::string ns;
    std::string frame_id;

public:
    /**
     * Input: marker publisher, namespace, frame, color, and point scale.
     * Operation: prepares marker metadata; it does not publish immediately.
     */
    PointsVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, std::string ns, std::string frame_id, std_msgs::msg::ColorRGBA color, float scale);

    /** Append one marker point without publishing. */
    void add_point(geometry_msgs::msg::Point p);

    /** Publish accumulated points and optionally clear them afterward. */
    void publish_points(bool clear_after_publish = true);
};

/** Publishes one pose-based RViz marker such as a goal or lookahead sphere. */
class MarkerVisualizer {
protected:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub;
    visualization_msgs::msg::Marker dot;
    std::string ns;
    std::string frame_id;

public:
    /**
     * Input: marker publisher, namespace, frame, color, scale, and marker type.
     * Operation: prepares marker metadata; it does not publish immediately.
     */
    MarkerVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, std::string ns, std::string frame_id, std_msgs::msg::ColorRGBA color, float scale, int shape);

    /** Replace the marker pose without publishing. */
    void set_pose(geometry_msgs::msg::Pose pose);

    /** Publish the marker at its current pose. */
    void publish_marker();
};

#endif
