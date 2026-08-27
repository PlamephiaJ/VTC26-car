#ifndef MOTION_PLANNING__RRT_HPP_
#define MOTION_PLANNING__RRT_HPP_

#include "motion_planning/Visualization.hpp"
#include "motion_planning/dynamic_obstacle_map.hpp"
#include "motion_planning/path_tracking.hpp"
#include "motion_planning/reference_path_manager.hpp"
#include "motion_planning/rrt_star_planner.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <chrono>
#include <cstddef>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

/**
 * ROS2 orchestration node for local motion planning.
 *
 * Responsibilities intentionally kept here:
 * - load ROS parameters and own publishers/subscribers;
 * - obtain TF transforms and translate messages between frames;
 * - invoke the goal selector, RRT* planner, path processor, and controller;
 * - publish maps, visualization markers, and vehicle commands.
 *
 * Algorithm implementations live in focused modules under
 * include/motion_planning so they can be located, tested, and optimized
 * without modifying ROS callback code.
 */
class RRT : public rclcpp::Node
{
public:
    RRT();
    ~RRT() override;

private:
    // Map and perception parameters.
    double map_inflation_margin_ = 0.2;
    double goal_ahead_distance_ = 3.5;
    double scan_range_ = 4.0;
    double detected_obstacle_margin_ = 0.22;

    // RRT* algorithm parameters.
    int maximum_rrt_iterations_ = 1200;
    int minimum_rrt_iterations_ = 1000;
    double sample_standard_deviation_ = 1.5;
    double step_size_ = 0.3;
    double near_range_ = 1.0;
    double goal_tolerance_ = 0.1;
    double goal_sample_rate_ = 0.1;
    double rrt_waypoint_interval_ = 0.2;

    // Optimal-trajectory progress and safe rejoin parameters.
    double optimal_rejoin_distance_ = 0.5;
    double optimal_rejoin_clear_time_ = 0.75;
    double progress_search_backward_ = 2.0;
    double progress_search_forward_ = 8.0;
    double projection_fallback_distance_ = 2.5;

    // Path tracking parameters.
    double lookahead_distance_ = 0.4;
    double pursuit_gain_ = 0.25;
    double steering_limit_ = 0.41;
    path_tracking::SpeedProfile speed_profile_;
    std::vector<double> visualization_primary_color_{0.1, 0.65, 1.0};
    std::vector<double> visualization_accent_color_{0.0, 1.0, 0.65};

    // Dynamic-obstacle rolling window.
    double dynamic_obstacle_persistence_ = 0.3;
    double dynamic_map_update_period_ = 0.05;

    // LiDAR-only fallback speed control when no RRT* detour is available.
    double blocked_path_stop_distance_ = 0.5;
    double blocked_path_speed_gain_ = 2.0;

    // ROS names and runtime state.
    std::string odom_topic_ = "/ego_racecar/odom";
    std::string map_topic_ = "/map";
    std::string scan_topic_ = "/scan";
    std::string dynamic_map_topic_ = "/ego_racecar/dynamic_map";
    std::string drive_topic_ = "/drive";
    std::string control_topic_ = "/ego_racecar/control";
    std::string fleet_control_topic_ = "/rrt/control";
    std::string waypoint_file_path_;
    bool start_on_launch_ = false;
    bool is_vehicle_enabled_ = false;

    /** Load and validate all ROS parameters. Throws on invalid configuration. */
    void load_parameters();

    /** Create subscriptions, publishers, TF listener, and visualization state. */
    void initialize_ros_interfaces();

    /** Load global waypoints from the configured CSV file. */
    void load_global_waypoints();

    /** Construct algorithm modules after their parameters are available. */
    void initialize_algorithm_modules();

    // ROS callbacks. Each callback performs message/frame orchestration only.
    void control_callback(const std_msgs::msg::String::ConstSharedPtr message);
    void map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message);
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message);
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr message);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        fleet_control_subscriber_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_map_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_publisher_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_node_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_branch_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_publisher_;

    // Focused algorithm/state modules.
    dynamic_obstacles::MapLayer obstacle_map_;
    std::unique_ptr<reference_path::Manager> reference_manager_;
    std::unique_ptr<rrt_star::Planner> planner_;
    std::vector<geometry_msgs::msg::Point> global_waypoints_;
    geometry_msgs::msg::Pose current_pose_;

    struct TimedObstacleFrame
    {
        double stamp_seconds = 0.0;
        std::vector<geometry_msgs::msg::Point> points;
    };
    std::deque<TimedObstacleFrame> obstacle_frames_;
    double last_dynamic_map_update_seconds_ = -1.0;

    // TF is kept in the ROS layer; algorithm modules consume map-frame data.
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string laser_frame_ = "/laser";
    std::string map_frame_ = "map";
    std::string vehicle_frame_ = "/base_link";
    geometry_msgs::msg::TransformStamped laser_to_map_;
    geometry_msgs::msg::TransformStamped map_to_vehicle_;

    /** Refresh the laser-to-map transform required by scan projection. */
    bool lookup_laser_transform();

    /** Refresh the map-to-vehicle transform required by path tracking. */
    bool lookup_vehicle_transform();

    /** Transform one point from laser frame to map frame. */
    geometry_msgs::msg::Point laser_point_to_map(
        const geometry_msgs::msg::Point& laser_point) const;

    /** Transform one point from map frame to vehicle frame. */
    geometry_msgs::msg::Point map_point_to_vehicle(
        const geometry_msgs::msg::Point& map_point) const;

    /**
     * Convert a planned path into one drive command and publish visualization.
     * Empty paths produce a stop command.
     */
    void follow_path(
        const nav_msgs::msg::Path& path,
        std::optional<double> speed_limit = std::nullopt);

    /** Slow along a blocked optimal reference after RRT* planning fails. */
    bool follow_blocked_reference(
        const std::vector<geometry_msgs::msg::Point>& optimal_reference);

    /** Publish a zero-speed command immediately. */
    void stop_vehicle();

    // ROS visualization helpers; they do not implement planning algorithms.
    void initialize_visualization();
    void visualize_goal(const geometry_msgs::msg::Point& goal);
    void visualize_tree(const rrt_star::Tree& tree);
    void clear_tree_visualization();
    void publish_path_marker(
        const std::vector<geometry_msgs::msg::Point>& points);
    void log_reference_transition(const reference_path::Decision& decision);

    std::unique_ptr<MarkerVisualizer> goal_visualizer_;
    std::unique_ptr<MarkerVisualizer> lookahead_visualizer_;
    std::unique_ptr<PointsVisualizer> global_waypoints_visualizer_;
    rclcpp::TimerBase::SharedPtr global_waypoints_timer_;
    visualization_msgs::msg::Marker tree_nodes_;
    visualization_msgs::msg::Marker tree_branches_;
};

#endif  // MOTION_PLANNING__RRT_HPP_
