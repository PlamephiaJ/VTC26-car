cmake_minimum_required(VERSION 3.8)
project(motion_planning)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
foreach(PACKAGE
  ament_cmake
  rclcpp
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

# Algorithm modules are kept in a ROS-node-independent static library so they
# can be tested and optimized without constructing an rclcpp::Node.
add_library(motion_planning_core STATIC
  src/dynamic_obstacle_map.cpp
  src/occupancy_grid.cpp
  src/optimal_trajectory.cpp
  src/path_tracking.cpp
  src/reference_path_manager.cpp
  src/rrt_star_planner.cpp
  src/rrt_tree.cpp)
target_include_directories(motion_planning_core PUBLIC include)
ament_target_dependencies(motion_planning_core
  geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs)

# The node target contains ROS2 orchestration, file loading, and visualization.
add_executable(rrt_node_sim
  src/main.cpp
  src/RRT.cpp
  src/FileHandler.cpp
  src/Visualization.cpp)
target_link_libraries(rrt_node_sim motion_planning_core)

ament_target_dependencies(rrt_node_sim
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros tf2_geometry_msgs tf2
)

# Install Cpp executables
install(TARGETS
  rrt_node_sim
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_occupancy_grid
    test/test_occupancy_grid.cpp)
  target_include_directories(test_occupancy_grid PRIVATE include)
  target_link_libraries(test_occupancy_grid motion_planning_core)
  ament_target_dependencies(test_occupancy_grid nav_msgs)

  ament_add_gtest(test_path_tracking
    test/test_path_tracking.cpp)
  target_include_directories(test_path_tracking PRIVATE include)
  target_link_libraries(test_path_tracking motion_planning_core)
  ament_target_dependencies(test_path_tracking geometry_msgs nav_msgs)

  ament_add_gtest(test_planning_modules
    test/test_planning_modules.cpp)
  target_include_directories(test_planning_modules PRIVATE include)
  target_link_libraries(test_planning_modules motion_planning_core)
  ament_target_dependencies(test_planning_modules
    geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
