/*
 * @Author: Yuhao Chen
 * @Date: 2024-06-23 09:53:41
 * @LastEditors: Yuhao Chen
 * @LastEditTime: 2024-06-25 11:21:35
 * @Description: RRT node main function.
 */

#include "../include/motion_planning/RRT.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRT>());
    rclcpp::shutdown();
    return 0;
}