/*
 * @Author: Yuhao Chen
 * @Date: 2024-06-23 19:12:35
 * @LastEditors: Yuhao Chen
 * @LastEditTime: 2024-07-01 11:50:00
 * @Description: File handler source file.
 */

#include "../include/motion_planning/FileHandler.hpp"

FileHandler::FileHandler(const std::string& file_path) : file_path(file_path) {}

const std::string& FileHandler::get_file_path() const
{
    return file_path;
}

CSVHandler::CSVHandler(const std::string& file_path) : FileHandler(file_path) {}

std::vector<geometry_msgs::msg::Point> CSVHandler::read_waypoint_list_from_csv()
{
    std::ifstream file;
    std::vector<geometry_msgs::msg::Point> waypoints_mapframe;
    file.open(file_path);
    if (!file.is_open())
    {
        throw std::invalid_argument("Couldn't open the file.");
    }

    std::string line;
    double x, y;
    // pass title row.
    std::getline(file, line);
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string token;
        if (std::getline(ss, token, ','))
        {
            x = std::stod(token);
        }
        if (std::getline(ss, token, ','))
        {
            y = std::stod(token);
        }
        geometry_msgs::msg::Point point_mapframe;
        point_mapframe.x = x;
        point_mapframe.y = y;
        point_mapframe.z = 0.0;
        waypoints_mapframe.emplace_back(point_mapframe);
    }
    file.close();

    return waypoints_mapframe;
}

std::vector<PosAndSpeed> CSVHandler::read_waypoint_and_speed_list_from_csv()
{
    std::ifstream file;
    std::vector<PosAndSpeed> position_and_speed_list;
    file.open(file_path);
    if (!file.is_open())
    {
        throw std::invalid_argument("Couldn't open the file.");
    }

    std::string line;
    double x, y;
    double speed = std::nan("");
    // pass title row.
    std::getline(file, line);
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string token;
        if (std::getline(ss, token, ','))
        {
            x = std::stod(token);
        }

        if (std::getline(ss, token, ','))
        {
            y = std::stod(token);
        }

        if (std::getline(ss, token, ','))
        {
            speed = std::stod(token);
        }
        else
        {
            throw std::runtime_error("Couldn't read speed information.");
        }

        PosAndSpeed pas;
        pas.position.x = x;
        pas.position.y = y;
        pas.position.z = 0.0;
        pas.speed = speed;

        position_and_speed_list.emplace_back(pas);
    }
    file.close();

    return position_and_speed_list;
}