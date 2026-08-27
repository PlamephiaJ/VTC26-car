/*
 * @Author: Yuhao Chen
 * @Date: 2024-06-23 19:10:46
 * @LastEditors: Yuhao Chen
 * @LastEditTime: 2024-07-01 11:49:20
 * @Description: File handler header file.
 */

#ifndef FILEHANDLER_HPP
#define FILEHANDLER_HPP

#include "geometry_msgs/msg/point.hpp"

#include <fstream>
#include <sstream>
#include <cmath>
#include <string>
#include <vector>

struct PosAndSpeed
{
    geometry_msgs::msg::Point position;
    double speed;
};

/** Base class that stores the source file path for concrete readers. */
class FileHandler {
protected:
    std::string file_path;
public:
    /** Input: path used by subsequent read operations. No file is opened yet. */
    FileHandler(const std::string& file_path);
    virtual ~FileHandler() {}

    /** Return the configured path by const reference; no state is modified. */
    const std::string& get_file_path() const;
    // virtual bool read(const std::string& filename) = 0;
    // virtual bool write(const std::string& filename, const std::string& data) = 0;
};

/** Reader for the waypoint CSV formats accepted by this package. */
class CSVHandler : public FileHandler {
public:
    /** Input: waypoint CSV path. Parsing is deferred to a read method. */
    CSVHandler(const std::string& file_path);
    virtual ~CSVHandler() {}
    // virtual bool read(const std::string& filename) override;
    // virtual bool write(const std::string& filename, const std::string& data) override { return false; }
    /**
     * Return waypoint positions in file order.
     * Throws an exception when the file cannot be opened or parsed.
     */
    std::vector<geometry_msgs::msg::Point> read_waypoint_list_from_csv();

    /**
     * Return waypoint position/speed records in file order.
     * Throws an exception when the file cannot be opened or parsed.
     */
    std::vector<PosAndSpeed> read_waypoint_and_speed_list_from_csv();
};

#endif
