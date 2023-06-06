#ifndef STARQ_GAIT_UTIL_HPP_
#define STARQ_GAIT_UTIL_HPP_

#include <map>
#include <iostream>
#include <fstream>
#include <sstream>

#include <gait_publisher/types.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace starq {

Trajectory load_trajectory(const std::string& file_name) {
    Trajectory trajectory;
    CSVData csv_data = read_csv(file_name);
    for (std::vector<float> points : csv_data) {
        sensor_msgs::msg::PointCloud cloud;
        if (points.size() % 3 != 0)
            continue;
        const int num_points = points.size() / 3;
        for (int idx = 0; idx < num_points; idx++) {
            geometry_msgs::msg::Point32 point;
            point.x = points[3*idx + 0];
            point.y = points[3*idx + 1];
            point.z = points[3*idx + 2];
            cloud.points.push_back(point);
        }
        trajectory.push_back(cloud);
    }
    return trajectory;
}

CSVData read_csv(const std::string& file_name) {
    const std::string abs_path = ament_index_cpp::get_package_share_directory("gait_publisher") + "/gaits/" + file_name;
    CSVData data;
    std::ifstream file(abs_path);
    if (!file.is_open())
        return data;
    std::string line;
    while (std::getline(file, line)) {
        std::vector<float> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            row.push_back(std::stof(cell));
        }
        data.push_back(row);
    }
    file.close();
    return data;
}

}

#endif