#ifndef STARQ_GAIT_TYPES_HPP_
#define STARQ_GAIT_TYPES_HPP_

#include <string>
#include <vector>
#include <memory>
#include <sensor_msgs/msg/point_cloud.hpp>

namespace starq {

using CSVData = std::vector<std::vector<float>>;
using Trajectory = std::vector<sensor_msgs::msg::PointCloud>;

struct Gait {
    std::string name;
    double publish_rate;
    Trajectory trajectory;
};

}

#endif