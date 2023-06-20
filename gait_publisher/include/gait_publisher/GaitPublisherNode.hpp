#ifndef STARQ_GAIT_PUBLISHER_NODE_HPP_
#define STARQ_GAIT_PUBLISHER_NODE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <yaml-cpp/yaml.h>

namespace starq {

using CSVData = std::vector<std::vector<float>>;
using Trajectory = std::vector<sensor_msgs::msg::PointCloud>;

struct Gait {
    std::string name;
    double publish_rate;
    Trajectory trajectory;
};

class GaitPublisherNode : public rclcpp::Node {

public:

    GaitPublisherNode()
    : rclcpp::Node("gait_publisher") {

        RCLCPP_INFO(this->get_logger(), "Starting Gait Publisher node.");

        this->declare_parameter<std::string>("config");
        std::string rel_config_path = this->get_parameter("config").as_string();
        config_path_ = "/home/pi/ros2_ws/src/boom_packages/gait_publisher/config/" + rel_config_path;

        gait_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
            "/legs/cmd", 1000);
        gait_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/gait", 10, 
            std::bind(&GaitPublisherNode::gait_callback, this, std::placeholders::_1));

        load_config();
    }

    void gait_callback(const std_msgs::msg::String::SharedPtr gait_msg) const {
        std::string gait_name = gait_msg->data;

        if (gait_map_.find(gait_name) == gait_map_.end()) {
            RCLCPP_ERROR(this->get_logger(), "Could not find the gait '%s'.", gait_name.c_str());
            return;
        }
        
        const Gait& gait = gait_map_.at(gait_name);

        using clock = std::chrono::steady_clock;
        std::chrono::microseconds loop_time(int(1E6 / gait.publish_rate));

        for (sensor_msgs::msg::PointCloud cloud : gait.trajectory) {
            auto cstart = clock::now();

            // Publish PointCloud
            gait_publisher_->publish(cloud);
            //RCLCPP_INFO(this->get_logger(), "Published PointCloud.");

            auto cend = clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(cend - cstart);
            if (elapsed < loop_time)
                std::this_thread::sleep_for(loop_time - elapsed);
        }
    }

    void load_config() {

        RCLCPP_INFO(this->get_logger(), "Initializing Gait publisher node from %s", config_path_.c_str());

        const YAML::Node config = YAML::LoadFile(config_path_);

        for (auto it = config.begin(); it != config.end(); ++it) {
            std::string gait_name = it->first.as<std::string>();
            const YAML::Node gait_conf = it->second;

            Gait gait;
            gait.publish_rate = gait_conf["publish_rate"].as<double>();
            gait.trajectory = load_trajectory_(gait_conf["file_name"].as<std::string>());

            if (gait.trajectory.size() == 0) {
                RCLCPP_ERROR(this->get_logger(), "Error loading the gait '%s'.", gait_name.c_str());
                continue;
            }

            gait_map_[gait_name] = gait;

            RCLCPP_INFO(this->get_logger(), "Added gait %s", gait_name.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Gait publisher node initialized.");
    }

private:

    std::string config_path_;
    std::map<std::string, Gait> gait_map_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr gait_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gait_subscriber_;

    CSVData read_csv_(const std::string& file_name) {
        std::cout << "READ" << std::endl;
        const std::string abs_path = ament_index_cpp::get_package_share_directory("gait_publisher") + "/gaits/" + file_name;
        CSVData data;
        std::ifstream file(abs_path);
        if (!file.is_open()) {
            RCLCPP_INFO(this->get_logger(), "Could not open CSV file '%s'", abs_path.c_str());
            return data;
        }
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

    Trajectory load_trajectory_(const std::string& file_name) {
        Trajectory trajectory;
        CSVData csv_data = read_csv_(file_name);
        for (std::vector<float> points : csv_data) {
            sensor_msgs::msg::PointCloud cloud;
            if (points.size() % 3 != 0) {
                RCLCPP_INFO(this->get_logger(), "CSV '%s' has an incorrect number of columns (%lu is not a multiple of 3)", file_name.c_str(), points.size());
                continue;
            }
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

};

}

#endif