#ifndef STARQ_GAIT_PUBLISHER_NODE_HPP_
#define STARQ_GAIT_PUBLISHER_NODE_HPP_

#include <map>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include <gait_publisher/types.hpp>
#include <gait_publisher/util.hpp>

namespace starq {

class GaitPublisherNode : public rclcpp::Node {

public:

    GaitPublisherNode()
    : rclcpp::Node("gait_publisher") {

        RCLCPP_INFO(this->get_logger(), "Starting Gait Publisher node.");

        this->declare_parameter<std::string>("config");
        std::string rel_config_path = this->get_parameter("config").as_string();
        config_path_ = ament_index_cpp::get_package_share_directory("leg_kinematics") + "/config/" + rel_config_path;

        gait_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
            "/legs/cmd", 10);
        gait_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/gait", 10, 
            std::bind(&GaitPublisherNode::gait_callback, this, std::placeholders::_1));

        load_config();
    }

    void gait_callback(const std_msgs::msg::String::SharedPtr gait_msg) const {
        std::string gait_name = gait_msg->data;

        if (gait_map_.find(gait_name) == gait_map_.end()) {
            RCLCPP_ERROR(this->get_logger(), "Could not find the gait %s", gait_name.c_str());
            return;
        }

        Gait gait = gait_map_.at(gait_name);

        using clock = std::chrono::steady_clock;
        std::chrono::milliseconds loop_time(1000 / int(gait.publish_rate));

        for (sensor_msgs::msg::PointCloud cloud : gait.trajectory) {
            auto cstart = clock::now();

            // Publish PointCloud
            gait_publisher_->publish(cloud);

            auto cend = clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cend - cstart);
            if (elapsed < loop_time)
                std::this_thread::sleep_for(loop_time - elapsed);
        }
    }

    void load_config() {

        RCLCPP_INFO(this->get_logger(), "Initializing Gait publisher node from %s", config_path_.c_str());

        const YAML::Node config = YAML::LoadFile(config_path_);
        const YAML::Node gaits_conf = config["gaits"];

        for (auto it = gaits_conf.begin(); it != gaits_conf.end(); ++it) {
            std::string gait_name = it->first.as<std::string>();
            const YAML::Node gait_conf = it->second;

            Gait gait;
            gait.publish_rate = gait_conf["publish_rate"].as<double>();
            gait.trajectory = load_trajectory(gait_conf["file_name"].as<std::string>());

            if (gait.trajectory.size() == 0) {
                RCLCPP_ERROR(this->get_logger(), "Error loading gait %s", gait_name.c_str());
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

};

}

#endif