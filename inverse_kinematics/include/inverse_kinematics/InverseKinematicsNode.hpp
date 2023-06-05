#ifndef STARQ_INVERSE_KINEMATICS_NODE_HPP_
#define STARQ_INVERSE_KINEMATICS_NODE_HPP_

#include <map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <inverse_kinematics/types.hpp>
#include <inverse_kinematics/KinematicModel.hpp>

#include <yaml-cpp/yaml.h>

namespace starq {

class InverseKinematicsNode : public rclcpp::Node {

public:

    InverseKinematicsNode() 
    : rclcpp::Node("inverse_kinematics_node"), max_motor_id_(-1) {

        RCLCPP_INFO(this->get_logger(), "Starting Inverse Kinematics node.");

        this->declare_parameter<std::string>("config");
        std::string rel_config_path = this->get_parameter("config").as_string();
        config_path_ = ament_index_cpp::get_package_share_directory("inverse_kinematics") + "/config/" + rel_config_path;

        cmd_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
            "/motors/cmd", 10);

        cmd_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "/legs/cmd", 10, 
            std::bind(&InverseKinematicsNode::command_callback, this, std::placeholders::_1));

        // TODO : Add info publisher & subscriber
        // TODO : Info callback + forward kinematics

    }
    
    void command_callback(const sensor_msgs::msg::PointCloud::SharedPtr pos_msg) const {
        trajectory_msgs::msg::JointTrajectoryPoint cmd_msg;

        if (max_motor_id_ == -1) {
            RCLCPP_WARN(this->get_logger(), "Leg configuration not loaded. No motors found.");
            return;
        }

        cmd_msg.positions = std::vector<double>(max_motor_id_ + 1); // Input size

        for (Leg leg : this->legs_) {

            size_t idx = leg.id;
            if (idx >= pos_msg->points.size()) {
                RCLCPP_WARN(this->get_logger(), "Input PointCloud is missing a position for %s", leg.name.c_str());
                continue;
            }

            const geometry_msgs::msg::Point32 point = pos_msg->points[idx];
            const LegPosition leg_position = {point.x, point.y, point.z};

            // Inverse Kinematics
            const MotorPosition motor_position = leg.model->get_inverse(leg_position);

            if (motor_position.size() != leg.motor_ids.size()) {
                RCLCPP_ERROR(this->get_logger(), "Invalid inverse kinematic output for %s", leg.name.c_str());
                continue;
            }

            for (size_t idx = 0; idx < motor_position.size(); idx++) {
                cmd_msg.positions[leg.motor_ids[idx]] = motor_position[idx];
            }

        }

        cmd_publisher_->publish(cmd_msg);
    }

    void add_model(const std::string name, KinematicModel::Ptr model) {
        model_map_[name] = model;
    }

    void init() {

        RCLCPP_INFO(this->get_logger(), "Initializing Inverse Kinematics node from %s", config_path_.c_str());

        const YAML::Node config = YAML::LoadFile(config_path_);
        const YAML::Node legs_conf = config["legs"];

        for (auto it = legs_conf.begin(); it != legs_conf.end(); ++it) {
            const YAML::Node leg_conf = it->second;

            Leg leg;
            leg.name = it->first.as<std::string>();
            leg.id = leg_conf["id"].as<int>();
            leg.motor_ids = leg_conf["motors"].as<std::vector<int>>();
            std::string kinematic_type = leg_conf["kinematics"].as<std::string>();
            if (model_map_.find(kinematic_type) != model_map_.end()) {
                leg.model = model_map_[leg_conf["kinematics"].as<std::string>()];
            } else {
                RCLCPP_ERROR(this->get_logger(), "No kinematic model found under the name %s for leg %s", kinematic_type.c_str(), leg.name.c_str());
                return;
            }

            legs_.push_back(leg);

            RCLCPP_INFO(this->get_logger(), "Added leg %s", leg.name.c_str());
            
            for (int motor_id : leg.motor_ids)
                if (motor_id > max_motor_id_)
                    max_motor_id_ = motor_id;
        }

        RCLCPP_INFO(this->get_logger(), "Inverse Kinematics node initialized.");
    }

private:

    std::string config_path_;

    int max_motor_id_;
    std::vector<Leg> legs_;
    std::map<std::string, KinematicModel::Ptr> model_map_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr cmd_subscriber_;

};

}

#endif