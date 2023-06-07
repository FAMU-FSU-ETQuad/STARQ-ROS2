#ifndef STARQ_INVERSE_KINEMATICS_NODE_HPP_
#define STARQ_INVERSE_KINEMATICS_NODE_HPP_

#include <map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <leg_kinematics/types.hpp>
#include <leg_kinematics/KinematicModel.hpp>

#include <yaml-cpp/yaml.h>

namespace starq {

class LegKinematicsNode : public rclcpp::Node {

public:

    LegKinematicsNode() 
    : rclcpp::Node("leg_kinematics_node"), max_motor_id_(-1) {

        RCLCPP_INFO(this->get_logger(), "Starting Leg Kinematics node.");

        this->declare_parameter<std::string>("config");
        std::string rel_config_path = this->get_parameter("config").as_string();
        config_path_ = ament_index_cpp::get_package_share_directory("leg_kinematics") + "/config/" + rel_config_path;

        cmd_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
            "/motors/cmd", 10);
        cmd_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "/legs/cmd", 10, 
            std::bind(&LegKinematicsNode::command_callback, this, std::placeholders::_1));

        info_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
            "/legs/info", 10);
        info_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
            "/motors/info", 10,
            std::bind(&LegKinematicsNode::info_callback, this, std::placeholders::_1));

    }
    

    void command_callback(const sensor_msgs::msg::PointCloud::SharedPtr cmd_msg_in) const {
        trajectory_msgs::msg::JointTrajectoryPoint cmd_msg_out;

        if (max_motor_id_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Leg configuration not loaded. No motors found.");
            return;
        }

        cmd_msg_out.positions = std::vector<double>(max_motor_id_ + 1); // Input size

        for (Leg leg : this->legs_) {

            size_t idx = leg.id;
            if (idx >= cmd_msg_in->points.size()) {
                RCLCPP_WARN(this->get_logger(), "Input PointCloud is missing a position for %s", leg.name.c_str());
                continue;
            }

            const geometry_msgs::msg::Point32 point = cmd_msg_in->points[idx];
            const LegPosition leg_position = {point.x, point.y, point.z};

            // Inverse Kinematics
            const MotorPosition motor_position = leg.model->get_inverse(leg_position);

            if (motor_position.size() != leg.motor_ids.size()) {
                RCLCPP_ERROR(this->get_logger(), "Invalid leg kinematic output for %s", leg.name.c_str());
                continue;
            }

            for (size_t idx = 0; idx < motor_position.size(); idx++) {
                cmd_msg_out.positions[leg.motor_ids[idx]] = motor_position[idx];
            }

        }

        cmd_publisher_->publish(cmd_msg_out);
        RCLCPP_INFO(this->get_logger(), "Published JointTrajectoryPoint.");
    }


    void info_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr info_msg_in) const {
        sensor_msgs::msg::PointCloud info_msg_out;

        if (max_motor_id_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Leg configuration not loaded. No motors found.");
            return;
        }

        info_msg_out.points = std::vector<geometry_msgs::msg::Point32>(legs_.size());
        for (Leg leg : legs_) {

            MotorPosition motor_position(leg.motor_ids.size());
            for (size_t idx = 0; idx < leg.motor_ids.size(); idx++) {
                int motor_id = leg.motor_ids[idx];
                if (motor_id >= int(info_msg_in->positions.size())) {
                    RCLCPP_ERROR(this->get_logger(), "Motor id %d is out of bounds", motor_id);
                    continue;
                }
                motor_position[idx] = info_msg_in->positions[motor_id];
            }

            // Forward Kinematics
            const LegPosition leg_position = leg.model->get_forward(motor_position);

            geometry_msgs::msg::Point32 leg_point;
            leg_point.x = leg_position[0];
            leg_point.y = leg_position[1];
            leg_point.z = leg_position[2];

            info_msg_out.points.push_back(leg_point);
        }

        info_publisher_->publish(info_msg_out);
    }


    void add_model(const std::string name, KinematicModel::Ptr model) {
        model_map_[name] = model;
    }


    void init() {

        RCLCPP_INFO(this->get_logger(), "Initializing Leg Kinematics node from %s", config_path_.c_str());

        const YAML::Node config = YAML::LoadFile(config_path_);
        const YAML::Node legs_conf = config["legs"];

        for (auto it = legs_conf.begin(); it != legs_conf.end(); ++it) {
            const YAML::Node leg_conf = it->second;

            Leg leg;
            leg.name = it->first.as<std::string>();
            leg.id = leg_conf["id"].as<int>();
            leg.motor_ids = leg_conf["motor_ids"].as<std::vector<int>>();
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

        RCLCPP_INFO(this->get_logger(), "Leg Kinematics node initialized.");
    }

private:

    std::string config_path_;

    int max_motor_id_;
    std::vector<Leg> legs_;
    std::map<std::string, KinematicModel::Ptr> model_map_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr cmd_subscriber_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr info_publisher_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr info_subscriber_;

};

}

#endif