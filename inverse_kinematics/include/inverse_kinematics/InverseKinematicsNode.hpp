#ifndef STARQ_INVERSE_KINEMATICS_NODE_HPP_
#define STARQ_INVERSE_KINEMATICS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <inverse_kinematics/types.hpp>
#include <inverse_kinematics/Leg.hpp>

namespace starq {

class InverseKinematicsNode : public rclcpp::Node {

public:

    InverseKinematicsNode() 
    : rclcpp::Node("inverse_kinematics_node") {

        cmd_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
            "/legs/positions", 10);

        pos_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "/motors/cmd", 10, 
            std::bind(&InverseKinematicsNode::position_callback, this, std::placeholders::_1));

    }
    
    void position_callback(const sensor_msgs::msg::PointCloud::SharedPtr pos_msg) const {
        trajectory_msgs::msg::JointTrajectoryPoint cmd_msg;

        for (Leg leg : this->legs_) {

            const int idx = leg.id();
            if (idx <= pos_msg->points.size()) {
                RCLCPP_WARN(this->get_logger(), "Input PointCloud is missing a position for %s", leg.name());
                continue;
            }

            const geometry_msgs::msg::Point32 point = pos_msg->points[idx];
            const LegPosition leg_position = {point.x, point.y, point.z};

            // Inverse Kinematics
            const MotorPosition motor_position = leg.model()->get_inverse(leg_position);

            // TODO: Insert into JointTrajectoryPoint

        }

        cmd_publisher_->publish(cmd_msg);
    }

private:


    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr pos_subscriber_;
    std::vector<Leg> legs_;

};

}

#endif