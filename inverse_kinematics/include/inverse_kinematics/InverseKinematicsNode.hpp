#ifndef STARQ_INVERSE_KINEMATICS_NODE_HPP_
#define STARQ_INVERSE_KINEMATICS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace starq {

template <typename KinematicModelType>
class InverseKinematicsNode : public rclcpp::Node {

public:

    InverseKinematicsNode() 
    : rclcpp::Node("inverse_kinematics_node"), model_(std::make_shared<KinematicModelType>()) {}

private:

    std::shared_ptr<KinematicModelType> model_;

};

}

#endif