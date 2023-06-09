#include <leg_kinematics/LegKinematicsNode.hpp>
#include <leg_kinematics/models/FiveBar2D.hpp>

using namespace starq;

#define L1 0.15f
#define L2 0.165f

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    std::shared_ptr<LegKinematicsNode> node = std::make_shared<LegKinematicsNode>();
    node->add_model("fivebar-2d", std::make_shared<FiveBar2DModel>(L1, L2));
    node->init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}