#include <inverse_kinematics/InverseKinematicsNode.hpp>
#include <inverse_kinematics/models/FiveBar2D.hpp>

using namespace starq;

#define L1 3.0f
#define L2 3.0f

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    std::shared_ptr<InverseKinematicsNode> node = std::make_shared<InverseKinematicsNode>();
    node->add_model("fivebar-2d", std::make_shared<FiveBar2DModel>(L1, L2));
    node->init();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}