#include <gait_publisher/GaitPublisherNode.hpp>

using namespace starq;

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<GaitPublisherNode> node = std::make_shared<GaitPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}