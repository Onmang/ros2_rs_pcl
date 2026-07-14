#include "rclcpp/rclcpp.hpp"
#include "ros2_rs_pcl/segplane/rspcl_segplane_component.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RspclSegPlaneComponent>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}