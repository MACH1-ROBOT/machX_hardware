#include <iostream>
#include <rclcpp/rclcpp.hpp>
// #include "camera.h"

class CameraServerNode : public rclcpp::Node
{
  public:
    CameraServerNode() : Node("camera_server_node")
    {
        RCLCPP_INFO(this->get_logger(), "Hello, ROS 2");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraServerNode>();
    rclcpp::spin(node);
    return 0;
}