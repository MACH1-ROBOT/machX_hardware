#include "camera_server_node.h"

CameraServerNode::CameraServerNode() : Node("camera_server_node"), Camera()
{
    luminosityPub_ = this->create_publisher<std_msgs::msg::Float32>("luminosity_value", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(LUMINOSITY_PUB_TIMER_MS),
                                     std::bind(&CameraServerNode::publishLuminosityValue, this));
    RCLCPP_INFO(this->get_logger(), "%s::Setting up camera server again.", __func__);
}

void CameraServerNode::publishLuminosityValue()
{
    auto luminosityMsg = std_msgs::msg::Float32();
    luminosityMsg.data = luminosityVal;
    luminosityPub_->publish(luminosityMsg);
    luminosityVal++;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraServerNode>();
    rclcpp::spin(node);
    return 0;
}