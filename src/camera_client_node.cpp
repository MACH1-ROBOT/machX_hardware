#include "camera_client_node.h"

CameraClientNode::CameraClientNode() : Node(CAMERA_CLIENT_NODE_NAME), Camera()
{
    InitializeClient();
}

CameraClientNode::~CameraClientNode()
{
    RCLCPP_INFO(this->get_logger(), "%s()::Client stopped.", __func__);
}

void CameraClientNode::InitializeClient()
{
    RCLCPP_INFO(this->get_logger(), "%s()::Setting up camera client.", __func__);
    luminositySub_ = this->create_subscription<std_msgs::msg::Float32>(
        "luminosity_value", 10,
        std::bind(&CameraClientNode::LuminosityValueCB, this, std::placeholders::_1));
}

void CameraClientNode::LuminosityValueCB(const std_msgs::msg::Float32::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "%s::Luminosity value - [%.2f]", __func__, msg->data);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}