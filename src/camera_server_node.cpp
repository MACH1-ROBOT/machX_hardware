#include "camera_server_node.h"
#include "camera_utils.h"

CameraServerNode::CameraServerNode() : Node(CAMERA_SERVER_NODE_NAME), Camera()
{
    InitializeServer();
}

CameraServerNode::CameraServerNode(std::string name, std::string type, int index, int fps)
    : Node(CAMERA_SERVER_NODE_NAME), Camera(name, type, index, fps)
{
    InitializeServer();
}

CameraServerNode::~CameraServerNode()
{
    RCLCPP_INFO(this->get_logger(), "%s()::Server stopped.", __func__);
}

void CameraServerNode::InitializeServer()
{
    RCLCPP_INFO(this->get_logger(), "%s()::Setting up camera server.", __func__);
    luminosityPub_ = this->create_publisher<std_msgs::msg::Float32>("luminosity_value", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(LUMINOSITY_PUB_TIMER_MS),
                                     std::bind(&CameraServerNode::Start, this));
}

void CameraServerNode::Start()
{
    RCLCPP_INFO_ONCE(this->get_logger(), "%s()::Sever started.", __func__);

    auto frame = CaptureFrame();
    if (!frame)
    {
        RCLCPP_ERROR(this->get_logger(), "%s()::Failed to capture frame.", __func__);
        Stop();
        return;
    }

    auto luminosity = camera_utils::GetLuminosity(*frame);
    if (luminosity)
    {
        auto luminosityMsg = std_msgs::msg::Float32();
        luminosityMsg.data = luminosity.value();
        luminosityPub_->publish(luminosityMsg);
        RCLCPP_DEBUG(this->get_logger(), "%s()::Luminosity value - [%.2f]", __func__,
                     luminosityMsg.data);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "%s()::Failed to get luminosity from the frame.", __func__);
    }
}

void CameraServerNode::Stop()
{
    RCLCPP_INFO_ONCE(this->get_logger(), "%s()::Stopping server.", __func__);
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraServerNode>("RPi4", "USB", 0, 30);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}