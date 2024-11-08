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

    compressedImageSub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "camera/image/compressed", 10,
        std::bind(&CameraClientNode::CompressedImageCB, this, std::placeholders::_1));
}

void CameraClientNode::CompressedImageCB(sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    // TODO: Use default duration parameter once camera driver version is updated.
    if (!DisplayFrame(frame, CAMERA_FPS_MS))
    {
        RCLCPP_WARN(this->get_logger(), "%s()::Failed to display frame.", __func__);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}