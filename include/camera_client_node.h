#pragma once
#include "camera.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

constexpr char CAMERA_CLIENT_NODE_NAME[] = "camera_client_node";
constexpr int CAMERA_FPS_MS = 33;

class CameraClientNode : public rclcpp::Node, protected camera_driver::Camera
{
  public:
    CameraClientNode();
    ~CameraClientNode();

  private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressedImageSub_;

    void InitializeClient();
    void CompressedImageCB(sensor_msgs::msg::CompressedImage::SharedPtr msg);
};