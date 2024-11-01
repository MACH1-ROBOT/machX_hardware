#pragma once
#include "camera.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

constexpr char CAMERA_CLIENT_NODE_NAME[] = "camera_client_node";

class CameraClientNode : public rclcpp::Node, protected camera_driver::Camera
{
  public:
    CameraClientNode();
    ~CameraClientNode();

  private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr luminositySub_;

    void InitializeClient();
    void LuminosityValueCB(std_msgs::msg::Float32::SharedPtr msg);
};