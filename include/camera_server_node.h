#pragma once
#include "camera.h"
#include "camera_utils.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

constexpr int LUMINOSITY_PUB_TIMER_MS = 1000;

class CameraServerNode : public rclcpp::Node, protected camera_driver::Camera
{
  public:
    CameraServerNode();

  private:
    float luminosityVal = 0;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr luminosityPub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publishLuminosityValue();
};