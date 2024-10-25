#pragma once
#include "camera.h"
#include "camera_utils.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

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