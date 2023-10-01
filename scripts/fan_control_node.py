#!/usr/bin/env python3
"""
ROS Fan Control Node

This ROS node serves as a wrapper for controlling the Fan interface on the Yahboom G1 Tank 4WD 
expansion board, allowing you to interact with the fan's functionality through ROS services.

Author: Julian A Rendon
Copyright (c) 2023 
License: MIT License
Last Updated: September 22, 2023
"""
import rospy

from gpiozero_plus import Fan
from mach1_msgs.srv import enable_fan, enable_fanResponse
from ros_utils import get_param


class FanController(Fan):
    """
    FanController is a class that controls a fan using GPIO and ROS services.

    It extends the `Fan` class from the `gpiozero_plus` library and provides
    a ROS service for enabling/disabling the fan.

    Parameters:
        `fan_pin (int)`: The GPIO pin connected to the fan.

        `initial_enabled (bool)`: The initial state of the fan (default is False, i.e., fan is off).
    """

    def __init__(self, fan_pin: int, initial_enabled: bool = False) -> None:
        """
        Initializes the FanController instance.
        """
        super().__init__(fan_pin, initial_enabled)
        self._fan_service = rospy.Service("enable_fan", enable_fan, self.fan_callback)
        self._current_fan_state = initial_enabled

    def fan_callback(self, state) -> enable_fanResponse:
        """
        Callback function for the fan service.

        Args:
            `state (enable_fan)`: The state to set for the fan.

        Returns:
            `enable_fanResponse`: A response message indicating the fan's state.
        """
        message = "Fan not toggled"

        if state.enabled != self._current_fan_state:
            if state.enabled:
                self.set_fan_state(state.enabled)
                message = "Fan toggled ON."
            else:
                self.set_fan_state(state.enabled)
                message = "Fan toggled OFF"
            rospy.loginfo(
                f"{self.fan_callback.__name__}()::FanEnabled:[{state.enabled}]"
            )
            self._current_fan_state = state.enabled
        else:
            rospy.logdebug(f"{self.fan_callback.__name__}()::{message}")
        return enable_fanResponse(message)


if __name__ == "__main__":
    try:
        rospy.init_node("fan_control_node")
        fan = FanController(
            get_param("/exp_board/fan/pin"),
            get_param("/exp_board/fan/enabled"),
        )
        rospy.spin()
        fan.disconnect()
    except rospy.ROSInitException:
        rospy.logerr("Failed to initialize node.")
    except Exception as ex:
        rospy.logerr(f"Exception caught::{str(ex)}")
