#!/usr/bin/env python3
"""
ROS Motor Control Node

This ROS node controls motors using GPIO and interfaces with velocity data from the "/vel_status" topic.
It adjusts motor actions based on received velocity commands.

Author: [Your Name]
Copyright (c) [Year]
License: [License]
Last Updated: [Date]
"""

from threading import Lock
import rospy
from gpiozero_plus import MotorController
from mach1_msgs.msg import VelStatus
from ros_utils.ros_utils import get_param


class MotorControl(MotorController):
    """
    MotorControl is a class that controls motors using GPIO and adjusts motor actions based on velocity commands.
    It extends the `MotorController` class from the `gpiozero_plus` library.

    Parameters:
        `left_motor (tuple)`: A tuple specifying the left motor pins (e.g., (1, 2)).

        `right_motor (tuple)`: A tuple specifying the right motor pins (e.g., (3, 4)).

        `curve_scale (float)`: A scaling factor for motor control.

    Attributes:
        `vel_sub (rospy.Subscriber)`: A ROS subscriber for velocity data.

        `linear_x (float)`: Linear velocity.

        `angular_z (float)`: Angular velocity.

        `motor_lock (threading.Lock)`: A thread lock for motor control.
    """

    def __init__(
        self, left_motor: tuple, right_motor: tuple, curve_scale: float
    ) -> None:
        """
        Initializes the MotorControl instance.
        """
        super().__init__(left_motor, right_motor, curve_scale)
        self._vel_sub = rospy.Subscriber("/vel_status", VelStatus, self.vel_callback)
        self._linear_x = 0.0
        self._angular_z = 0.0
        self._motor_lock = Lock()

        self.drive()

    def vel_callback(self, vel: VelStatus) -> None:
        """
        Callback function for velocity data.

        Args:
            `vel (VelStatus)`: Velocity status message.
        """
        with self._motor_lock:
            self._linear_x = vel.twist_msg.linear.x
            self._angular_z = vel.twist_msg.angular.z

    def drive(self) -> None:
        """
        Main motor control loop.
        """
        while not rospy.is_shutdown():
            with self._motor_lock:
                self.run(self._linear_x, self._angular_z)
                self._linear_x = self._angular_z = 0.0
            rospy.Rate(20).sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("motor_control_node")
        motor = MotorControl(
            tuple(get_param("/exp_board/motor/left_rear")),
            tuple(get_param("/exp_board/motor/right_rear")),
            get_param("/exp_board/motor/curve_scale"),
        )
        motor.disconnect()
    except rospy.ROSInitException:
        rospy.logerr("Failed to initialize node.")
    except Exception as ex:
        rospy.logerr(f"Exception caught: {str(ex)}")
