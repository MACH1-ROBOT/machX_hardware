#!/usr/bin/env python3
"""
ROS Ultrasonic Sensor Node

This ROS node interfaces with an ultrasonic sensor connected to the GPIO pins of the Raspberry Pi.
It publishes ultrasonic sensor data and handles distance measurements.

Author: Julian A Rendon
Copyright (c) 2023 
License: MIT License
Last Updated: September 25, 2023
"""
import rospy

from gpiozero_plus import Ultrasonic
from mach1_msgs.msg import Collision
from ros_utils.ros_utils import get_param


class UltraSonicSensor(Ultrasonic):
    """
    UltraSonicSensor is a class that controls an ultrasonic sensor using GPIO and publishes data through ROS.

    It extends the `Ultrasonic` class from the `gpiozero_plus` library and publishes ultrasonic sensor data
    to the `/ultrasonic_status` topic.

    Parameters:
        `echo_pin (int)`: The GPIO pin connected to the echo pin of the ultrasonic sensor.

        `trig_pin (int)`: The GPIO pin connected to the trig pin of the ultrasonic sensor.

        `max_distance (float)`: The maximum measurable distance of the ultrasonic sensor in meters.

        `threshold (float)`: The threshold distance for collision detection in meters (e.g., obstacle detection).

    Attributes:

        `ultrasonic_pub (rospy.Publisher)`: A ROS publisher for ultrasonic sensor data.
        `sensor_msg (mach1_msgs.msg.Collision)`: A message object for storing sensor data.
    """

    def __init__(
        self, echo_pin: int, trig_pin: int, max_distance: float, threshold: float
    ) -> None:
        """
        Initializes the UltraSonicSensor instance.
        """
        super().__init__(echo_pin, trig_pin, max_distance, threshold)
        self._ultrasonic_pub = rospy.Publisher(
            "/ultrasonic_status", Collision, queue_size=1
        )
        self._sensor_msg = Collision()
        self._sensor_msg.threshold = threshold

        self.measure()

    def measure(self) -> None:
        """
        Measures ultrasonic sensor data and publishes it
        """
        while not rospy.is_shutdown():
            self._sensor_msg.distance = self.get_distance()
            if self._sensor_msg.distance is not None:
                rospy.loginfo(
                    f"{self.measure.__name__}()::Distance: [{self._sensor_msg.distance:.3f}] meters"
                )
                self._ultrasonic_pub.publish(self._sensor_msg)
            rospy.Rate(60).sleep()
            rospy.spin()


if __name__ == "__main__":
    try:
        rospy.init_node("ultrasonic_sensor_node")
        sensor = UltraSonicSensor(
            get_param("/exp_board/ultrasonic/echo"),
            get_param("/exp_board/ultrasonic/trig"),
            get_param("/exp_board/ultrasonic/max_dist"),
            get_param("/exp_board/ultrasonic/threshold"),
        )
        sensor.disconnect()
    except rospy.ROSInitException:
        rospy.logerr("Failed to initialize node.")
    except Exception as ex:
        rospy.logerr(f"Exception caught::{str(ex)}")
