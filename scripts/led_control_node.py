#!/usr/bin/env python3

import rospy

from gpiozero_plus import RGBLEDController
from ros_utils.ros_utils import get_param


class RGBControl(RGBLEDController):
    def __init__(
        self,
        red_pin: int,
        green_pin: int,
        blue_pin: int,
    ) -> None:
        super().__init__(red_pin, green_pin, blue_pin)

        # FIXME: Add ROBOT STATE instead
        self._current_led_mode = "DEFAULT"
        self._new_led_mode = "TEST"

        self.run()

    def run(self) -> None:
        while not rospy.is_shutdown():
            if self._new_led_mode != self._current_led_mode:
                rospy.logdebug(f"{self.run.__name__}()::Mode - [{self._new_led_mode}]")
                # FIXME: This needs to change colors according to the ROBOT STATE
                self.change_color(color=(255, 255, 255), brightness=0.25)
            self._current_led_mode = self._new_led_mode
            rospy.Rate(1).sleep()


if __name__ == "__main__":
    try:
        rgb_led = RGBControl(
            get_param("/exp_board/rgb_led/red"),
            get_param("/exp_board/rgb_led/green"),
            get_param("/exp_board/rgb_led/blue"),
        )
        rgb_led.disconnect()

    except rospy.ROSInitException:
        rospy.logerr("Failed to initialize node.")
