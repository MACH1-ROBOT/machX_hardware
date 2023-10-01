#!/usr/bin/env python3

from enum import Enum


class RobotState(Enum):
    ERROR = (0, "Error")
    LOW_BATT = (1, "Low Battery")
    STARTUP = (2, "Start up")
    NO_WIFI = (3, "No WiFi")
    TELEOP = (4, "Teleoperation")
    UPDATE = (5, "System Update")
    IDLE = (6, "Idle")
    TEST = (7, "Testing")

    def __init__(self, priority, description):
        self._value_ = priority
        self.description = description
