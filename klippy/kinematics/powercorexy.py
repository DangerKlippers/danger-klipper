# Code for handling the kinematics of corexy robots
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from .corexy import CoreXYKinematics
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..extras.powercore import PowerCore


class PowerCoreXYKinematics(CoreXYKinematics):
    def __init__(self, toolhead, config):
        super().__init__(toolhead, config)
        self.powercore: PowerCore = config.get_printer().lookup_object(
            "powercore"
        )

    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (
            xpos < limits[0][0]
            or xpos > limits[0][1]
            or ypos < limits[1][0]
            or ypos > limits[1][1]
        ):
            self._check_endstops(move)
        if move.axes_d[2]:
            self._check_endstops(move)
        
        self.powercore.check_move(move)


def load_kinematics(toolhead, config):
    return PowerCoreXYKinematics(toolhead, config)
