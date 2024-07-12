# Code for handling the kinematics of corexy robots
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import logging
from .corexy import CoreXYKinematics
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..toolhead import Move
    from ..extras.powercore import PowerCore

class PowerCoreXYKinematics(CoreXYKinematics):
    def __init__(self, toolhead, config):
        super().__init__(toolhead, config)
        self.powercore: PowerCore = config.get_printer().lookup_object(
            "powercore"
        )

    def scale_segmented_move(self, move):
        self.powercore.scale_move(move)

    


def load_kinematics(toolhead, config):
    return PowerCoreXYKinematics(toolhead, config)
