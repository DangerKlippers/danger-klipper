# Code for handling the kinematics of corexy robots
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from .cartesian import CartKinematics
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..extras.powercore import PowerCore


class PowerCartesianKinematics(CartKinematics):
    def __init__(self, toolhead, config):
        super().__init__(toolhead, config)
        self.powercore: PowerCore = config.get_printer().lookup_object(
            "powercore"
        )

    def check_move(self, move):
        super().check_move(move)
        self.powercore.check_move(move)


def load_kinematics(toolhead, config):
    return CartKinematics(toolhead, config)
