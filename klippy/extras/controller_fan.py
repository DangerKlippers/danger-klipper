# Support a fan for cooling the MCU whenever a stepper or heater is on
#
# Copyright (C) 2019  Nils Friedchen <nils.friedchen@googlemail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import fan

PIN_MIN_TIME = 0.100


class ControllerFan:
    def __init__(self, config, defined_fan=None):
        self.name = config.get_name().split()[1]
        self.printer = config.get_printer()
        self.printer.register_event_handler(
            "klippy:connect", self.handle_connect
        )
        if defined_fan is None:
            self.printer.register_event_handler(
                "klippy:ready", self.handle_ready
            )
            self.fan = fan.Fan(config)
        else:
            self.fan = defined_fan
        self.stepper_names = config.getlist("stepper", None)
        self.stepper_enable = self.printer.load_object(config, "stepper_enable")
        self.printer.load_object(config, "heaters")
        self.heaters = []
        self.fan_speed = config.getfloat(
            "fan_speed", default=1.0, minval=0.0, maxval=1.0
        )
        self.idle_speed = config.getfloat(
            "idle_speed", default=self.fan_speed, minval=0.0, maxval=1.0
        )
        self.idle_timeout = config.getint("idle_timeout", default=30, minval=0)
        self.heater_names = config.getlist("heater", ("extruder",))
        self.last_on = self.idle_timeout
        self.last_speed = 0.0

    def handle_connect(self):
        # Heater lookup
        pheaters = self.printer.lookup_object("heaters")
        self.heaters = [pheaters.lookup_heater(n) for n in self.heater_names]
        # Stepper lookup
        all_steppers = self.stepper_enable.get_steppers()
        if self.stepper_names is None:
            self.stepper_names = all_steppers
            return
        if not all(x in all_steppers for x in self.stepper_names):
            raise self.printer.config_error(
                "One or more of these steppers are unknown: "
                "%s (valid steppers are: %s)"
                % (self.stepper_names, ", ".join(all_steppers))
            )

    def handle_ready(self):
        reactor = self.printer.get_reactor()
        reactor.register_timer(
            self.callback, reactor.monotonic() + PIN_MIN_TIME
        )

    def get_status(self, eventtime):
        return self.fan.get_status(eventtime)

    def get_speed(self, eventtime):
        speed = self.idle_speed
        active = False
        for name in self.stepper_names:
            active |= self.stepper_enable.lookup_enable(name).is_motor_enabled()
        for heater in self.heaters:
            _, target_temp = heater.get_temp(eventtime)
            if target_temp:
                active = True
        if active:
            self.last_on = 0
            speed = self.fan_speed
        elif self.idle_timeout != -1:
            if self.last_on >= self.idle_timeout:
                speed = 0.0
            else:
                self.last_on += 1
        return speed

    def callback(self, eventtime):
        speed = self.get_speed(eventtime)
        if speed != self.last_speed:
            self.last_speed = speed
            curtime = self.printer.get_reactor().monotonic()
            print_time = self.fan.get_mcu().estimated_print_time(curtime)
            self.fan.set_speed(print_time + PIN_MIN_TIME, speed)
        return eventtime + 1.0


def load_config_prefix(config):
    return ControllerFan(config)
