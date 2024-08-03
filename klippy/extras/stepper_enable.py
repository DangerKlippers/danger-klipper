# Support for enable pins on stepper motor drivers
#
# Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

DISABLE_STALL_TIME = 0.100


# Tracking of shared stepper enable pins
class StepperEnablePin:
    def __init__(self, mcu_enable, enable_count):
        self.mcu_enable = mcu_enable
        self.enable_count = enable_count
        self.is_dedicated = True

    def set_enable(self, print_time):
        if not self.enable_count:
            self.mcu_enable.set_digital(print_time, 1)
        self.enable_count += 1

    def set_disable(self, print_time):
        self.enable_count -= 1
        if not self.enable_count:
            self.mcu_enable.set_digital(print_time, 0)


def setup_enable_pin(printer, pin):
    if pin is None:
        # No enable line (stepper always enabled)
        enable = StepperEnablePin(None, 9999)
        enable.is_dedicated = False
        return enable
    ppins = printer.lookup_object("pins")
    pin_params = ppins.lookup_pin(
        pin, can_invert=True, share_type="stepper_enable"
    )
    enable = pin_params.get("class")
    if enable is not None:
        # Shared enable line
        enable.is_dedicated = False
        return enable
    mcu_enable = pin_params["chip"].setup_pin("digital_out", pin_params)
    mcu_enable.setup_max_duration(0.0)
    enable = pin_params["class"] = StepperEnablePin(mcu_enable, 0)
    return enable


# Enable line tracking for each stepper motor
class EnableTracking:
    def __init__(self, stepper, enable):
        self.stepper = stepper
        self.enable = enable
        self.callbacks = []
        self.is_enabled = False
        self.stepper.add_active_callback(self.motor_enable)

    def register_state_callback(self, callback):
        self.callbacks.append(callback)

    def motor_enable(self, print_time):
        if not self.is_enabled:
            for cb in self.callbacks:
                cb(print_time, True)
            self.enable.set_enable(print_time)
            self.is_enabled = True

    def motor_disable(self, print_time):
        if self.is_enabled:
            # Enable stepper on future stepper movement
            for cb in self.callbacks:
                cb(print_time, False)
            self.enable.set_disable(print_time)
            self.is_enabled = False
            self.stepper.add_active_callback(self.motor_enable)

    def is_motor_enabled(self):
        return self.is_enabled

    def has_dedicated_enable(self):
        return self.enable.is_dedicated


# Global stepper enable line tracking
class PrinterStepperEnable:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.enable_lines = {}
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_request_restart
        )
        # Register M18/M84 commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("M18", self.cmd_M18)
        gcode.register_command("M84", self.cmd_M18)
        gcode.register_command(
            "SET_STEPPER_ENABLE",
            self.cmd_SET_STEPPER_ENABLE,
            desc=self.cmd_SET_STEPPER_ENABLE_help,
        )

    def register_stepper(self, config, mcu_stepper):
        name = mcu_stepper.get_name()
        enable = setup_enable_pin(self.printer, config.get("enable_pin", None))
        self.enable_lines[name] = EnableTracking(mcu_stepper, enable)

    def stepper_off(self, stepper_name, print_time, rail_name):
        el = self.enable_lines[stepper_name]
        el.motor_disable(print_time)
        if rail_name != "extruder":
            self.printer.send_event(
                "stepper_enable:disable_%s" % rail_name.lower(), print_time
            )

    def motor_off(self):
        self.axes_off()

    def axes_off(self, axes=None):
        if axes is None:
            axes = [0, 1, 2, 3]
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.dwell(DISABLE_STALL_TIME)
        print_time = toolhead.get_last_move_time()
        kin = toolhead.get_kinematics()
        if 3 in axes:
            if "extruder" in self.enable_lines:
                self.stepper_off("extruder", print_time, "extruder")
                i = 1
                extruder_name = f"extruder{i}"
                while extruder_name in self.enable_lines:
                    self.stepper_off(extruder_name, print_time, "extruder")
                    i += 1
                    extruder_name = f"extruder{i}"
        if hasattr(kin, "get_connected_rails"):
            for axis in axes:
                try:
                    rails = kin.get_connected_rails(axis)
                    for rail in rails:
                        steppers = rail.get_steppers()
                        rail_name = rail.mcu_stepper.get_name(True)
                        for stepper in steppers:
                            self.stepper_off(
                                stepper.get_name(), print_time, rail_name
                            )
                except IndexError:
                    continue
        else:
            if 0 in axes or 1 in axes or 2 in axes:
                for axis_name, el in self.enable_lines.items():
                    if not axis_name.startswith("extruder"):
                        el.motor_disable(print_time)
                self.printer.send_event("stepper_enable:motor_off", print_time)
        self.printer.send_event("stepper_enable:axes_off", print_time)
        toolhead.dwell(DISABLE_STALL_TIME)

    def motor_debug_enable(self, stepper, enable, notify=True):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.dwell(DISABLE_STALL_TIME)
        print_time = toolhead.get_last_move_time()
        kin = toolhead.get_kinematics()
        if not hasattr(kin, "get_rails"):
            notify = False
        el = self.enable_lines[stepper]
        if enable:
            el.motor_enable(print_time)
        else:
            el.motor_disable(print_time)
            if notify:
                for rail in kin.get_rails():
                    for stepper in rail.get_steppers():
                        if stepper.get_name() == stepper:
                            self.printer.send_event(
                                "stepper_enable:disable_%s"
                                % rail.mcu_stepper.get_name(True).lower(),
                                print_time,
                            )
        logging.info(
            "%s has been manually %s",
            stepper,
            "enabled" if enable else "disabled",
        )
        toolhead.dwell(DISABLE_STALL_TIME)

    def get_status(self, eventtime):
        steppers = {
            name: et.is_motor_enabled() for (name, et) in self.enable_lines.items()
        }
        return {"steppers": steppers}

    def _handle_request_restart(self, print_time):
        self.motor_off()

    def cmd_M18(self, gcmd):
        axes = []
        for pos, axis in enumerate("XYZE"):
            if gcmd.get(axis, None) is not None:
                axes.append(pos)
        if not axes:
            axes = [0, 1, 2, 3]
        # Turn off motors
        self.axes_off(axes)

    cmd_SET_STEPPER_ENABLE_help = "Enable/disable individual stepper by name"

    def cmd_SET_STEPPER_ENABLE(self, gcmd):
        stepper_name = gcmd.get("STEPPER", None)
        notify = gcmd.get_int("NOTIFY", 1)
        if stepper_name not in self.enable_lines:
            gcmd.respond_info(
                'SET_STEPPER_ENABLE: Invalid stepper "%s"' % (stepper_name,)
            )
            return
        stepper_enable = gcmd.get_int("ENABLE", 1)
        self.motor_debug_enable(stepper_name, stepper_enable, notify)

    def lookup_enable(self, name):
        if name not in self.enable_lines:
            raise self.printer.config_error("Unknown stepper '%s'" % (name,))
        return self.enable_lines[name]

    def get_steppers(self):
        return list(self.enable_lines.keys())


def load_config(config):
    return PrinterStepperEnable(config)
