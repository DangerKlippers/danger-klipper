# TMC4671 servo driver support
#
# Copyright (C) 2024       Andrew McGregor <andrewmcgr@gmail.com>
#
# Based heavily on Klipper TMC stepper drivers which are:
#
# Copyright (C) 2018-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, collections
import math
from time import monotonic_ns
from enum import IntEnum
from statistics import median_low
from .. import bus, tmc
from .helpers import FieldHelper

# The 4671 has a 25 MHz external clock
TMC_FREQUENCY = 25000000.0
# However there is a 100 MHz internal clock, hence 10 ns units in places

# Some magic numbers for the driver


class MotionMode(IntEnum):
    stopped_mode = 0
    torque_mode = 1
    velocity_mode = 2
    position_mode = 3
    uq_ud_ext_mode = 8



def format_phi(val):
    phi = val * 360.0 / 65536.0
    if phi < 0.0:
        phi += 360
    return "%.3f" % (phi)


def format_q4_12(val):
    return "%.4f" % (val * 2**-12)


def to_q4_12(val):
    return round(val * 2**12) & 0xFFFF


def from_q4_12(val):
    return val * 2**-12


def format_q0_15(val):
    return "%.7f" % (val * 2**-15)


def from_q8_8(val):
    return val * 2**-8


def format_q8_8(val):
    return "%.3f" % (from_q8_8(val))


def to_q8_8(val):
    return round(val * 2**8) & 0xFFFF


def format_q3_29(val):
    return "%.9f" % (val * 2**-29)


FieldFormatters = {
    "CONFIG_BIQUAD_X_A_1": format_q3_29,
    "CONFIG_BIQUAD_X_A_2": format_q3_29,
    "CONFIG_BIQUAD_X_B_0": format_q3_29,
    "CONFIG_BIQUAD_X_B_1": format_q3_29,
    "CONFIG_BIQUAD_X_B_2": format_q3_29,
    "CONFIG_BIQUAD_V_A_1": format_q3_29,
    "CONFIG_BIQUAD_V_A_2": format_q3_29,
    "CONFIG_BIQUAD_V_B_0": format_q3_29,
    "CONFIG_BIQUAD_V_B_1": format_q3_29,
    "CONFIG_BIQUAD_V_B_2": format_q3_29,
    "CONFIG_BIQUAD_T_A_1": format_q3_29,
    "CONFIG_BIQUAD_T_A_2": format_q3_29,
    "CONFIG_BIQUAD_T_B_0": format_q3_29,
    "CONFIG_BIQUAD_T_B_1": format_q3_29,
    "CONFIG_BIQUAD_T_B_2": format_q3_29,
    "CONFIG_BIQUAD_F_A_1": format_q3_29,
    "CONFIG_BIQUAD_F_A_2": format_q3_29,
    "CONFIG_BIQUAD_F_B_0": format_q3_29,
    "CONFIG_BIQUAD_F_B_1": format_q3_29,
    "CONFIG_BIQUAD_F_B_2": format_q3_29,
    "HALL_POSITION_000": format_phi,
    "HALL_POSITION_120": format_phi,
    "HALL_POSITION_240": format_phi,
    "HALL_POSITION_060": format_phi,
    "HALL_POSITION_180": format_phi,
    "HALL_POSITION_300": format_phi,
    "HALL_PHI_E": format_phi,
    "HALL_PHI_E_INTERPOLATED": format_phi,
    "HALL_PHI_E_PHI_M_OFFSET": format_phi,
    "HALL_PHI_M": format_phi,
    "PHI_E": format_phi,
    "HALL_PHI_E_OFFSET": format_phi,
    "HALL_PHI_M_OFFSET": format_phi,
    "ABN_DECODER_PHI_M": format_phi,
    "ABN_DECODER_PHI_E": format_phi,
    "ABN_DECODER_PHI_E_OFFSET": format_phi,
    "ABN_DECODER_PHI_M_OFFSET": format_phi,
    "PID_FLUX_P": format_q4_12,
    "PID_FLUX_I": format_q4_12,
    "PID_TORQUE_P": format_q4_12,
    "PID_TORQUE_I": format_q4_12,
    "PID_VELOCITY_P": format_q4_12,
    "PID_VELOCITY_I": format_q4_12,
    "PID_POSITION_P": format_q4_12,
    "PID_POSITION_I": format_q4_12,
}

DumpGroups = {
    "Default": ["CHIPINFO_SI_TYPE", "CHIPINFO_SI_VERSION", "STATUS_FLAGS", "PHI_E"],
    "HALL": [
        "HALL_MODE",
        "HALL_POSITION_060_000",
        "HALL_POSITION_180_120",
        "HALL_POSITION_300_240",
        "HALL_PHI_E_INTERPOLATED_PHI_E",
        "HALL_PHI_E_PHI_M_OFFSET",
        "HALL_PHI_M",
    ],
    "ABN": [
        "ABN_DECODER_COUNT",
        "ABN_DECODER_MODE",
        "ABN_DECODER_PPR",
        "ABN_DECODER_PHI_E_PHI_M_OFFSET",
        "ABN_DECODER_PHI_E_PHI_M",
    ],
    "ADC": [
        "ADC_I1_RAW_ADC_I0_RAW",
        "ADC_IWY_IUX",
        "ADC_IV",
        "ADC_I0_SCALE_OFFSET",
        "ADC_I1_SCALE_OFFSET",
    ],
    "AENC": [
        "AENC_DECODER_MODE",
        "AENC_DECODER_PPR",
        "ADC_I1_RAW_ADC_I0_RAW",
        "ADC_AGPI_A_RAW_ADC_VM_RAW",
        "ADC_AENC_UX_RAW_ADC_AGPI_B_RAW",
        "ADC_AENC_WY_RAW_ADC_AENC_VN_RAW",
        "AENC_DECODER_PHI_A_RAW",
    ],
    "PWM": [
        "PWM_POLARITIES",
        "PWM_MAXCNT",
        "PWM_BBM_H_BBM_L",
        "PWM_SV_CHOP",
        "MOTOR_TYPE_N_POLE_PAIRS",
    ],
    "PIDCONF": [
        "PID_FLUX_P_FLUX_I",
        "PID_TORQUE_P_TORQUE_I",
        "PID_VELOCITY_P_VELOCITY_I",
        "PID_POSITION_P_POSITION_I",
    ],
    "MONITOR": [
        "INTERIM_PIDIN_TARGET_TORQUE",
        "PID_VELOCITY_ACTUAL",
        "INTERIM_PIDIN_TARGET_VELOCITY",
        "PID_POSITION_ACTUAL",
    ],
    "PID": [
        "PID_FLUX_P_FLUX_I",
        "PID_TORQUE_P_TORQUE_I",
        "PID_VELOCITY_P_VELOCITY_I",
        "PID_POSITION_P_POSITION_I",
        "PID_TORQUE_FLUX_TARGET",
        "PID_TORQUE_FLUX_OFFSET",
        "PID_VELOCITY_TARGET",
        "PID_POSITION_TARGET",
        "PID_TORQUE_FLUX_ACTUAL",
        "INTERIM_PIDIN_TARGET_FLUX",
        "PID_ERROR_PID_FLUX_ERROR",
        "PID_ERROR_PID_FLUX_ERROR_SUM",
        "INTERIM_PIDIN_TARGET_TORQUE",
        "PID_ERROR_PID_TORQUE_ERROR",
        "PID_ERROR_PID_TORQUE_ERROR_SUM",
        "PID_VELOCITY_ACTUAL",
        "INTERIM_PIDIN_TARGET_VELOCITY",
        "PID_ERROR_PID_VELOCITY_ERROR",
        "PID_ERROR_PID_VELOCITY_ERROR_SUM",
        "PID_POSITION_ACTUAL",
        "INTERIM_PIDIN_TARGET_POSITION",
        "PID_ERROR_PID_POSITION_ERROR",
        "PID_ERROR_PID_POSITION_ERROR_SUM",
    ],
    "STEP": [
        "STEP_WIDTH",
        "PHI_E",
        "MODE_RAMP_MODE_MOTION",
        "STATUS_FLAGS",
        "PID_POSITION_TARGET",
    ],
    "FILTERS": [
        "CONFIG_BIQUAD_X_A_1",
        "CONFIG_BIQUAD_X_A_2",
        "CONFIG_BIQUAD_X_B_0",
        "CONFIG_BIQUAD_X_B_1",
        "CONFIG_BIQUAD_X_B_2",
        "CONFIG_BIQUAD_X_ENABLE",
        "CONFIG_BIQUAD_V_A_1",
        "CONFIG_BIQUAD_V_A_2",
        "CONFIG_BIQUAD_V_B_0",
        "CONFIG_BIQUAD_V_B_1",
        "CONFIG_BIQUAD_V_B_2",
        "CONFIG_BIQUAD_V_ENABLE",
        "CONFIG_BIQUAD_T_A_1",
        "CONFIG_BIQUAD_T_A_2",
        "CONFIG_BIQUAD_T_B_0",
        "CONFIG_BIQUAD_T_B_1",
        "CONFIG_BIQUAD_T_B_2",
        "CONFIG_BIQUAD_T_ENABLE",
        "CONFIG_BIQUAD_F_A_1",
        "CONFIG_BIQUAD_F_A_2",
        "CONFIG_BIQUAD_F_B_0",
        "CONFIG_BIQUAD_F_B_1",
        "CONFIG_BIQUAD_F_B_2",
        "CONFIG_BIQUAD_F_ENABLE",
    ],
}


######################################################################
# Biquad filter utilities
######################################################################


# Filter design formula from 4671 datasheet
def biquad_lpf_tmc(fs, f, D):
    w0 = 2.0 * math.pi * f / fs
    b2 = b1 = 0.0
    b0 = 1.0
    a2 = 1.0 / (w0**2)
    a1 = 2.0 * D / w0
    a0 = 1.0
    return b0, b1, b2, a0, a1, a2


# Filter design formulae from https://www.w3.org/TR/audio-eq-cookbook/


# Design a biquad low pass filter in canonical form
def biquad_lpf(fs, f, Q):
    w0 = 2.0 * math.pi * f / fs
    cw0 = math.cos(w0)
    sw0 = math.sin(w0)
    alpha = 0.5 * sw0 / Q
    b1 = 1.0 - cw0
    b0 = b2 = b1 / 2.0
    a0 = 1 + alpha
    a1 = -2.0 * cw0
    a2 = 1 - alpha
    return b0, b1, b2, a0, a1, a2


# Design a biquad notch filter in canonical form
def biquad_notch(fs, f, Q):
    w0 = 2.0 * math.pi * f / fs
    cw0 = math.cos(w0)
    sw0 = math.sin(w0)
    alpha = 0.5 * sw0 / Q
    b1 = -2.0 * cw0
    b0 = b2 = 1.0
    a0 = 1 + alpha
    a1 = -2.0 * cw0
    a2 = 1 - alpha
    return b0, b1, b2, a0, a1, a2


# Z-transform and normalise a biquad filter, according to TMC
def biquad_Z_tmc(T, b0, b1, b2, a0, a1, a2):
    den = T**2 - 2 * a1 + 4 * a2
    b2z = (b0 * (T**2) + 2 * b1 * T + 4 * b2) / den
    b1z = (2 * b0 * (T**2) - 8 * b2) / den
    b0z = (b0 * (T**2) - 2 * b1 * T + 4 * b2) / den
    a2z = (T**2 + 2 * a1 * T + 4 * a2) / den
    a1z = (2 * (T**2) - 8 * a2) / den
    e29 = 2**29
    b0 = round(b0z / a0 * e29)
    b1 = round(b1z / a0 * e29)
    b2 = round(b2z / a0 * e29)
    a1 = round(-a1z / a0 * e29)
    a2 = round(-a2z / a0 * e29)
    # return in the same order as the config registers
    return a1, a2, b0, b1, b2


# Normalise a biquad filter, according to TMC
def biquad_tmc(b0, b1, b2, a0, a1, a2):
    e29 = 2**29
    b0 = round(b0 / a0 * e29)
    b1 = round(b1 / a0 * e29)
    b2 = round(b2 / a0 * e29)
    a1 = round(-a1 / a0 * e29)
    a2 = round(-a2 / a0 * e29)
    # return in the same order as the config registers
    return a1, a2, b0, b1, b2


# S-IMC PI controller design
def simc(k, theta, tau1, tauc):
    Kc = (1.0 / k) * (tau1 / (tauc + theta))
    taui = min(tau1, 4 * (tauc + theta))
    return Kc, taui


######################################################################
# Current control
######################################################################


MAX_CURRENT = 10.000


class CurrentHelper:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        self.run_current = config.getfloat("run_current", above=0.0, maxval=MAX_CURRENT)
        self.homing_current = config.getfloat(
            "homing_current", above=0.0, maxval=MAX_CURRENT, default=self.run_current
        )
        self.current_scale = config.getfloat(
            "current_scale_ma_lsb", 1.272, above=0.0, maxval=10
        )
        self.flux_limit = self._calc_flux_limit(self.run_current)
        self.fields.set_field("PID_TORQUE_FLUX_LIMITS", self.flux_limit)

    def _calc_flux_limit(self, current):
        flux_limit = round(current * 1e3 / self.current_scale)
        return flux_limit

    def convert_adc_current(self, adc):
        return adc * self.current_scale * 1e-3

    def get_run_current(self):
        return self.run_current

    def get_homing_current(self):
        return self.homing_current

    def get_current(self):
        c = self.convert_adc_current(self._read_field("PID_TORQUE_FLUX_LIMITS"))
        iux = self.convert_adc_current(self._read_field("ADC_IUX"))
        iv = self.convert_adc_current(self._read_field("ADC_IV"))
        iwy = self.convert_adc_current(self._read_field("ADC_IWY"))
        return c, MAX_CURRENT, iux, iv, iwy

    def set_current(self, run_current):
        self.run_current = run_current
        self.flux_limit = self._calc_flux_limit(self.run_current)
        self._write_field("PID_TORQUE_FLUX_LIMITS", self.flux_limit)
        return self.flux_limit

    def _read_field(self, field):
        reg_name = self.fields.lookup_register(field)
        reg_value = self.mcu_tmc.get_register(reg_name)
        return self.fields.get_field(field, reg_value=reg_value, reg_name=reg_name)

    def _write_field(self, field, val):
        reg_name = self.fields.lookup_register(field)
        reg_value = self.mcu_tmc.get_register(reg_name)
        self.mcu_tmc.set_register(
            reg_name,
            self.fields.set_field(field, val, reg_value=reg_value, reg_name=reg_name),
        )


######################################################################
# Helper to configure the microstep settings
######################################################################


def StepHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    stepper_name = " ".join(config.get_name().split()[1:])
    if not config.has_section(stepper_name):
        raise config.error(
            "Could not find config section '[%s]' required by tmc4671 driver"
            % (stepper_name,)
        )
    sconfig = config.getsection(stepper_name)
    steps = {1 << i: 1 << i for i in range(0, 16)}
    res = sconfig.getchoice("full_steps_per_rotation", steps, default=8)
    mres = sconfig.getchoice("microsteps", steps, default=256)
    if res * mres > 65536:
        raise config.error(
            "Product of res and mres must be less than 65536 for [%s]" % (stepper_name,)
        )
    step_width = 65536 // (res * mres)
    fields.set_field("STEP_WIDTH", step_width)


######################################################################
# Periodic error checking
######################################################################


class TMCErrorCheck:
    def __init__(self, config, mcu_tmc):

        self.printer = config.get_printer()
        name_parts = config.get_name().split()
        self.stepper_name = " ".join(name_parts[1:])
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        self.check_timer = None
        self.status_warn_mask = self._make_mask(
            [
                "PID_IQ_TARGET_LIMIT",
                "PID_ID_TARGET_LIMIT",
                "PID_V_OUTPUT_LIMIT",
                "REF_SW_R",
                "REF_SW_L",
            ]
        )
        # Useful for debugging
        # self.status_warn_mask = 0xffffffff
        self.status_error_mask = self._make_mask(
            ["PWM_MIN", "PWM_MAX", "ADC_I_CLIPPED", "AENC_CLIPPED"]
        )
        self.last_status = 0
        # Setup for temperature query
        self.adc_temp = None
        self.adc_temp_reg = config.getchoice(
            "adc_temp_reg", ADC_GPIO_FIELDS, default=None
        )
        if self.adc_temp_reg is not None:
            pheaters = self.printer.load_object(config, "heaters")
            pheaters.register_monitor(config)

    def _make_mask(self, entries):
        mask = 0
        for f in entries:
            mask = self.fields.set_field(f, 1, mask, "STATUS_FLAGS")
        return mask

    def _query_status(self):
        try:
            status = self.mcu_tmc.get_register("STATUS_FLAGS")
            fmt = self.fields.pretty_format("STATUS_FLAGS", status)
            # logging.info("TMC 4671 '%s' raw %s", self.stepper_name, fmt)
            if (
                status & self.status_warn_mask
                != self.last_status & self.status_warn_mask
            ):
                fmt = self.fields.pretty_format("STATUS_FLAGS", status)
                logging.info("TMC 4671 '%s' reports %s", self.stepper_name, fmt)
            self.mcu_tmc.set_register_once("STATUS_FLAGS", 0)
            status = self.mcu_tmc.get_register("STATUS_FLAGS")
            self.last_status = status
            if status & self.status_error_mask:
                fmt = self.fields.pretty_format("STATUS_FLAGS", status)
                raise self.printer.command_error(
                    "TMC 4671 '%s' reports error: %s" % (self.stepper_name, fmt)
                )
            for reg_name in DumpGroups["MONITOR"]:
                val = self.mcu_tmc.get_register(reg_name)
                logging.info(
                    "TMC 4671 '%s' %s: %s",
                    self.stepper_name,
                    reg_name,
                    self.fields.pretty_format(reg_name, val),
                )
        except self.printer.command_error as e:
            self.printer.invoke_shutdown(str(e))
            return self.printer.get_reactor().NEVER

    def _query_temperature(self):
        try:
            if self.adc_temp_reg is not None:
                self.adc_temp = self.mcu_tmc.read_field(self.adc_temp_reg)
                # TODO: remove this, just temp logging
                # self._convert_temp(self.adc_temp)
        except self.printer.command_error as e:
            # Ignore comms error for temperature
            self.adc_temp = None
            return

    def _do_periodic_check(self, eventtime):
        try:
            self._query_status()
            self._query_temperature()
        except self.printer.command_error as e:
            self.printer.invoke_shutdown(str(e))
            return self.printer.get_reactor().NEVER
        return eventtime + 1.0

    def stop_checks(self):
        if self.check_timer is None:
            return
        self.printer.get_reactor().unregister_timer(self.check_timer)
        self.check_timer = None

    def start_checks(self):
        if self.check_timer is not None:
            self.stop_checks()
        reactor = self.printer.get_reactor()
        curtime = reactor.monotonic()
        self.check_timer = reactor.register_timer(
            self._do_periodic_check, curtime + 1.0
        )
        return True

    # Per the OpenFFBoard firmware source
    # [thermistor ffboard]
    # temperature1: 25
    # resistance1: 10000
    # beta: 4300
    def _convert_temp(self, adc):
        v = adc - 0x7FFF
        if v < 0:
            temp = 0.0
        else:
            # temp = 1500.0 * 43252.0 / float(v)
            temp = 64878000.0 / float(v)
            # temp = (1.0/298.15) + math.log(temp / 10000.0) / 4300.0
            temp = (0.003354016) + math.log(temp / 10000.0) / 4300.0
            temp = 1.0 / temp
            temp -= 273.15
        logging.info("TMC %s temp: %g", self.stepper_name, temp)

    def get_status(self, eventtime=None):
        if self.check_timer is None:
            return {"drv_status": None, "temperature": None}
        temp = None
        if self.adc_temp is not None:
            # Convert it to temp here
            temp = self._convert_temp(adc)
        return {"drv_status": None, "temperature": temp}


######################################################################
# Helper class for "sensorless homing"
######################################################################


class TMCVirtualPinHelper:
    def __init__(self, config, mcu_tmc, current_helper):
        self.printer = config.get_printer()
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        self.current_helper = current_helper
        self.diag_pin = config.get("diag_pin", None)
        self.mcu_endstop = None
        self.en_pwm = False
        # Stalls show up as either IQ_ERRSUM or V_OUTPUT.
        # Include the reference switches so can connect endstops
        # to driver boards.
        self.status_mask_entries = config.getlist(
            "homing_mask",
            [
                "PID_IQ_OUTPUT_LIMIT",
                # "PID_ID_OUTPUT_LIMIT",
                # "PID_IQ_ERRSUM_LIMIT",
                # "PID_ID_ERRSUM_LIMIT",
                "PID_IQ_TARGET_LIMIT",
                "PID_ID_TARGET_LIMIT",
                # "PID_X_OUTPUT_LIMIT",
                # "PID_V_OUTPUT_LIMIT",
                "REF_SW_R",
                "REF_SW_L",
            ],
        )
        self.status_mask = 0
        for f in self.status_mask_entries:
            self.status_mask = self.fields.set_field(
                f, 1, self.status_mask, "STATUS_FLAGS"
            )
        # Register virtual_endstop pin
        name_parts = config.get_name().split()
        self.name = "_".join(name_parts)
        ppins = self.printer.lookup_object("pins")
        ppins.register_chip("%s_%s" % (name_parts[0], name_parts[-1]), self)

    def setup_pin(self, pin_type, pin_params):
        # Validate pin
        ppins = self.printer.lookup_object("pins")
        if pin_type != "endstop" or pin_params["pin"] != "virtual_endstop":
            raise ppins.error("tmc virtual endstop only useful as endstop")
        if pin_params["invert"] or pin_params["pullup"]:
            raise ppins.error("Can not pullup/invert tmc virtual pin")
        if self.diag_pin is None:
            raise ppins.error("tmc virtual endstop requires diag pin config")
        # Setup for sensorless homing
        self.printer.register_event_handler(
            "homing:homing_move_begin", self.handle_homing_move_begin
        )
        self.printer.register_event_handler(
            "homing:homing_move_end", self.handle_homing_move_end
        )
        self.mcu_endstop = ppins.setup_pin("endstop", self.diag_pin)
        return self.mcu_endstop

    def handle_homing_move_begin(self, hmove):
        if self.mcu_endstop not in hmove.get_mcu_endstops():
            return
        # self.mcu_tmc.write_field("PID_VELOCITY_LIMIT", 3000)
        self.current_helper.set_current(self.current_helper.get_homing_current())
        self.mcu_tmc.set_register_once("STATUS_FLAGS", 0)
        self.mcu_tmc.write_field("STATUS_MASK", self.status_mask)
        status = self.mcu_tmc.get_register("STATUS_FLAGS")
        fmt = self.fields.pretty_format("STATUS_FLAGS", status)
        logging.info("TMC 4671 '%s' status at homing start %s", self.name, fmt)

    def handle_homing_move_end(self, hmove):
        status = self.mcu_tmc.get_register("STATUS_FLAGS")
        fmt = self.fields.pretty_format("STATUS_FLAGS", status)
        logging.info("TMC 4671 '%s' status at homing end %s", self.name, fmt)
        if self.mcu_endstop not in hmove.get_mcu_endstops():
            return
        # self.mcu_tmc.write_field("PID_VELOCITY_LIMIT", 0x300000)
        self.current_helper.set_current(self.current_helper.get_run_current())
        self.mcu_tmc.write_field("STATUS_MASK", 0)
        self.mcu_tmc.set_register_once("STATUS_FLAGS", 0)


######################################################################
# SPI communication, fields, and registers
######################################################################


# 4671 does not support chaining, so that's removed
# 4671 protocol does not require dummy reads
# default speed is 1 MHz, conservative for the device.
# would need timing control if going faster than 2 MHz.
class MCU_TMC_SPI_simple:
    def __init__(self, config, pin_option="cs_pin"):
        self.printer = config.get_printer()
        self.mutex = self.printer.get_reactor().mutex()
        self.spi = bus.MCU_SPI_from_config(
            config, 3, default_speed=1000000, pin_option=pin_option
        )

    def reg_read(self, reg):
        cmd = [reg, 0x00, 0x00, 0x00, 0x00]
        # self.spi.spi_send(cmd)
        with self.mutex:
            params = self.spi.spi_transfer(cmd)
        pr = bytearray(params["response"])
        return (pr[1] << 24) | (pr[2] << 16) | (pr[3] << 8) | pr[4]

    def reg_write(self, reg, val, print_time=None):
        minclock = 0
        if print_time is not None:
            minclock = self.spi.get_mcu().print_time_to_clock(print_time)
        data = [
            (reg | 0x80) & 0xFF,
            (val >> 24) & 0xFF,
            (val >> 16) & 0xFF,
            (val >> 8) & 0xFF,
            val & 0xFF,
        ]
        with self.mutex:
            self.spi.spi_send(data, minclock)
        return self.reg_read(reg)


# Helper code for working with TMC devices via SPI
# 4671 does have overlay registers, so support those
class MCU_TMC_SPI:
    def __init__(self, config, name_to_reg, fields, tmc_frequency, pin_option):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.tmc_spi = MCU_TMC_SPI_simple(config, pin_option=pin_option)
        self.mutex = self.tmc_spi.mutex
        self.name_to_reg = name_to_reg
        self.fields = fields
        self.tmc_frequency = tmc_frequency

    def get_fields(self):
        return self.fields

    def get_register(self, reg_name):
        reg, addr = self.name_to_reg[reg_name]
        if addr is not None:
            for retry in range(5):
                v = self.tmc_spi.reg_write(reg + 1, addr)
                if v == addr:
                    break
            else:
                raise self.printer.command_error(
                    "Unable to write tmc spi '%s' address register %s (last read %x)"
                    % (self.name, reg_name, v)
                )
        read = self.tmc_spi.reg_read(reg)
        return read

    def set_register_once(self, reg_name, val, print_time=None):
        reg, addr = self.name_to_reg[reg_name]
        if addr is not None:
            v = self.tmc_spi.reg_write(reg + 1, addr, print_time)
            if v != addr:
                raise self.printer.command_error(
                    "Unable to write tmc spi '%s' address register %s (last read %x)"
                    % (self.name, reg_name, v)
                )
        v = self.tmc_spi.reg_write(reg, val, print_time)

    def set_register(self, reg_name, val, print_time=None):
        reg, addr = self.name_to_reg[reg_name]
        if addr is not None:
            for retry in range(5):
                v = self.tmc_spi.reg_write(reg + 1, addr, print_time)
                if v == addr:
                    break
            else:
                raise self.printer.command_error(
                    "Unable to write tmc spi '%s' address register %s (last read %x)"
                    % (self.name, reg_name, v)
                )
        for retry in range(5):
            v = self.tmc_spi.reg_write(reg, val, print_time)
            if v == val:
                return
        raise self.printer.command_error(
            "Unable to write tmc spi '%s' register %s" % (self.name, reg_name)
        )

    def get_tmc_frequency(self):
        return self.tmc_frequency

    def read_field(self, field):
        reg_name = self.fields.lookup_register(field)
        reg_value = self.get_register(reg_name)
        return self.fields.get_field(field, reg_value=reg_value, reg_name=reg_name)

    def write_field(self, field, val):
        reg_name = self.fields.lookup_register(field)
        reg_value = self.get_register(reg_name)
        self.set_register(
            reg_name,
            self.fields.set_field(field, val, reg_value=reg_value, reg_name=reg_name),
        )


######################################################################
# Main driver class
######################################################################


class TMC4671:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.stepper_name = " ".join(config.get_name().split()[1:])
        self.name = config.get_name().split()[-1]
        self.init_done = False
        self.alignment_done = False
        self.fields = FieldHelper(
            Fields,
            signed_fields=SignedFields,
            field_formatters=FieldFormatters,
            prefix="foc_",
        )
        # 6100 is optional for boards without one.
        gcode = self.gcode = self.printer.lookup_object("gcode")
        if config.get("drv_cs_pin", None) is not None:
            self.fields6100 = FieldHelper(Fields6100, prefix="drv_")
            self.mcu_tmc6100 = MCU_TMC_SPI(
                config, Registers6100, self.fields6100, 12e6, pin_option="drv_cs_pin"
            )
            gcode.register_mux_command(
                "DUMP_TMC6100",
                "STEPPER",
                self.name,
                self.cmd_DUMP_TMC6100,
                desc=self.cmd_DUMP_TMC6100_help,
            )
        else:
            self.fields6100 = None
            self.mcu_tmc6100 = None
        self.mcu_tmc = MCU_TMC_SPI(
            config, Registers, self.fields, TMC_FREQUENCY, pin_option="cs_pin"
        )
        self.read_translate = None
        self.read_registers = Registers.keys()
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.stepper = None
        self.stepper_enable = self.printer.load_object(config, "stepper_enable")
        self.printer.register_event_handler(
            "klippy:mcu_identify", self._handle_mcu_identify
        )
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        # Register commands
        self.step_helper = StepHelper(config, self.mcu_tmc)
        self.current_helper = CurrentHelper(config, self.mcu_tmc)
        self.error_helper = TMCErrorCheck(config, self.mcu_tmc)
        # TMCVirtualPinHelper(config, self.mcu_tmc, self.current_helper)
        gcode.register_mux_command(
            "SET_TMC_FIELD",
            "STEPPER",
            self.name,
            self.cmd_SET_TMC_FIELD,
            desc=self.cmd_SET_TMC_FIELD_help,
        )
        gcode.register_mux_command(
            "DUMP_TMC",
            "STEPPER",
            self.name,
            self.cmd_DUMP_TMC,
            desc=self.cmd_DUMP_TMC_help,
        )
        gcode.register_mux_command(
            "TMC_DEBUG_MOVE",
            "STEPPER",
            self.name,
            self.cmd_TMC_DEBUG_MOVE,
            desc=self.cmd_TMC_DEBUG_MOVE_help,
        )
        gcode.register_mux_command(
            "TMC_TUNE_PID",
            "STEPPER",
            self.name,
            self.cmd_TMC_TUNE_PID,
            desc=self.cmd_TMC_TUNE_PID_help,
        )
        gcode.register_mux_command(
            "TMC_ENC_OFFSETS",
            "STEPPER",
            self.name,
            self.cmd_TMC_ENC_OFFSETS,
            desc=self.cmd_TMC_ENC_OFFSETS_help,
        )
        gcode.register_mux_command(
            "INIT_TMC",
            "STEPPER",
            self.name,
            self.cmd_INIT_TMC,
            desc=self.cmd_INIT_TMC_help,
        )
        gcode.register_mux_command(
            "SET_TMC_CURRENT",
            "STEPPER",
            self.name,
            self.cmd_SET_TMC_CURRENT,
            desc=self.cmd_SET_TMC_CURRENT_help,
        )
        # Allow other registers to be set from the config
        set_config_field = self.fields.set_config_field
        if self.fields6100 is not None:
            set_config6100_field = self.fields6100.set_config_field
            # defaults as per 4671+6100 BOB datasheet
            set_config6100_field(config, "singleline", 0)
            set_config6100_field(config, "normal", 1)
            set_config6100_field(config, "DRVSTRENGTH", 0)
            set_config6100_field(config, "BBMCLKS", 10)
        # This should not really be set to anything else
        # therefore not providing convenience interface
        maxcnt = 0xF9F
        set_config_field(config, "PWM_MAXCNT", maxcnt)  # 25 kHz
        # These are used later by filter definitions
        self.pwmfreq = 4.0 * TMC_FREQUENCY / (maxcnt + 1.0)
        self.pwmT = maxcnt * 10e-9
        set_config_field(config, "PWM_BBM_L", 10)
        set_config_field(config, "PWM_BBM_H", 10)
        set_config_field(config, "PWM_CHOP", 7)
        set_config_field(config, "PWM_SV", 1)
        set_config_field(config, "MOTOR_TYPE", 3)
        set_config_field(config, "N_POLE_PAIRS", 4)
        set_config_field(config, "ADC_I_UX_SELECT", 0)
        set_config_field(config, "ADC_I_V_SELECT", 2)
        set_config_field(config, "ADC_I_WY_SELECT", 1)
        set_config_field(config, "ADC_I0_SELECT", 0)
        set_config_field(config, "ADC_I1_SELECT", 1)
        # set_config_field(config, "CFG_ADC_I0", 0)
        # set_config_field(config, "CFG_ADC_I1", 0)
        # set_config_field(config, "CFG_ADC_VM", 4)
        set_config_field(config, "AENC_DEG", 1)  # 120 degree analog hall
        set_config_field(config, "AENC_PPR", 1)  # 120 degree analog hall
        set_config_field(config, "ABN_APOL", 0)
        set_config_field(config, "ABN_BPOL", 0)
        set_config_field(config, "ABN_NPOL", 0)
        set_config_field(config, "ABN_USE_ABN_AS_N", 0)
        set_config_field(config, "ABN_CLN", 0)
        set_config_field(config, "ABN_DIRECTION", 0)
        set_config_field(config, "ABN_DECODER_PPR", 1600)
        set_config_field(config, "HALL_INTERP", 0)
        set_config_field(config, "HALL_SYNC", 1)
        set_config_field(config, "HALL_POLARITY", 0)
        set_config_field(config, "HALL_DIR", 0)
        set_config_field(config, "HALL_DPHI_MAX", 0xAAAA)
        set_config_field(config, "HALL_PHI_E_OFFSET", 0)
        set_config_field(config, "HALL_BLANK", 2)
        set_config_field(config, "PHI_E_SELECTION", 3)  # ABN
        set_config_field(config, "POSITION_SELECTION", 9)  # ABN
        set_config_field(config, "VELOCITY_SELECTION", 3)  # ABN
        # set_config_field(config, "VELOCITY_METER_SELECTION", 0) # Default velocity meter
        set_config_field(
            config, "VELOCITY_METER_SELECTION", 1
        )  # PWM frequency velocity meter
        set_config_field(
            config, "MODE_PID_SMPL", 2
        )  # Advanced PID samples position at fPWM/3
        set_config_field(config, "MODE_PID_TYPE", 1)  # Advanced PID mode
        set_config_field(
            config, "PIDOUT_UQ_UD_LIMITS", 31500
        )  # Voltage limit, 32768 = Vm
        # TODO: get this from the size of the printer
        set_config_field(config, "PID_POSITION_LIMIT_LOW", -0x10000000)
        set_config_field(config, "PID_POSITION_LIMIT_HIGH", 0x10000000)
        # TODO: Units, what should this be anyway?
        set_config_field(config, "PID_VELOCITY_LIMIT", 0x3000000)
        set_config_field(config, "PID_FLUX_OFFSET", 0)
        pid_defaults = [
            ("FLUX_P", 3.6, "CURRENT_P_n", 1),
            ("FLUX_I", 0.27, "CURRENT_I_n", 1),
            ("TORQUE_P", 3.6, "CURRENT_P_n", 1),
            ("TORQUE_I", 0.27, "CURRENT_I_n", 1),
            ("VELOCITY_P", 5.0, "VELOCITY_P_n", 1),
            ("VELOCITY_I", 0.5, "VELOCITY_I_n", 1),
            ("POSITION_P", 1.0, "POSITION_P_n", 1),
            ("POSITION_I", 0.5, "POSITION_I_n", 1),
        ]
        self.pid_helpers = {
            n: PIDHelper(config, self.mcu_tmc, n, v, nn, nv)
            for n, v, nn, nv in pid_defaults
        }

    def _read_field(self, field):
        reg_name = self.fields.lookup_register(field)
        reg_value = self.mcu_tmc.get_register(reg_name)
        return self.fields.get_field(field, reg_value=reg_value, reg_name=reg_name)

    def _write_field(self, field, val):
        reg_name = self.fields.lookup_register(field)
        reg_value = self.mcu_tmc.get_register(reg_name)
        self.mcu_tmc.set_register(
            reg_name,
            self.fields.set_field(field, val, reg_value=reg_value, reg_name=reg_name),
        )

    def enable_biquad(self, enable_field, *biquad):
        reg, addr = self.mcu_tmc.name_to_reg[enable_field]
        for o, i in enumerate(biquad):
            self.mcu_tmc.tmc_spi.reg_write(reg + 1, addr - 6 + o)
            self.mcu_tmc.tmc_spi.reg_write(reg, i)
        self.mcu_tmc.tmc_spi.reg_write(reg + 1, addr)
        self.mcu_tmc.tmc_spi.reg_write(reg, 1)

    def disable_biquad(self, enable_field):
        reg, addr = self.mcu_tmc.name_to_reg[enable_field]
        self.mcu_tmc.tmc_spi.reg_write(reg + 1, addr)
        self.mcu_tmc.tmc_spi.reg_write(reg, 0)

    def _do_enable(self, print_time):
        try:
            # Only need to do this once.
            if not self.alignment_done:
                # Just test the PID, as it also sets up the encoder offsets
                self._write_field("PID_FLUX_TARGET", 0)
                self._write_field("PID_TORQUE_TARGET", 0)
                self._write_field("PID_VELOCITY_TARGET", 0)
                P, I = self._tune_flux_pid(True, 1.0, print_time)
                self._write_field("ABN_DECODER_COUNT", 0)
                self._write_field("PID_POSITION_TARGET", 0)
                self.alignment_done = True
            # Do this every time, may have moved while disabled
            self._write_field("PID_TORQUE_TARGET", 0)
            self._write_field("PID_VELOCITY_TARGET", 0)
            self._write_field("PID_POSITION_TARGET", 0)
            self._write_field("PID_POSITION_ACTUAL", 0)
            self.error_helper.start_checks()
            self._write_field("MODE_MOTION", MotionMode.position_mode)
        except self.printer.command_error as e:
            self.printer.invoke_shutdown(str(e))

    def _do_disable(self, print_time):
        try:
            self.error_helper.stop_checks()
            # Switching off the enable line will turn off the drivers
            # but, belt and braces, stop the controller as well.
            self._write_field("MODE_MOTION", MotionMode.stopped_mode)
            self._write_field("PID_TORQUE_TARGET", 0)
            self._write_field("PID_VELOCITY_TARGET", 0)
            self._write_field("PID_POSITION_TARGET", 0)
            self._write_field("PID_POSITION_ACTUAL", 0)
        except self.printer.command_error as e:
            self.printer.invoke_shutdown(str(e))

    def _handle_mcu_identify(self):
        # Lookup stepper object
        force_move = self.printer.lookup_object("force_move")
        self.stepper = force_move.lookup_stepper(self.stepper_name)
        # Note default pulse duration and step_both_edge unavailable
        self.stepper.setup_default_pulse_duration(0.000000100, False)

    def _handle_stepper_enable(self, print_time, is_enable):
        if is_enable:
            cb = lambda ev: self._do_enable(print_time)
        else:
            cb = lambda ev: self._do_disable(print_time)
        self.printer.get_reactor().register_callback(cb)

    def _handle_connect(self):
        print_time = self.printer.lookup_object("toolhead").get_last_move_time()
        # Check for soft stepper enable/disable
        enable_line = self.stepper_enable.lookup_enable(self.stepper_name)
        # Send init
        try:
            self._init_registers()
        except self.printer.command_error as e:
            logging.info("TMC %s failed to init: %s", self.name, str(e))
        enable_line.register_state_callback(self._handle_stepper_enable)

    def _handle_ready(self):
        print_time = self.printer.lookup_object("toolhead").get_last_move_time()
        # Now enable 6100
        if self.fields6100 is not None:
            self.mcu_tmc6100.set_register(
                "GCONF", self.fields6100.set_field("disable", 0), print_time
            )
        enable_line = self.stepper_enable.lookup_enable(self.stepper_name)
        enable_line.motor_enable(print_time)
        # Just test the PID, as it also sets up the encoder offsets
        self._write_field("PID_FLUX_TARGET", 0)
        self._write_field("PID_TORQUE_TARGET", 0)
        self._write_field("PID_VELOCITY_TARGET", 0)
        P, I = self._tune_flux_pid(True, 1.0, print_time)
        self._write_field("ABN_DECODER_COUNT", 0)
        self._write_field("PID_POSITION_TARGET", 0)
        print_time = self.printer.lookup_object("toolhead").get_last_move_time()
        enable_line.motor_disable(print_time)
        self.alignment_done = True
        self._write_field("STATUS_MASK", 0)
        self._write_field("PID_FLUX_TARGET", 0)
        self._write_field("PID_TORQUE_TARGET", 0)
        self._write_field("PID_VELOCITY_TARGET", 0)
        self._write_field("ABN_DECODER_COUNT", 0)
        self._write_field("PID_POSITION_TARGET", 0)
        self._write_field("MODE_MOTION", MotionMode.stopped_mode)
        self.init_done = True
        self.gcode.respond_info("init-ed 4671!")

    def _calibrate_adc(self, print_time):
        self._write_field("PWM_CHOP", 0)
        self._write_field("CFG_ADC_I0", 0)
        self._write_field("CFG_ADC_I1", 0)
        i0_off, i1_off = self._sample_adc("ADC_I1_RAW_ADC_I0_RAW")
        # Following code would calibrate the scalers as well.
        # However, not doing that makes things simpler, and the
        # calibration result is always +- 1 from nominal.
        # self._write_field("CFG_ADC_I0", 2)
        # self._write_field("CFG_ADC_I1", 2)
        # i0_l, i1_l = self._sample_adc("ADC_I1_RAW_ADC_I0_RAW")
        # self._write_field("CFG_ADC_I0", 3)
        # self._write_field("CFG_ADC_I1", 3)
        # i0_h, i1_h = self._sample_adc("ADC_I1_RAW_ADC_I0_RAW")
        # self._write_field("CFG_ADC_I0", 0)
        # self._write_field("CFG_ADC_I1", 0)
        # self._write_field("ADC_I1_SCALE", (256*(i1_h-i1_l))//32768)
        # self._write_field("ADC_I0_SCALE", (256*(i0_h-i0_l))//32768)
        self._write_field("ADC_I1_SCALE", 256)
        self._write_field("ADC_I0_SCALE", 256)
        self._write_field("ADC_I1_OFFSET", i1_off)
        self._write_field("ADC_I0_OFFSET", i0_off)
        self._write_field("PWM_CHOP", 7)
        logging.info("TMC 4671 %s ADC offsets I0=%d I1=%d", self.name, i0_off, i1_off)
        logging.info("TMC 4671 %s ADC VM %s", self.name, str(self._sample_vm()))
        # Now calibrate for brake chopper
        vml, vmh = self._sample_vm()
        vmr = abs(vmh - vml)
        high = (vmh - 32768) // 20 + 2 * vmr + vmh
        if high < 65536:
            self._write_field("ADC_VM_LIMIT_HIGH", high)
            self._write_field("ADC_VM_LIMIT_LOW", vmr + vmh)
        else:
            # What else can we do but turn the brake off?
            self._write_field("ADC_VM_LIMIT_HIGH", 0)
            self._write_field("ADC_VM_LIMIT_LOW", 0)

    def _sample_adc(self, reg_name):
        self.printer.lookup_object("toolhead").dwell(0.2)
        reg, addr = self.mcu_tmc.name_to_reg[reg_name]
        self.mcu_tmc.tmc_spi.reg_write(reg + 1, addr)
        i1 = []
        i0 = []
        n = 100
        for i in range(n):
            v = self.fields.get_reg_fields(reg_name, self.mcu_tmc.tmc_spi.reg_read(reg))
            i1.append(v["ADC_I1_RAW"])
            i0.append(v["ADC_I0_RAW"])
            self.printer.lookup_object("toolhead").dwell(0.0005)
        return median_low(i0), median_low(i1)

    def _sample_vm(self):
        self.printer.lookup_object("toolhead").dwell(0.2)
        vm = []
        n = 100
        for i in range(n):
            vm.append(self._read_field("ADC_VM_RAW"))
            self.printer.lookup_object("toolhead").dwell(0.0005)
        return min(vm), max(vm)

    def _tune_flux_pid(self, test_existing, derate, print_time):
        return self._tune_pid("FLUX", 1.5, derate, True, test_existing, print_time)

    def _tune_torque_pid(self, test_existing, derate, print_time):
        return self._tune_pid("TORQUE", 1.0, derate, True, test_existing, print_time)

    def _tune_pid(self, X, Kc, derate, offsets, test_existing, print_time):
        ch = self.current_helper
        dwell = self.printer.lookup_object("toolhead").dwell
        old_mode = self._read_field("MODE_MOTION")
        old_phi_e_selection = self._read_field("PHI_E_SELECTION")
        self._write_field("MODE_MOTION", MotionMode.stopped_mode)
        limit_cur = self._read_field("PID_TORQUE_FLUX_LIMITS")
        old_flux_offset = self._read_field("PID_FLUX_OFFSET")
        self._write_field("PHI_E_SELECTION", 1)  # external mode, so it won't change.
        self._write_field("PHI_E_EXT", 0)  # and, set this to be PHI_E = 0
        self._write_field("UD_EXT", limit_cur)
        self._write_field("UQ_EXT", 0)
        dwell(0.2)
        self._write_field("PID_TORQUE_TARGET", 0)
        self._write_field("PID_VELOCITY_TARGET", 0)
        self._write_field("PID_POSITION_TARGET", 0)
        self._write_field("PID_POSITION_ACTUAL", 0)
        self._write_field("PWM_CHOP", 7)
        self._write_field("MODE_MOTION", MotionMode.uq_ud_ext_mode)
        for i in range(5):
            dwell(0.1)
            self._write_field("PWM_CHOP", 0)
            # Give it some time to settle
            dwell(0.1)
            self._write_field("PWM_CHOP", 7)
        # Give it some time to settle
        dwell(0.4)
        if offsets:
            # While we're here, set the offsets
            self._write_field(
                "HALL_PHI_E_OFFSET", -self._read_field("HALL_PHI_E") % 65536
            ),
            self._write_field("ABN_DECODER_COUNT", 0)
            self._write_field("ABN_DECODER_PHI_E_OFFSET", 0)
        self._write_field("MODE_MOTION", MotionMode.stopped_mode)
        # Give it some time to settle
        dwell(0.2)
        self._write_field("MODE_MOTION", MotionMode.torque_mode)
        test_cur = limit_cur  # // 2
        old_cur = self._read_field("PID_%s_ACTUAL" % X)
        dwell(0.2)
        if not test_existing:
            Kc0 = Kc
            self._write_field("PID_%s_P" % X, to_q8_8(Kc0))
            self._write_field("PID_%s_I" % X, to_q8_8(0.0))
        else:
            Kc0 = from_q4_12(self._read_field("PID_%s_P" % X))
        # Do a setpoint change experiment
        self._write_field("PID_%s_TARGET" % X, 0)
        logging.info("test_cur = %d" % test_cur)
        n = 200
        c = self._dump_pid(n, X)
        self._write_field("PID_%s_TARGET" % X, test_cur)
        c += self._dump_pid(n, X)
        # Experiment over, switch off
        self._write_field("PID_%s_TARGET" % X, 0)
        # Put motion config back how it was
        self._write_field("PID_TORQUE_TARGET", 0)
        self._write_field("PID_VELOCITY_TARGET", 0)
        self._write_field("PID_POSITION_TARGET", 0)
        self._write_field("PID_POSITION_ACTUAL", 0)
        self._write_field("MODE_MOTION", old_mode)
        self._write_field("PID_FLUX_OFFSET", old_flux_offset)
        self._write_field("PHI_E_SELECTION", old_phi_e_selection)
        # Analysis and logging
        # for r in c:
        #    logging.info("%g,%s"%(float(r[0]-c[0][0])/1e9,','.join(map(str, r[1:]))))
        # At this point we can determine system model
        y0 = sum(float(a[1]) for a in c[0:n]) / float(n)
        yinf = (
            sum(float(a[1]) for a in c[3 * n // 2 : 2 * n]) / float(2 * n - 3 * n // 2)
            - y0
        )
        if yinf < 0.0:
            # Wups, turned out negative...
            yinf *= -1.0
            c = [(t, -y) for t, y in c]
        yp = -100000  # Not a possible value
        tp = 0
        for tpt, ypt in c[n + 1 : -1]:
            if ypt < yp:
                break
            yp, tp = ypt, tpt
        yp -= y0
        # It is better to overestimate tp than under.
        # tp -= float(c[n][0] + c[n+1][0])/2.0
        tp -= c[n][0]
        tp *= 1e-9  # it was in nanoseconds, we want seconds
        D = (yp - yinf) / yinf
        B = abs((test_cur - yinf) / yinf)
        A = 1.152 * D**2 - 1.607 * D + 1
        r = 2 * A / B
        logging.info("yinf = %g, yp = %g, tp = %g" % (yinf, yp, tp))
        k = 1.0 / (Kc0 * B)
        theta = tp * (0.309 + 0.209 * math.exp(-0.61 * r))
        tau1 = r * theta
        logging.info(
            "TMC 4671 %s %s PID system model k=%g, theta=%g, tau1=%g"
            % (
                self.name,
                X,
                k,
                theta,
                tau1,
            )
        )
        Kc, taui = simc(k, theta, tau1, 0.0005)
        # Account for sampling frequency
        Ki = Kc / (taui * 2 * 25e3)
        Kc *= derate
        Ki *= derate
        logging.info(
            "TMC 4671 %s %s PID coefficients Kc=%g, Ti=%g (Ki=%g)"
            % (self.name, X, Kc, taui, Ki)
        )
        if not test_existing:
            self._write_field("PID_%s_P" % X, self.pid_helpers["%s_P" % X].to_f(Kc))
            self._write_field("PID_%s_I" % X, self.pid_helpers["%s_I" % X].to_f(Ki))
        return Kc, Ki

    def _dump_pid(self, n, X):
        f = "PID_%s_ACTUAL" % X
        c = [(0, 0)] * (n)
        for i in range(n):
            cur = self._read_field(f)
            c[i] = (
                monotonic_ns(),
                cur,
            )
        return c

    def _init_registers(self, print_time=None):
        if print_time is None:
            print_time = self.printer.lookup_object("toolhead").get_last_move_time()
        ping = self.mcu_tmc.get_register("CHIPINFO_SI_TYPE")
        if ping != 0x34363731:
            raise self.printer.command_error(
                "TMC 4671 not identified, identification register returned %x" % (ping,)
            )
        # Disable 6100
        if self.fields6100 is not None:
            self.mcu_tmc6100.set_register(
                "GCONF", self.fields6100.set_field("disable", 1), print_time
            )
        self.mcu_tmc.set_register_once("STATUS_FLAGS", 0)
        # Set torque and current in 4671 to zero
        self._write_field("PID_FLUX_TARGET", 0)
        self._write_field("PID_TORQUE_TARGET", 0)
        self._write_field("PID_VELOCITY_TARGET", 0)
        self._write_field("PID_POSITION_TARGET", 0)
        self._write_field("PWM_CHOP", 7)
        # Send registers, 6100 first if configured then 4671
        if self.fields6100 is not None:
            for reg_name in list(self.fields6100.registers.keys()):
                val = self.fields6100.registers[reg_name]  # Val may change during loop
                self.mcu_tmc6100.set_register(reg_name, val, print_time)
        for reg_name in list(self.fields.registers.keys()):
            if reg_name == "STATUS_FLAGS":
                continue
            val = self.fields.registers[reg_name]  # Val may change during loop
            self.mcu_tmc.set_register(reg_name, val, print_time)
        self._calibrate_adc(print_time)
        # setup filters
        self.enable_biquad(
            "CONFIG_BIQUAD_F_ENABLE",
            *biquad_tmc(*biquad_lpf(self.pwmfreq, 4600, 2**-0.5)),
        )
        self.enable_biquad(
            "CONFIG_BIQUAD_T_ENABLE",
            *biquad_tmc(*biquad_lpf(self.pwmfreq, 4600, 2**-0.5)),
        )
        self.enable_biquad(
            "CONFIG_BIQUAD_X_ENABLE",
            *biquad_tmc(
                *biquad_lpf(
                    self.pwmfreq / (self._read_field("MODE_PID_SMPL") + 1.0),
                    200,
                    2**-0.5,
                )
            ),
        )
        self.enable_biquad(
            "CONFIG_BIQUAD_V_ENABLE",
            *biquad_tmc(*biquad_lpf(self.pwmfreq, 4600, 2**-0.5)),
        )
        # *biquad_tmc(*biquad_notch(self.pwmfreq, 195, 2**-0.5)))
        self._write_field("CONFIG_BIQUAD_F_ENABLE", 1)
        self._write_field("CONFIG_BIQUAD_T_ENABLE", 1)
        self._write_field("CONFIG_BIQUAD_V_ENABLE", 1)
        self._write_field("CONFIG_BIQUAD_X_ENABLE", 0)

    def get_status(self, eventtime=None):
        if not self.init_done:
            return {
                "run_current": 0.0,
                "current_ux": 0.0,
                "current_v": 0.0,
                "current_wy": 0.0,
                "velocity_target": 0.0,
                "position_target": 0.0,
                "velocity_actual": 0.0,
                "position_actual": 0.0,
            }
        current = self.current_helper.get_current()
        velocity_target = self._read_field("PID_VELOCITY_TARGET")
        position_target = self._read_field("PID_POSITION_TARGET")
        velocity_actual = self._read_field(f"PID_VELOCITY_ACTUAL")
        position_actual = self._read_field("PID_POSITION_ACTUAL")
        res = {
            "run_current": current[0],
            "current_ux": current[2],
            "current_v": current[3],
            "current_wy": current[4],
            "velocity_target": velocity_target,
            "position_target": position_target,
            "velocity_actual": velocity_actual,
            "position_actual": position_actual,
        }
        # res.update(self.echeck_helper.get_status(eventtime))
        return res

    cmd_INIT_TMC_help = "Initialize TMC stepper driver registers"

    def cmd_INIT_TMC(self, gcmd):
        logging.info("INIT_TMC %s", self.name)
        print_time = self.printer.lookup_object("toolhead").get_last_move_time()
        self._init_registers(print_time)

    cmd_TMC_ENC_OFFSETS_help = "Set the encoder angle offsets"

    def cmd_TMC_ENC_OFFSETS(self, gcmd):
        test_existing = gcmd.get_int("CHECK", 0)
        logging.info("TMC_ENC_OFFSETS %s", self.name)
        print_time = self.printer.lookup_object("toolhead").get_last_move_time()
        self._get_angle_offsets_bang(print_time)

    cmd_TMC_TUNE_PID_help = "Tune the current and torque PID coefficients"

    def cmd_TMC_TUNE_PID(self, gcmd):
        test_existing = gcmd.get_int("CHECK", 0)
        derate = gcmd.get_float("DERATE", 1.0)
        logging.info("TMC_TUNE_PID %s", self.name)
        print_time = self.printer.lookup_object("toolhead").get_last_move_time()
        P, I = self._tune_flux_pid(test_existing, derate, print_time)
        self._write_field("PID_TORQUE_P", self.pid_helpers["TORQUE_P"].to_f(P))
        self._write_field("PID_TORQUE_I", self.pid_helpers["TORQUE_I"].to_f(I))

    cmd_TMC_DEBUG_MOVE_help = "Test TMC motion mode (motor must be free to move)"

    def cmd_TMC_DEBUG_MOVE(self, gcmd):
        logging.info("TMC_DEBUG_MOVE %s", self.name)
        velocity = gcmd.get_int("VELOCITY", None)
        torque = gcmd.get_int("TORQUE", None)
        pos = gcmd.get_int("POSITION", None)
        open_loop_velocity = gcmd.get_int("OPENVEL", None)
        print_time = self.printer.lookup_object("toolhead").get_last_move_time()
        if velocity is not None:
            self._debug_pid_motion(
                velocity, MotionMode.velocity_mode, "PID_VELOCITY_TARGET", print_time
            )
        elif torque is not None:
            self._debug_pid_motion(
                torque, MotionMode.torque_mode, "PID_TORQUE_TARGET", print_time
            )
        elif pos is not None:
            self._debug_pid_motion(
                pos, MotionMode.position_mode, "PID_POSITION_TARGET", print_time
            )
        elif open_loop_velocity is not None:
            self._debug_openloop_velocity_motion(open_loop_velocity, print_time)

    def _debug_pid_motion(self, velocity, mode, target_reg, print_time):
        # self._write_field("PWM_CHOP", 7)
        old_mode = self._read_field("MODE_MOTION")
        self._write_field("PID_POSITION_ACTUAL", 0)
        self.printer.lookup_object("toolhead").dwell(0.2)
        # Clear all the status flags for later reference
        reg, _ = self.mcu_tmc.name_to_reg["STATUS_FLAGS"]
        self.mcu_tmc.tmc_spi.reg_write(reg, 0, print_time)
        limit_cur = self._read_field("PID_TORQUE_FLUX_LIMITS")
        self._write_field("PID_FLUX_TARGET", 0)
        self._write_field("PID_TORQUE_TARGET", 0)
        self._write_field(target_reg, 0)
        self._write_field("MODE_MOTION", mode)
        n2 = 20
        if mode == MotionMode.stopped_mode:
            v = [velocity * i // (2 * n2) for i in range(n2, 2 * n2)] + [
                velocity for i in range(n2 * 2)
            ]
        else:
            v = [velocity for i in range(n2, 2 * n2)] + [
                velocity for i in range(n2 * 2)
            ]
        n = 500
        c = self._dump_motion(n, f=lambda x: self._write_field(target_reg, x), v=v)
        self._write_field("MODE_MOTION", old_mode)
        self._write_field(target_reg, 0)
        # self._write_field("PWM_CHOP", 0)
        for i in range(n):
            logging.info(",".join(map(str, c[i])))

    def _dump_motion(self, n, f=None, v=None):
        n2 = n
        if v is not None:
            n2 = len(v) + 1
        iv = 0
        c = [(0, 0)] * (n)
        for i in range(n):
            if f is not None and i % n2 == 0:
                f(v[i // n2])
            c[i] = (
                monotonic_ns() / 1e9,
                self._read_field("PID_POSITION_ACTUAL"),
                self._read_field("PID_VELOCITY_ACTUAL"),
                self._read_field("PID_ERROR_PID_VELOCITY_ERROR"),
                self._read_field("INTERIM_PIDIN_TARGET_TORQUE"),
                self._read_field("PID_TORQUE_ACTUAL"),
                self._read_field("PID_ERROR_PID_TORQUE_ERROR"),
                self._read_field("PID_ERROR_PID_TORQUE_ERROR_SUM"),
                self._read_field("INTERIM_PIDIN_TARGET_FLUX"),
                self._read_field("PID_FLUX_ACTUAL"),
                self._read_field("PID_ERROR_PID_FLUX_ERROR"),
                self._read_field("PID_ERROR_PID_FLUX_ERROR_SUM"),
                ":::",
                self._read_field("ADC_I0_RAW"),
                self._read_field("ADC_I1_RAW"),
                self._read_field("ADC_IUX"),
                self._read_field("INTERIM_PWM_UX"),
                self._read_field("ADC_IV"),
                self._read_field("INTERIM_PWM_UV"),
                self._read_field("ADC_IWY"),
                self._read_field("INTERIM_PWM_WY"),
                ":::",
                self._read_field("ABN_DECODER_PHI_M") % 65536,
                self._read_field("ABN_DECODER_PHI_E") % 65536,
                self._read_field("HALL_PHI_M") % 65536,
                self._read_field("HALL_PHI_E") % 65536,
                self._read_field("PHI_E") % 65536,
                (self._read_field("PHI_E") - self._read_field("HALL_PHI_E")) % 65536,
                (self._read_field("PHI_E") - self._read_field("ABN_DECODER_PHI_E"))
                % 65536,
            )
            self.printer.lookup_object("toolhead").dwell(15.0 / n)
        return c

    def _debug_openloop_velocity_motion(self, velocity, print_time):
        self._write_field("PWM_CHOP", 7)
        self._write_field("MODE_MOTION", MotionMode.uq_ud_ext_mode)
        limit_cur = self._read_field("PID_TORQUE_FLUX_LIMITS")
        self._write_field("UD_EXT", limit_cur * 10)  # 10 ohm servo
        self._write_field("UQ_EXT", 0)
        old_phi_e_sel = self._read_field("PHI_E_SELECTION")
        self._write_field("PHI_E_SELECTION", 2)
        phi_e = self._read_field("PHI_E")
        # Clear all the status flags for later reference
        reg, _ = self.mcu_tmc.name_to_reg["STATUS_FLAGS"]
        self.mcu_tmc.tmc_spi.reg_write(reg, 0, print_time)
        # might not stick, so write with a one-shot
        reg, _ = self.mcu_tmc.name_to_reg["OPENLOOP_PHI"]
        n = 50000
        self._write_field("OPENLOOP_VELOCITY_TARGET", velocity)
        self._write_field("OPENLOOP_ACCELERATION", 1000)
        c = self._dump_motion(n)
        self._write_field("OPENLOOP_VELOCITY_TARGET", 0)
        self._write_field("OPENLOOP_ACCELERATION", 0)
        self.printer.lookup_object("toolhead").dwell(0.2)
        self._write_field("MODE_MOTION", MotionMode.stopped_mode)
        self._write_field("PHI_E_SELECTION", old_phi_e_sel)
        self._write_field("PWM_CHOP", 0)
        for i in range(n):
            logging.info(",".join(map(str, c[i])))

    cmd_DUMP_TMC6100_help = "Read and display TMC6100 stepper driver registers"

    def cmd_DUMP_TMC6100(self, gcmd):
        logging.info("DUMP_TMC6100 %s", self.name)
        field_name = gcmd.get("FIELD", None)
        if field_name is not None:
            reg_name = self.fields6100.lookup_register(field_name.upper())
            if reg_name is None:
                reg_name = field_name
        else:
            reg_name = gcmd.get("REGISTER", None)
        if reg_name is not None:
            reg_name = reg_name.upper()
            if reg_name in self.read_registers:
                # readable register
                val = self.mcu_tmc6100.get_register(reg_name)
                if self.read_translate is not None:
                    reg_name, val = self.read_translate(reg_name, val)
                gcmd.respond_info(self.fields6100.pretty_format(reg_name, val))
            else:
                raise gcmd.error("Unknown register name '%s'" % (reg_name))
        else:
            group = gcmd.get("GROUP", "Default")
            if group not in DumpGroups6100:
                raise gcmd.error("Unknown group name '%s'" % (group))
            gcmd.respond_info("========== Queried registers ==========")
            for reg_name in DumpGroups6100[group]:
                val = self.mcu_tmc6100.get_register(reg_name)
                if self.read_translate is not None:
                    reg_name, val = self.read_translate(reg_name, val)
                gcmd.respond_info(self.fields6100.pretty_format(reg_name, val))

    cmd_DUMP_TMC_help = "Read and display TMC stepper driver registers"

    def cmd_DUMP_TMC(self, gcmd):
        logging.info("DUMP_TMC %s", self.name)
        field_name = gcmd.get("FIELD", None)
        if field_name is not None:
            reg_name = self.fields.lookup_register(field_name.upper())
            if reg_name is None:
                reg_name = field_name
        else:
            reg_name = gcmd.get("REGISTER", None)
        if reg_name is not None:
            reg_name = reg_name.upper()
            if reg_name in self.read_registers:
                # readable register
                val = self.mcu_tmc.get_register(reg_name)
                if self.read_translate is not None:
                    reg_name, val = self.read_translate(reg_name, val)
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            else:
                raise gcmd.error("Unknown register name '%s'" % (reg_name))
        else:
            group = gcmd.get("GROUP", "Default")
            if group not in DumpGroups:
                raise gcmd.error("Unknown group name '%s'" % (group))
            gcmd.respond_info("========== Queried registers ==========")
            for reg_name in DumpGroups[group]:
                val = self.mcu_tmc.get_register(reg_name)
                if self.read_translate is not None:
                    reg_name, val = self.read_translate(reg_name, val)
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))

    cmd_SET_TMC_FIELD_help = "Set a register field of a TMC driver"

    def cmd_SET_TMC_FIELD(self, gcmd):
        field_name = gcmd.get("FIELD").upper()
        reg_name = self.fields.lookup_register(field_name, None)
        if reg_name is None:
            raise gcmd.error("Unknown field name '%s'" % (field_name,))
        value = gcmd.get_int("VALUE", None)
        # velocity = gcmd.get_float('VELOCITY', None, minval=0.)
        if value is None:
            raise gcmd.error("Specify VALUE")
        # if velocity is not None:
        #     if self.mcu_tmc.get_tmc_frequency() is None:
        #         raise gcmd.error(
        #             "VELOCITY parameter not supported by this driver")
        #     value = TMCtstepHelper(self.mcu_tmc, velocity,
        #                            pstepper=self.stepper)
        reg_val = self.fields.set_field(field_name, value)
        print_time = self.printer.lookup_object("toolhead").get_last_move_time()
        self.mcu_tmc.set_register(reg_name, reg_val, print_time)

    cmd_SET_TMC_CURRENT_help = "Set the current of a TMC driver"

    def cmd_SET_TMC_CURRENT(self, gcmd):
        ch = self.current_helper
        prev_cur, max_cur = ch.get_current()
        run_current = gcmd.get_float("CURRENT", None, minval=0.0, maxval=max_cur)
        if run_current is not None:
            reg_val = ch.set_current(run_current)
            prev_cur, max_cur = ch.get_current()
            print_time = self.printer.lookup_object("toolhead").get_last_move_time()
            self.mcu_tmc.set_register("PID_TORQUE_FLUX_LIMITS", reg_val, print_time)
        gcmd.respond_info("Run Current: %0.2fA" % (prev_cur,))


def load_config_prefix(config):
    return TMC4671(config)
