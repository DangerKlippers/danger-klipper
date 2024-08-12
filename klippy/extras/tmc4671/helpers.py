import logging, collections
from .register_bank import *
from .. import bus
from typing import Union


# Return the position of the first bit set in a mask
def ffs(mask):
    return (mask & -mask).bit_length() - 1


######################################################################
# Field manipulation helpers
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
    def __init__(self, config, registers, field_helper, tmc_frequency, pin_option):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.tmc_spi = MCU_TMC_SPI_simple(config, pin_option=pin_option)
        self.mutex = self.tmc_spi.mutex
        self.registers: Union[Registers6100, Registers4671] = registers
        self.field_helper: FieldHelper = field_helper
        self.tmc_frequency = tmc_frequency

    def get_field_helper(self):
        return self.field_helper

    def get_register(self, reg_name):
        reg, addr = self.registers[reg_name].value
        if addr is not None:
            for retry in range(5):
                v = self.tmc_spi.reg_write(reg + 1, addr)
                if v == addr:
                    break
            else:
                raise self.printer.command_error(
                    f"Unable to write tmc spi '{self.name}' address register {reg_name} (last read {v})"
                )
        read = self.tmc_spi.reg_read(reg)
        return read

    def set_register_once(self, reg_name, val, print_time=None):
        reg, addr = self.registers[reg_name]
        if addr is not None:
            v = self.tmc_spi.reg_write(reg + 1, addr, print_time)
            if v != addr:
                raise self.printer.command_error(
                    f"Unable to write tmc spi '{self.name}' address register {reg_name} (last read {v})"
                )
        v = self.tmc_spi.reg_write(reg, val, print_time)

    def set_register(self, reg_name, val, print_time=None):
        reg, addr = self.registers[reg_name]
        if addr is not None:
            for retry in range(5):
                v = self.tmc_spi.reg_write(reg + 1, addr, print_time)
                if v == addr:
                    break
            else:
                raise self.printer.command_error(
                    f"Unable to write tmc spi '{self.name}' address register {reg_name} (last read {v})"
                )
        for retry in range(5):
            v = self.tmc_spi.reg_write(reg, val, print_time)
            if v == val:
                return
        raise self.printer.command_error(
            f"Unable to write tmc spi '{self.name}' register {reg_name} (last read {v})"
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


class PIDHelper:
    def __init__(self, config, mcu_tmc: MCU_TMC_SPI, var, def_v, nvar, def_n):
        self.mcu_tmc = mcu_tmc
        self.field_helper = mcu_tmc.get_field_helper()
        fvar = "PID_" + var
        logging.info(f'TMC: {",".join((str(i) for i in [var, def_v, nvar, fvar]))}')
        set_config_field = self.field_helper.set_config_field
        set_config_field(config, nvar, def_n)
        if nvar in FieldParsers:
            self.to_f = FieldParsers[nvar]
            self.from_f = FieldFormatters[nvar]
        else:
            if self.field_helper.get_field(nvar):
                self.to_f = to_q4_12
                self.from_f = from_q4_12
            else:
                self.to_f = to_q8_8
                self.from_f = from_q8_8

            FieldFormatters[fvar] = self.from_f

        set_config_field(config, fvar, self.to_f(def_v))


class FieldHelper:
    def __init__(
        self,
        field_enum: FieldEnum,
        register_enum: RegisterEnum,
        printer,
        signed_fields=[],
        field_formatters={},
        prefix="driver_",
    ):
        self.field_enum = field_enum
        self.register_enum = register_enum
        self.register_cache = collections.OrderedDict()
        self.printer = printer
        self.reg_to_field: dict[str, list[FieldEnum]] = {
            reg.name: [
                field for field in field_enum.members() if field.value.register == reg
            ]
            for reg in register_enum.members()
        }
        # self.all_fields = all_fields
        # self.signed_fields = {sf: 1 for sf in signed_fields}
        # self.field_formatters = field_formatters
        # self.registers = registers
        # if self.registers is None:
        #     self.registers = collections.OrderedDict()
        # self.field_to_register = {
        #     f: r for r, fields in self.all_fields.items() for f in fields
        # }
        self.prefix = prefix

    def name_is_register(self, name):
        return name in self.register_enum.member_names()

    def name_is_field(self, name):
        return name in self.field_enum.member_names()

    def lookup_register_info_from_name(self, name, default=None) -> RegisterEnum:
        if self.name_is_register(name):
            # name is a register, return it
            return self.register_enum.get(name)
        # name is a field, return its register
        field = self.field_enum.get(name, default)
        if field is not None:
            return field.value.register
        return None

    def lookup_field_info_from_name(self, name, default=None):
        field = self.field_enum.get(name, default)
        if field is not None:
            return field.value
        return None

    def get_field(self, field_name, reg_value=None, reg_name=None):
        # Returns value of the register field
        if reg_name is None:
            register = self.lookup_register_info_from_name(field_name)
            reg_name = register.name
        else:
            register = self.lookup_register_info_from_name(reg_name)

        if reg_value is None:
            if reg_name not in self.register_cache:
                logging.info(f"no value for {reg_name} in cache, using 0...")
                reg_value = 0
            else:
                reg_value = self.register_cache.get(reg_name)

        if self.name_is_register(field_name):
            mask = 0xFFFFFFFF
            signed = register.value.signed
        else:
            field = self.lookup_field_info_from_name(field_name)
            mask = field.mask
            signed = field.signed

        field_value = (reg_value & mask) >> ffs(mask)

        if signed and ((reg_value & mask) << 1) > mask:
            field_value -= 1 << field_value.bit_length()
        return field_value

    def set_field(self, field_name, field_value, reg_value=None, reg_name=None):
        # Returns register value with field bits filled with supplied value
        if reg_name is None:
            reg_name = self.lookup_register_info_from_name(field_name).name
        if reg_value is None:
            if reg_name not in self.register_cache:
                logging.info(f"no value for {reg_name} in cache, using 0...")
                reg_value = 0
            else:
                reg_value = self.register_cache.get(reg_name)

        if self.name_is_register(field_name):
            mask = 0xFFFFFFFF
        else:
            mask = self.lookup_field_info_from_name(field_name).mask

        new_value = (reg_value & ~mask) | ((field_value << ffs(mask)) & mask)
        self.register_cache[reg_name] = new_value
        return new_value

    def set_config_field(self, config, field_name, default):
        # Allow a field to be set from the config file
        config_name = self.prefix + field_name
        register = self.lookup_register_info_from_name(field_name)
        if register is None:
            raise self.printer.config_error(f"invalid field name: '{field_name}'")
        reg_name = register.name
        if self.name_is_register(field_name):
            mask = 0xFFFFFFFF
            signed = register.value.signed
            parser = FieldParsers.get(register)
        else:
            field = self.lookup_field_info_from_name(field_name)
            mask = field.mask
            signed = field.signed
            parser = FieldParsers.get(field)

        maxval = mask >> ffs(mask)
        if maxval == 1:
            val = config.getboolean(config_name, default)
        elif signed:
            config_maxval = maxval // 2
            config_minval = -(config_maxval + 1)
            val = config.getfloat(
                config_name, float(default), minval=config_minval, maxval=config_maxval
            )
        else:
            val = config.getfloat(config_name, float(default), minval=0, maxval=maxval)

        if isinstance(val, float):
            if parser is not None:
                val = parser(val)
            else:
                val = int(val)

        return self.set_field(field_name, val, reg_name=reg_name)

    def pretty_format(self, reg_name, reg_value):
        # Provide a string description of a register
        # reg_fields = self.all_fields.get(reg_name, {reg_name: 0xFFFFFFFF})
        reg_fields = self.reg_to_field.get(reg_name, {})
        # reg_fields = sorted([(mask, name) for name, mask in reg_fields.items()])
        field_names = [field.name for field in reg_fields]
        fields = []
        for field_name in field_names:
            field_value = self.get_field(field_name, reg_value, reg_name)
            sval = FieldFormatters.get(field_name, str)(field_value)
            if sval and sval != "0":
                fields.append(" %s=%s" % (field_name.lower(), sval))
        return "%-11s %08x%s" % (reg_name + ":", reg_value, "".join(fields))

    def get_reg_fields(self, reg_name, reg_value) -> dict[str, FieldEnum]:
        # Provide fields found in a register
        reg_fields = self.reg_to_field.get(reg_name, {})
        return {
            field.name: self.get_field(field.name, reg_value, reg_name)
            for field in reg_fields
        }


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


# parsers must return an integer
FieldParsers = {
    Fields4671.PID_FLUX_P: to_q4_12,
    Fields4671.PID_FLUX_I: to_q4_12,
    Fields4671.PID_TORQUE_P: to_q4_12,
    Fields4671.PID_TORQUE_I: to_q4_12,
    Fields4671.PID_VELOCITY_P: to_q4_12,
    Fields4671.PID_VELOCITY_I: to_q4_12,
    Fields4671.PID_POSITION_P: to_q4_12,
    Fields4671.PID_POSITION_I: to_q4_12,
}
FieldFormatters = {
    Registers4671.CONFIG_BIQUAD_X_A_1: format_q3_29,
    Registers4671.CONFIG_BIQUAD_X_A_2: format_q3_29,
    Registers4671.CONFIG_BIQUAD_X_B_0: format_q3_29,
    Registers4671.CONFIG_BIQUAD_X_B_1: format_q3_29,
    Registers4671.CONFIG_BIQUAD_X_B_2: format_q3_29,
    Registers4671.CONFIG_BIQUAD_V_A_1: format_q3_29,
    Registers4671.CONFIG_BIQUAD_V_A_2: format_q3_29,
    Registers4671.CONFIG_BIQUAD_V_B_0: format_q3_29,
    Registers4671.CONFIG_BIQUAD_V_B_1: format_q3_29,
    Registers4671.CONFIG_BIQUAD_V_B_2: format_q3_29,
    Registers4671.CONFIG_BIQUAD_T_A_1: format_q3_29,
    Registers4671.CONFIG_BIQUAD_T_A_2: format_q3_29,
    Registers4671.CONFIG_BIQUAD_T_B_0: format_q3_29,
    Registers4671.CONFIG_BIQUAD_T_B_1: format_q3_29,
    Registers4671.CONFIG_BIQUAD_T_B_2: format_q3_29,
    Registers4671.CONFIG_BIQUAD_F_A_1: format_q3_29,
    Registers4671.CONFIG_BIQUAD_F_A_2: format_q3_29,
    Registers4671.CONFIG_BIQUAD_F_B_0: format_q3_29,
    Registers4671.CONFIG_BIQUAD_F_B_1: format_q3_29,
    Registers4671.CONFIG_BIQUAD_F_B_2: format_q3_29,
    Registers4671.HALL_PHI_E_PHI_M_OFFSET: format_phi,
    Registers4671.HALL_PHI_M: format_phi,
    Registers4671.PHI_E: format_phi,
    Fields4671.HALL_POSITION_000: format_phi,
    Fields4671.HALL_POSITION_120: format_phi,
    Fields4671.HALL_POSITION_240: format_phi,
    Fields4671.HALL_POSITION_060: format_phi,
    Fields4671.HALL_POSITION_180: format_phi,
    Fields4671.HALL_POSITION_300: format_phi,
    Fields4671.HALL_PHI_E: format_phi,
    Fields4671.HALL_PHI_E_INTERPOLATED: format_phi,
    Fields4671.HALL_PHI_E_OFFSET: format_phi,
    Fields4671.HALL_PHI_M_OFFSET: format_phi,
    Fields4671.ABN_DECODER_PHI_M: format_phi,
    Fields4671.ABN_DECODER_PHI_E: format_phi,
    Fields4671.ABN_DECODER_PHI_E_OFFSET: format_phi,
    Fields4671.ABN_DECODER_PHI_M_OFFSET: format_phi,
    Fields4671.PID_FLUX_P: format_q4_12,
    Fields4671.PID_FLUX_I: format_q4_12,
    Fields4671.PID_TORQUE_P: format_q4_12,
    Fields4671.PID_TORQUE_I: format_q4_12,
    Fields4671.PID_VELOCITY_P: format_q4_12,
    Fields4671.PID_VELOCITY_I: format_q4_12,
    Fields4671.PID_POSITION_P: format_q4_12,
    Fields4671.PID_POSITION_I: format_q4_12,
}
