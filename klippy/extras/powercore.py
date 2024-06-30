from . import pwm_in
import math
from typing import TYPE_CHECKING
from simple_pid import PID
import logging

if TYPE_CHECKING:
    from ..toolhead import ToolHead, Move
    from ..configfile import ConfigWrapper
    from ..klippy import Printer
    from ..gcode import GCodeDispatch
    from ..reactor import SelectReactor as Reactor


class PowerCore:
    def __init__(self, config: "ConfigWrapper"):
        self._pwm_reader = PowerCorePWMReader(config)
        self._pwm_reader.setup_callback(self.pwm_in_callback)
        self.printer: "Printer" = config.get_printer()
        self.reactor: "Reactor" = self.printer.get_reactor()
        self.toolhead: "ToolHead" = None
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )
        self.gcode: "GCodeDispatch" = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "GET_DUTY_CYCLE",
            self.cmd_get_duty_cycle,
            desc="Get the current duty cycle",
        )
        self.gcode.register_command(
            "ENABLE_POWERCORE_FEED_SCALING",
            self.cmd_enable_scaling,
            desc="Enable powercore feedrate scaling",
        )
        self.gcode.register_command(
            "DISABLE_POWERCORE_FEED_SCALING",
            self.cmd_disable_scaling,
            desc="Disable powercore feedrate scaling",
        )
        self.gcode.register_command(
            "SET_POWERCORE_PID", 
            self.cmd_set_pid_params, 
            desc="Set powercore pid params"
        )
        self.gcode.register_command(
            "RESET_POWERCORE_PID",
            self.cmd_reset_pid,
            desc="resets pid controller"
        )
        self.gcode.register_command(
            "SET_POWERCORE_TARGET_DUTY_CYCLE",
            self.cmd_set_target_duty_cycle,
            desc="set the target powercore target duty cycle"
        )
        self.target_duty_cycle: float = config.getfloat(
            "target_duty_cycle", 0.75, minval=0.0, maxval=1.0
        )
        self.min_feedrate: float = config.getfloat(
            "min_feedrate", 0.1, minval=0.0
        )  # mm/min
        self.max_feedrate: float = config.getfloat(
            "max_feedrate", 64.0, minval=0.0
        )  # mm/min
        self.adjustment_accel = config.getfloat(
            "powercore_adjustment_accel", 500.0, above=0.0
        )
        self.verbose_pid_output = config.getboolean(
            "verbose_pid_output", False
        )
        self.scaling_enabled = False
        self.pid_controller = PID(
            Kp=config.getfloat("pid_kp", 0.1),
            Ki=config.getfloat("pid_ki", 0.7),
            Kd=config.getfloat("pid_kd", 0.0),
            setpoint=self.target_duty_cycle,
            output_limits=(0, 1),
            sample_time=self._pwm_reader.sample_interval,
            time_fn=self.reactor.monotonic,
        )

    def pwm_in_callback(self, duty_cycle):
        if not self.scaling_enabled:
            return
        self.output = self.pid_controller(duty_cycle)
        if self.verbose_pid_output:
            self.gcode.respond_info(f"PowerCore PID-loop output: {self.output}")

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")

    def cmd_reset_pid(self, gcmd):
        self.pid_controller.reset()
        gcmd.respond_info("Reset powercore PID")

    def cmd_set_target_duty_cycle(self, gcmd):
        target_duty_cycle = gcmd.get_float("DUTY_CYCLE", self.target_duty_cycle)
        self.target_duty_cycle = target_duty_cycle
        self.pid_controller.setpoint = target_duty_cycle
        gcmd.respond_info(f"Target duty cycle: {target_duty_cycle}")
        
    def cmd_set_pid_params(self, gcmd):
        kp = gcmd.get_float("KP", self.pid_controller.Kp)
        ki = gcmd.get_float("KI", self.pid_controller.Ki)
        kd = gcmd.get_float("KD", self.pid_controller.Kd)
        self.pid_controller.tunings = (kp, ki, kd)
        gcmd.respond_info(f"PID Params: Kp: {kp}, Ki: {ki}, Kd: {kd}")

    def cmd_get_duty_cycle(self, gcmd):
        duty_cycle = self._pwm_reader.get_current_duty_cycle()
        gcmd.respond_info(f"duty_cycle: {duty_cycle}")

    def cmd_enable_scaling(self, gcmd):
        self.enable_scaling()
        gcmd.respond_info("Enabled powercore move scaling")

    def cmd_disable_scaling(self, gcmd):
        self.disable_scaling()
        gcmd.respond_info("Disabled powercore move scaling")


    def enable_scaling(self):
        self.pid_controller.reset()
        self.scaling_enabled = True

    def disable_scaling(self):
        self.scaling_enabled = False

    def check_move(self, move: "Move"):
        
        if not self.scaling_enabled:
            return
        else:
            self.scale_move(move)

    def scale_move(self, move: "Move"):
        logging.info("scale_move")
        current_duty_cycle = self._pwm_reader.get_current_duty_cycle()
        # output = self.pid_controller(current_duty_cycle)
        output = self.output
        # output it 0-1, scale it to min_feedrate-max_feedrate
        feedrate = self.min_feedrate + output * (
            self.max_feedrate - self.min_feedrate
        )
        logging.info(f"orig move feedrate: {math.sqrt(move.max_cruise_v2)}")
        logging.info(f"new move feedrate: {feedrate}")
        logging.info(f"current duty cycle: {current_duty_cycle}")
        logging.info(f"pid output: {output}")
        # feedrate is in mm/min, set_speed expects mm/sec
        move.set_speed(feedrate * 60, self.adjustment_accel)
        self.gcode.respond_info(
            f"Current duty cycle: {current_duty_cycle}, output: {output}, feedrate: {feedrate}"
        )


class PowerCorePWMReader:
    def __init__(self, config):
        printer = config.get_printer()
        self._pwm_in = None

        pin = config.get("alrt_pin")
        pwm_frequency = config.getfloat("alrt_pwm_frequency", 100.0, above=0.0)
        additional_timeout_ticks = config.getint("additional_timeout_ticks", 0, minval=0)
        self.sample_interval = config.getfloat(
            "alrt_sample_interval", 0.1, above=0.1
        )
        self._pwm_in = pwm_in.PWMIn(
            printer, pin, self.sample_interval, pwm_frequency, additional_timeout_ticks
        )
    def setup_callback(self, cb):
        self._pwm_in.setup_callback(cb)
        

    def get_current_duty_cycle(self):
        return round(self._pwm_in.get_duty_cycle(), 3)


def load_config(config):
    return PowerCore(config)
