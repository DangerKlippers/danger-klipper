from . import pulse_counter
from typing import TYPE_CHECKING
from simple_pid import PID

if TYPE_CHECKING:
    from ..toolhead import ToolHead, Move
    from ..configfile import ConfigWrapper
    from ..klippy import Printer
    from ..gcode import GCodeDispatch


class PowerCore:
    def __init__(self, config: ConfigWrapper):
        self._pwm_reader = PowerCorePWMReader(config)
        self.printer: Printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.toolhead: ToolHead = self.printer.lookup_object("toolhead")
        self.gcode: GCodeDispatch = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "GET_DUTY_CYCLE",
            self.cmd_get_duty_cycle,
            desc="Get the current duty cycle",
        )
        self.gcode.register_command(
            "ENABLE_POWERCORE_FEED_SCALING",
            self.cmd_enable_scaling,
            desc="Enable scaling",
        )
        self.gcode.register_command(
            "DISABLE_POWERCORE_FEED_SCALING",
            self.cmd_disable_scaling,
            desc="Disable scaling",
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
        self.scaling_enabled = True
        self.pid_controller = PID(
            Kp=config.getfloat("kp", 1.0),
            Ki=config.getfloat("ki", 0.0),
            Kd=config.getfloat("kd", 0.0),
            setpoint=self.target_duty_cycle,
            output_limits=(0, 1),
            sample_time=None,
            time_fn=self.reactor.monotonic(),
        )

    def cmd_get_duty_cycle(self, gcmd):
        duty_cycle = self._pwm_reader.get_current_duty_cycle()
        gcmd.respond_info(f"duty_cycle: {duty_cycle}")

    def cmd_enable_scaling(self, gcmd):
        self.enable_scaling()
        gcmd.respond_ok()

    def cmd_disable_scaling(self, gcmd):
        self.disable_scaling()
        gcmd.respond_ok()

    def enable_scaling(self):
        self.pid_controller.reset()
        self.scaling_enabled = True

    def disable_scaling(self):
        self.scaling_enabled = False

    def check_move(self, move: Move):
        if not self.scaling_enabled:
            return
        else:
            self.scale_move(move)

    def scale_move(self, move: Move):
        current_duty_cycle = self._pwm_reader.get_current_duty_cycle()
        output = self.pid_controller(current_duty_cycle)
        # output it 0-1, scale it to min_feedrate-max_feedrate
        feedrate = self.min_feedrate + output * (
            self.max_feedrate - self.min_feedrate
        )
        # feedrate is in mm/min, set_speed expects mm/sec
        move.set_speed(feedrate * 60, self.adjustment_accel)
        self.gcode.respond_info(
            f"Current duty cycle: {current_duty_cycle}, output: {output}, feedrate: {feedrate}"
        )


class PowerCorePWMReader:
    def __init__(self, config):
        printer = config.get_printer()
        self._pwm_counter = None

        pin = config.get("alrt_pin")
        poll_time = config.getfloat("alrt_poll_interval", 0.0015, above=0.0)
        pwm_frequency = config.getfloat("pwm_frequency", 100.0, above=0.0)
        sample_time = config.getfloat("alrt_sample_time", 0.1, above=0.0)
        self._pwm_counter = pulse_counter.PWMCounter(
            printer, pin, sample_time, poll_time, pwm_frequency
        )

    def get_current_duty_cycle(self, eventtime):
        return self._pwm_counter.get_duty_cycle()


def load_config_prefix(config):
    return PowerCore(config)
