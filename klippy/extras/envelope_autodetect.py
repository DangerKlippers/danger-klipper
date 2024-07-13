from enum import Enum
from .homing import Homing
from .homing import HomingMove
from gcode import GCodeDispatch
from toolhead import ToolHead
from kinematics.corexy import CoreXYKinematics


class Direction(Enum):
    X_MIN = 1
    X_MAX = 2
    Y_MIN = 3
    Y_MAX = 4


class EnvelopeAutoDetect:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler(
            "klippy:connect", self.handle_connect
        )
        self.gcode: GCodeDispatch = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "AUTODETECT_ENVELOPE",
            self.cmd_AUTODETECT_ENVELOPE,
            desc="Autodetect envelope",
        )
        self.toolhead: ToolHead = None
        self.kin: CoreXYKinematics = None  # placeholder type for easy dev
        self.homing_state = None

    def handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        self.kin: CoreXYKinematics = (
            self.toolhead.get_kinematics()
        )  # placeholder type for easy dev
        self.homing_state = Homing(self.printer)

    def cmd_AUTODETECT_ENVELOPE(self, gcmd):
        self.do_autodetect()

    def do_autodetect(self):
        self.gcode.respond_info("Autodetecting envelope")
        x_length = self.do_autodetect_x()

    def do_autodetect_x(self):
        self.gcode.respond_info("Autodetecting envelope x")
        self.do_single_home("x", True, set_current=True, revert_current=False)
        x_length = self.do_single_home(
            "x", False, set_current=False, revert_current=True
        )
        return x_length

    def do_single_home(
        self,
        axis: str,
        positive: bool,
        set_current: bool = True,
        revert_current: bool = False,
    ):
        axis_index = "xyz".index(axis.lower())
        max_pos = [0, 0, 0]
        max_pos[axis_index] = self.kin.axes_max[axis_index]
        min_pos = [0, 0, 0]
        min_pos[axis_index] = self.kin.axes_min[axis_index]
        if positive:
            start_pos = min_pos
            home_pos = max_pos
        else:
            start_pos = max_pos
            home_pos = min_pos
        rail = self.kin.rails[axis_index]
        hi = rail.get_homing_info()
        self.toolhead.set_position(start_pos, homing_axes=[axis_index])
        endstops = rail.get_endstops()
        hmove = HomingMove(self.printer, endstops)
        if set_current:
            self.homing_state._set_current_homing([axis_index], pre_homing=True)
        hmove.homing_move(home_pos, hi.speed)
        dist_from_start_to_endstop = hmove.distance_elapsed
        self.gcode.respond_info(
            f"axis: {axis}, positive: {positive}, dist: {dist_from_start_to_endstop}"
        )
        print_time = self.toolhead.get_last_move_time()
        if revert_current:
            self.homing_state._set_current_homing(
                [axis_index], pre_homing=False
            )
        for endstop in endstops:
            # re-querying a tmc endstop seems to reset the state
            # otherwise it triggers almost immediately upon second home
            # this seems to be an adequate substitute for a 2 second dwell.
            endstop[0].query_endstop(print_time)
        return dist_from_start_to_endstop


def load_config(config):
    return EnvelopeAutoDetect(config)
