from .homing import Homing
from .homing import HomingMove
from gcode import GCodeDispatch
from toolhead import ToolHead
from kinematics.corexy import CoreXYKinematics


class EnvelopeAutoDetect:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode: GCodeDispatch = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "AUTODETECT_ENVELOPE",
            self.cmd_AUTODETECT_ENVELOPE,
            desc="Autodetect envelope",
        )
        self.toolhead: ToolHead = self.printer.lookup_object("toolhead")
        self.kin: CoreXYKinematics = (
            self.toolhead.get_kinematics()
        )  # placeholder type for easy dev

    def cmd_AUTODETECT_ENVELOPE(self):
        self.do_autodetect()

    def do_autodetect(self):
        self.gcode.respond_info("Autodetecting envelope")

    def do_autodetect_x(self):
        self.gcode.respond_info("Autodetecting envelope x")
        axis = 0
        rail = self.kin.rails[axis]
        # homing to the left first.
        self.toolhead.set_position([0, 0, 0], homing_axes=[axis])

        endstops = rail.get_endstops()
        homing_state = Homing(self.printer)
        hmove = HomingMove(self.printer, endstops)
        homing_state._set_current_homing([axis], pre_homing=True)
        homepos = [self.kin.axes_max[axis], 0, 0]
        hi = rail.get_homing_info()
        hmove.homing_move(homepos, hi.speed)
        dist_from_start_to_endstop = hmove.distance_elapsed
