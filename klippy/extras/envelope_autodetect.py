import math
import statistics
import logging
from .homing import Homing
from .homing import HomingMove
from gcode import GCodeDispatch
from toolhead import ToolHead
from kinematics.corexy import CoreXYKinematics


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
        self.gcode.register_command("DETECT_RACKING", self.cmd_DETECT_RACKING, desc="detect racking")
        self.speed = config.getfloat("speed", 100.0, above=0.0)
        self.iterations = config.getint("iterations", 4, minval=1)
        self.clearance = config.getfloat("clearance", 20, above=0.0)
        self.toolhead: ToolHead = None
        self.kin: CoreXYKinematics = None  # placeholder type for easy dev
        self.homing_state = None

    def handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        self.kin: CoreXYKinematics = (
            self.toolhead.get_kinematics()
        )  # placeholder type for easy dev
        self.homing_state = Homing(self.printer)
        self.homing_infos = {'xyz'[i]: rail.get_homing_info() for i, rail in enumerate(self.kin.rails)}

    def cmd_AUTODETECT_ENVELOPE(self, gcmd):
        gcmd.respond_info("Autodetecting envelope")
        x_length = self.do_autodetect_travel_for_axis('x')
        self.move_to_middle_of_axis('x')
        gcmd.respond_info(f"x travel: {x_length}")
        y_length = self.do_autodetect_travel_for_axis('y')
        self.move_to_middle_of_axis('y')
        gcmd.respond_info(f"y travel: {y_length}")

    def cmd_DETECT_RACKING(self, gcmd):
        self.do_single_home(
            "y", positive=True
        )
        self.relative_move(
            "y", -self.clearance
        )
        y_max_x_travel = self.do_autodetect_travel_for_axis('x')
        self.move_to_middle_of_axis('x')
        self.do_single_home("y", positive=False)
        self.relative_move("y", self.clearance)
        y_min_x_travel = self.do_autodetect_travel_for_axis('x')
        self.move_to_middle_of_axis('x')
        self.do_single_home("x", positive=True)
        self.relative_move("x", -self.clearance)
        x_max_y_travel = self.do_autodetect_travel_for_axis("y")
        self.move_to_middle_of_axis('y')
        self.do_single_home("x", positive=False)
        self.relative_move("x", self.clearance)
        x_min_y_travel = self.do_autodetect_travel_for_axis("y")
        self.move_to_middle_of_axis('y')
        self.move_to_middle_of_axis('x')
        gcmd.respond_info(f"racking raw data:")
        gcmd.respond_info(f"x_travel_at_y_max: {y_max_x_travel}")
        gcmd.respond_info(f"x_travel_at_y_min: {y_min_x_travel}")
        gcmd.respond_info(f"y_travel_at_x_max: {x_max_y_travel}")
        gcmd.respond_info(f"y_travel_at_x_min: {x_min_y_travel}")
        
    def relative_move(self, axis, distance):
        axis_index = 'xyz'.index(axis.lower())
        position = self.toolhead.get_position()
        position[axis_index] += distance
        self.toolhead.manual_move(position, self.speed)

    def move_to_middle_of_axis(self, axis):
        axis_index = 'xyz'.index(axis.lower())
        position = self.toolhead.get_position()
        rail = self.kin.rails[axis_index]
        _, rail_max = rail.get_range()
        middle = rail_max / 2
        position[axis_index] = middle
        self.toolhead.move(position, self.speed)

    def do_autodetect_travel_for_axis(self, axis):
        self.gcode.respond_info(f"Autodetecting envelope {axis}")
        axis_index = 'xyz'.index(axis.lower())
        homing_positive_dir = self.homing_infos[axis].positive_dir
        direction = homing_positive_dir
        #true = positive, false = negative
        self.set_current_pre_home(axis)
        distances = []
        for _ in range(self.iterations):
            self.do_single_home(
                axis, direction
            )
            length = self.do_single_home(
                axis, not direction
            )
            distances.append(length)
        if homing_positive_dir: #make sure we end at right side
            self.do_single_home(
                axis, True
            )
        self.set_current_post_home(axis)

        mode = statistics.mode(distances)
        mode = math.floor(mode * 10) / 10

        position = self.toolhead.get_position()
        position[axis_index] = mode
        self.toolhead.set_position(position, homing_axes=[axis_index])
        self.kin.set_axis_limits(axis_index, (0, mode))
        return mode
    
    def set_current_pre_home(self, axis):
        axis_index = "xyz".index(axis.lower())
        self.homing_state._set_current_homing([axis_index], pre_homing=True)

    def set_current_post_home(self, axis):
        axis_index = "xyz".index(axis.lower())
        self.homing_state._set_current_homing([axis_index], pre_homing=False)

    def do_single_home(
        self,
        axis: str,
        positive: bool,
    ):
        axis_index = "xyz".index(axis.lower())
        rail = self.kin.rails[axis_index]
        rail_min, rail_max = rail.get_range()
        rail_range = rail_max - rail_min
        buffer = rail_range * 0.25
        
        max_pos = [0.0, 0.0, 0.0, 0.0]
        max_pos[axis_index] = self.kin.axes_max[axis_index]
        min_pos = [0.0, 0.0, 0.0, 0.0]
        min_pos[axis_index] = self.kin.axes_min[axis_index]

        if positive:
            start_pos = min_pos
            home_pos = max_pos
            home_pos[axis_index] += buffer
        else:
            start_pos = max_pos
            home_pos = min_pos
            home_pos[axis_index] -= buffer
        logging.info(f"positive: {positive}")
        logging.info(f"start_pos: {start_pos}, home_pos: {home_pos}")
        hi = rail.get_homing_info()
        self.toolhead.set_position(start_pos, homing_axes=[axis_index])
        old_limits = self.kin.limits[axis_index]
        self.kin.set_axis_limits(
            axis_index, (min_pos[axis_index] - buffer, max_pos[axis_index] + buffer)
        )
        endstops = rail.get_endstops()
        hmove = HomingMove(self.printer, endstops)
        hmove.homing_move(home_pos, hi.speed)
        dist_from_start_to_endstop = hmove.distance_elapsed
        pos = self.toolhead.get_position()
        
        pos[axis_index] = old_limits[int(positive)]
        self.toolhead.set_position(pos, homing_axes = [axis_index])
        self.kin.set_axis_limits(
            axis_index, old_limits
        )
        print_time = self.toolhead.get_last_move_time()
        for endstop in endstops:
            # re-querying a tmc endstop seems to reset the state
            # otherwise it triggers almost immediately upon second home
            # this seems to be an adequate substitute for a 2 second dwell.
            endstop[0].query_endstop(print_time)
        return abs(dist_from_start_to_endstop[axis_index])


def load_config(config):
    return EnvelopeAutoDetect(config)
