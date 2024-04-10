# Axis Twist Compensation
#
# Copyright (C) 2022  Jeremy Tan <jeremytkw98@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
from . import manual_probe as ManualProbe, bed_mesh as BedMesh


DEFAULT_SAMPLE_COUNT = 3
DEFAULT_SPEED = 50.0
DEFAULT_HORIZONTAL_MOVE_Z = 5.0


class AxisTwistCompensation:
    def __init__(self, config):
        # get printer
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        gcode_macro = self.printer.load_object(config, "gcode_macro")

        # get values from [axis_twist_compensation] section in printer .cfg
        self.horizontal_move_z = config.getfloat(
            "horizontal_move_z", DEFAULT_HORIZONTAL_MOVE_Z
        )
        self.speed = config.getfloat("speed", DEFAULT_SPEED)
        self.calibrate_start_x = config.getfloat("calibrate_start_x")
        self.calibrate_end_x = config.getfloat("calibrate_end_x")
        self.calibrate_y = config.getfloat("calibrate_y")

        self.wait_for_continue = config.getboolean("wait_for_continue", True)
        self.start_gcode = gcode_macro.load_template(config, "start_gcode", "")
        self.end_gcode = gcode_macro.load_template(config, "end_gcode", "")
        self.abort_gcode = (
            self.end_gcode
            if config.get("abort_gcode", None) is None
            else gcode_macro.load_template(config, "abort_gcode", "")
        )

        self.z_compensations = config.getlists(
            "z_compensations", default=[], parser=float
        )
        self.compensation_start_x = config.getfloat(
            "compensation_start_x", default=None
        )
        self.compensation_end_x = config.getfloat(
            "compensation_start_y", default=None
        )

        self.m = None
        self.b = None

        # setup calibrater
        self.calibrater = Calibrater(self, config)

    def get_status(self, eventtime):
        return self.calibrater.get_status(eventtime)

    def get_z_compensation_value(self, pos):
        if not self.z_compensations:
            return 0

        x_coord = pos[0]
        z_compensations = self.z_compensations
        sample_count = len(z_compensations)
        spacing = (self.calibrate_end_x - self.calibrate_start_x) / (
            sample_count - 1
        )
        interpolate_t = (x_coord - self.calibrate_start_x) / spacing
        interpolate_i = int(math.floor(interpolate_t))
        interpolate_i = BedMesh.constrain(interpolate_i, 0, sample_count - 2)
        interpolate_t -= interpolate_i
        interpolated_z_compensation = BedMesh.lerp(
            interpolate_t,
            z_compensations[interpolate_i],
            z_compensations[interpolate_i + 1],
        )
        return interpolated_z_compensation

    def clear_compensations(self):
        self.z_compensations = []
        self.m = None
        self.b = None


class Calibrater:
    def __init__(self, compensation, config):
        # setup self attributes
        self.compensation = compensation
        self.printer = compensation.printer
        self.gcode = self.printer.lookup_object("gcode")
        self.probe = None
        # probe settings are set to none, until they are available
        self.lift_speed, self.probe_x_offset, self.probe_y_offset, _ = (
            None,
            None,
            None,
            None,
        )
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )
        self.speed = compensation.speed
        self.horizontal_move_z = compensation.horizontal_move_z
        self.start_gcode = compensation.start_gcode
        self.end_gcode = compensation.end_gcode
        self.abort_gcode = compensation.abort_gcode
        self.wait_for_continue = compensation.wait_for_continue
        self.start_point = (
            compensation.calibrate_start_x,
            compensation.calibrate_y,
        )
        self.end_point = (
            compensation.calibrate_end_x,
            compensation.calibrate_y,
        )
        self.results = None
        self.current_point_index = None
        self.gcmd = None
        self.configname = config.get_name()

        self.nozzle_points = None
        self.probe_points = None
        self.interval_dist = None

        self.is_active = False
        # register gcode handlers
        self._register_gcode_handlers()

    def _handle_connect(self):
        self.probe = self.printer.lookup_object("probe", None)
        if self.probe is None:
            config = self.printer.lookup_object("configfile")
            raise config.error(
                "AXIS_TWIST_COMPENSATION requires [probe] to be defined"
            )
        self.lift_speed = self.probe.get_lift_speed()
        self.probe_x_offset, self.probe_y_offset, _ = self.probe.get_offsets()

    def _register_gcode_handlers(self):
        # register gcode handlers
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "AXIS_TWIST_COMPENSATION_CALIBRATE",
            self.cmd_AXIS_TWIST_COMPENSATION_CALIBRATE,
            desc=self.cmd_AXIS_TWIST_COMPENSATION_CALIBRATE_help,
        )

    def get_status(self, eventtime):
        return {"is_active": self.is_active}

    cmd_AXIS_TWIST_COMPENSATION_CALIBRATE_help = """
    Performs the x twist calibration wizard
    Measure z probe offset at n points along the x axis,
    and calculate x twist compensation
    """

    def cmd_AXIS_TWIST_COMPENSATION_CALIBRATE(self, gcmd):
        if self.is_active:
            raise gcmd.error(
                "Already running a twist compensation calibration. Use ABORT"
                " to abort it."
            )
        manual_probe = self.printer.lookup_object("manual_probe")
        if manual_probe.status["is_active"]:
            raise gcmd.error(
                "Already in a manual Z probe. Use ABORT to abort it."
            )
        self.is_active = True
        self.gcode.register_command(
            "ABORT", self.cmd_ABORT, desc=self.cmd_ABORT_help
        )
        self.gcode.register_command(
            "QUERY_TWIST_COMPENSATION_RUNNING",
            self.cmd_QUERY_TWIST_COMPENSATION_RUNNING,
            desc=self.cmd_QUERY_TWIST_COMPENSATION_RUNNING_help,
        )
        self.start_gcode.run_gcode_from_command()

        self.gcmd = gcmd
        sample_count = gcmd.get_int("SAMPLE_COUNT", DEFAULT_SAMPLE_COUNT)

        # check for valid sample_count
        if sample_count is None or sample_count < 2:
            raise self.gcmd.error("SAMPLE_COUNT to probe must be at least 2")

        # clear the current config
        self.compensation.clear_compensations()

        # calculate some values
        x_range = self.end_point[0] - self.start_point[0]
        self.interval_dist = x_range / (sample_count - 1)
        self.nozzle_points = self._calculate_nozzle_points(
            sample_count, self.interval_dist
        )
        self.probe_points = self._calculate_probe_points(
            self.nozzle_points, self.probe_x_offset, self.probe_y_offset
        )

        # begin calibration
        self.current_point_index = 0
        self.results = []
        self._calibration(
            self.probe_points, self.nozzle_points, self.interval_dist
        )

    def _calculate_nozzle_points(self, sample_count, interval_dist):
        # calculate the points to put the probe at, returned as a list of tuples
        nozzle_points = []
        for i in range(sample_count):
            x = self.start_point[0] + i * interval_dist
            y = self.start_point[1]
            nozzle_points.append((x, y))
        return nozzle_points

    def _calculate_probe_points(
        self, nozzle_points, probe_x_offset, probe_y_offset
    ):
        # calculate the points to put the nozzle at
        # returned as a list of tuples
        probe_points = []
        for point in nozzle_points:
            x = point[0] - probe_x_offset
            y = point[1] - probe_y_offset
            probe_points.append((x, y))
        return probe_points

    def _move_helper(self, target_coordinates, override_speed=None):
        # pad target coordinates
        target_coordinates = (
            (target_coordinates[0], target_coordinates[1], None)
            if len(target_coordinates) == 2
            else target_coordinates
        )
        toolhead = self.printer.lookup_object("toolhead")
        speed = self.speed if target_coordinates[2] is None else self.lift_speed
        speed = override_speed if override_speed is not None else speed
        toolhead.manual_move(target_coordinates, speed)

    def _calibration(self, probe_points, nozzle_points, interval):
        # begin the calibration process
        self.gcmd.respond_info(
            "AXIS_TWIST_COMPENSATION_CALIBRATE: "
            "Probing point %d of %d"
            % (self.current_point_index + 1, len(probe_points))
        )

        # horizontal_move_z (to prevent probe trigger or hitting bed)
        self._move_helper((None, None, self.horizontal_move_z))

        # move to point to probe
        self._move_helper(
            (
                probe_points[self.current_point_index][0],
                probe_points[self.current_point_index][1],
                None,
            )
        )

        # probe the point
        self.current_measured_z = self.probe.run_probe(self.gcmd)[2]

        # horizontal_move_z (to prevent probe trigger or hitting bed)
        self._move_helper((None, None, self.horizontal_move_z))

        # move the nozzle over the probe point
        self._move_helper((nozzle_points[self.current_point_index]))

        self.gcode.register_command("ABORT", None)

        # start the manual (nozzle) probe
        ManualProbe.ManualProbeHelper(
            self.printer,
            self.gcmd,
            self._manual_probe_callback_factory(
                probe_points, nozzle_points, interval
            ),
        )

    def _manual_probe_callback_factory(
        self, probe_points, nozzle_points, interval_dist
    ):
        # returns a callback function for the manual probe
        is_end = self.current_point_index == len(probe_points) - 1

        def callback(kin_pos):
            if kin_pos is None:
                # probe was cancelled
                self.cmd_ABORT()
                return
            z_offset = self.current_measured_z - kin_pos[2]
            self.results.append(z_offset)
            if is_end:
                # end of calibration
                self._finalize_calibration()
            else:
                # move to next point
                self.current_point_index += 1

                self.probe_points = probe_points
                self.nozzle_points = nozzle_points
                self.interval_dist = interval_dist

                self.gcode.register_command(
                    "ABORT", self.cmd_ABORT, desc=self.cmd_ABORT_help
                )

                if self.wait_for_continue:
                    self._move_helper((None, None, self.horizontal_move_z))

                    self.gcode.register_command(
                        "CONTINUE",
                        self.cmd_CONTINUE,
                        desc=self.cmd_CONTINUE_help,
                    )

                    self.gcmd.respond_info(
                        "Type CONTINUE to continue to the next probing point"
                    )
                else:
                    self._calibration(
                        self.probe_points,
                        self.nozzle_points,
                        self.interval_dist,
                    )

        return callback

    cmd_CONTINUE_help = "Continue to the next probing point"

    def cmd_CONTINUE(self, gcmd):
        self.gcode.register_command("CONTINUE", None)
        self._calibration(
            self.probe_points, self.nozzle_points, self.interval_dist
        )

    cmd_QUERY_TWIST_COMPENSATION_RUNNING_help = """Query if we are running a
                                                   twist compensation"""

    def cmd_QUERY_TWIST_COMPENSATION_RUNNING(self, gcmd):
        gcmd.respond_info("Twist Compensation running")
        return

    cmd_ABORT_help = "Abort the running probe calibration"

    def cmd_ABORT(self, gcmd=None):
        self.gcmd.respond_info(
            "AXIS_TWIST_COMPENSATION_CALIBRATE: Probe cancelled, "
            "calibration aborted"
        )
        self.abort_gcode.run_gcode_from_command()
        self.is_active = False
        self.gcode.register_command("QUERY_TWIST_COMPENSATION_RUNNING", None)
        self.gcode.register_command("ABORT", None)
        self.gcode.register_command("CONTINUE", None)

    def _finalize_calibration(self):
        # finalize the calibration process
        # calculate average of results
        avg = sum(self.results) / len(self.results)
        # subtract average from each result
        # so that they are independent of z_offset
        self.results = [avg - x for x in self.results]
        # save the config
        configfile = self.printer.lookup_object("configfile")
        values_as_str = ", ".join(["{:.6f}".format(x) for x in self.results])
        configfile.set(self.configname, "z_compensations", values_as_str)
        configfile.set(
            self.configname, "compensation_start_x", self.start_point[0]
        )
        configfile.set(self.configname, "compensation_end_x", self.end_point[0])
        self.compensation.z_compensations = self.results
        self.compensation.compensation_start_x = self.start_point[0]
        self.compensation.compensation_end_x = self.end_point[0]

        self.nozzle_points = None
        self.probe_points = None
        self.interval_dist = None

        self.gcode.register_command("ABORT", None)
        self.gcode.register_command("CONTINUE", None)

        # output result
        self.gcmd.respond_info(
            "AXIS_TWIST_COMPENSATION_CALIBRATE: Calibration complete, "
            "offsets: %s, mean z_offset: %f" % (self.results, avg)
        )

        self.gcode.respond_info(
            "AXIS_TWIST_COMPENSATION state has been saved "
            "for the current session.  The SAVE_CONFIG command will "
            "update the printer config file and restart the printer."
        )

        self.end_gcode.run_gcode_from_command()
        self.is_active = False
        self.gcode.register_command("QUERY_TWIST_COMPENSATION_RUNNING", None)


def verify_no_compensation(printer):
    gcode = printer.lookup_object("gcode")
    try:
        gcode.register_command("QUERY_TWIST_COMPENSATION_RUNNING", "dummy")
        gcode.register_command("QUERY_TWIST_COMPENSATION_RUNNING", None)
    except printer.config_error as e:
        raise gcode.error(
            "Already running a twist compensation calibration. Use ABORT"
            " to abort it."
        )


# klipper's entry point using [axis_twist_compensation] section in printer.cfg
def load_config(config):
    return AxisTwistCompensation(config)
