# Support for GPIO input edge counters
#
# Copyright (C) 2021  Adrian Keet <arkeet@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class MCU_counter:
    def __init__(self, printer, pin, sample_time, poll_time):
        ppins = printer.lookup_object("pins")
        pin_params = ppins.lookup_pin(pin, can_pullup=True)
        self._mcu = pin_params["chip"]
        self._oid = self._mcu.create_oid()
        self._pin = pin_params["pin"]
        self._pullup = pin_params["pullup"]
        self._poll_time = poll_time
        self._poll_ticks = 0
        self._sample_time = sample_time
        self._callback = None
        self._last_count = 0
        self._mcu.register_config_callback(self.build_config)

    def build_config(self):
        self._mcu.add_config_cmd(
            "config_counter oid=%d pin=%s pull_up=%d"
            % (self._oid, self._pin, self._pullup)
        )
        clock = self._mcu.get_query_slot(self._oid)
        self._poll_ticks = self._mcu.seconds_to_clock(self._poll_time)
        sample_ticks = self._mcu.seconds_to_clock(self._sample_time)
        self._mcu.add_config_cmd(
            "query_counter oid=%d clock=%d poll_ticks=%d sample_ticks=%d"
            % (self._oid, clock, self._poll_ticks, sample_ticks),
            is_init=True,
        )
        self._mcu.register_response(
            self._handle_counter_state, "counter_state", self._oid
        )

    # Callback is called periodically every sample_time
    def setup_callback(self, cb):
        self._callback = cb

    def _handle_counter_state(self, params):
        next_clock = self._mcu.clock32_to_clock64(params["next_clock"])
        time = self._mcu.clock_to_print_time(next_clock - self._poll_ticks)

        count_clock = self._mcu.clock32_to_clock64(params["count_clock"])
        count_time = self._mcu.clock_to_print_time(count_clock)

        # handle 32-bit counter overflow
        last_count = self._last_count
        delta_count = (params["count"] - last_count) & 0xFFFFFFFF
        count = last_count + delta_count
        self._last_count = count

        if self._callback is not None:
            self._callback(time, count, count_time)


class FrequencyCounter:
    def __init__(self, printer, pin, sample_time, poll_time):
        self._callback = None
        self._last_time = self._last_count = None
        self._freq = 0.0
        self._counter = MCU_counter(printer, pin, sample_time, poll_time)
        self._counter.setup_callback(self._counter_callback)

    def _counter_callback(self, time, count, count_time):
        if self._last_time is None:  # First sample
            self._last_time = time
        else:
            delta_time = count_time - self._last_time
            if delta_time > 0:
                self._last_time = count_time
                delta_count = count - self._last_count
                self._freq = delta_count / delta_time
            else:  # No counts since last sample
                self._last_time = time
                self._freq = 0.0
            if self._callback is not None:
                self._callback(time, self._freq)
        self._last_count = count

    def get_frequency(self):
        return self._freq


class PWMCounter:
    def __init__(self, printer, pin, sample_time, poll_time, frequency):
        self._callback = None
        self._last_time = self._last_count = None
        self._frequency = frequency  # pulses / second at 100% duty cycle
        self._duty_cycle = 0.0
        self._counter = MCU_counter(printer, pin, sample_time, poll_time)
        self._counter.setup_callback(self._counter_callback)

    def _counter_callback(self, time, count, count_time):
        if self._last_time is None:  # First sample
            self._last_time = time
        else:
            delta_time = count_time - self._last_time
            if delta_time > 0:
                self._last_time = count_time
                delta_count = count - self._last_count
                counts_per_sec = delta_count / delta_time
                # if delta time is 0.25 seconds
                # and delta count is 10
                # and pwm freq is 100hz
                # counts per second is 40
                # so duty cycle is 40 / 100 = 0.4
                time_scale_factor = 1 / delta_time
                self.duty_cycle = (
                    counts_per_sec * time_scale_factor / self._frequency
                )

            else:  # No counts since last sample
                self._last_time = time
                self._freq = 0.0
            if self._callback is not None:
                self._callback(time, self._freq)

        self._last_count = count

    def get_duty_cycle(self):
        return self._duty_cycle
