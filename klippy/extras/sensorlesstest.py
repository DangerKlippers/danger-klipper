class SensorlessTest:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.stepper_x_5160 = self.printer.lookup_object("tmc5160 stepper_x")
        self.stepper_y_5160 = self.printer.lookup_object("tmc5160 stepper_y")

    def get_status(self, eventtime):
        sg_result_x = self.stepper_x_5160.fields.get_field("sg_result")
        sg_result_y = self.stepper_y_5160.fields.get_field("sg_result")
        return {"sg_result_x": sg_result_x, "sg_result_y": sg_result_y}


def load_config_prefix(config):
    return SensorlessTest(config)
