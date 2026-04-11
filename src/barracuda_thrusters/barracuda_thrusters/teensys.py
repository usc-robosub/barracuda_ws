# for logging
from rclpy.logging import get_logger

# for i2c comms, constants
from enum import IntEnum
import struct
# from smbus import SMBus

# for reading from the enable/killswitch gpio pin
# from .jetson_gpio_utils import safe_write_i2c_block_data, setup_jetson_enable_gpio_pin
from .jetson_gpio_utils import JetsonGpioUtils

# for converting force val to duty cycle val that microcontroller uses to output pwm signal
from .force_to_pwm_dc import f_to_dc

# t200 thruster constants
from .t200_thruster_constants import T200

# teensy constants - see teensy_constants.py for full list, comments
from . import teensy_constants as TC


class Teensys:
    def __init__(self):
        self.logger = get_logger("Teensys")
        self.jetson_gpio = JetsonGpioUtils()

        # enable teensy pwm output if jetson enable gpio pin level indicates "enabled"
        # (disabled by default)
        if self.jetson_gpio.enable_pin_enabled():
           self._set_enable(True) 

        # set up the callback for rising/falling enable pin edge
        self.jetson_gpio.setup_enable_pin_callback(self._enable_pin_callback)

    def set_pwm_outputs(self, force_outptuts: list[float]):
        if len(force_outptuts) != T200.NTHRUSTERS:
            self.logger.error("length of force_outputs != T200.NTHRUSTERS")
        dcs = [f_to_dc(f) for f in force_outptuts]

        for i, addr in enumerate(TC.I2C_ADDRESSES):
            # force outputs [0, 1, 2, 3]: port teensy, force outputs [4, 5, 6, 7]: starboard teensy
            dcs_start_idx = i * len(TC.PWM_PINS)
            dcs_end_idx = dcs_start_idx + len(TC.PWM_PINS)

            # BYTES_FMT_STR specifies four (what len(TC.PWM_PINS) evals to) 16-bit unsigned ints
            data = list(struct.pack(TC.BYTES_FMT_STR, *dcs[dcs_start_idx:dcs_end_idx]))

            self.jetson_gpio.safe_write_i2c_block_data(
                # sending four 16-bit uints (8 bytes) to each teensy
                addr,
                TC.DCS_REG,
                data,
            )

    def _enable_pin_callback(self, channel):
        # is there a way to get the type of edge without checking manually here?
        # a way to set up different callbacks for different edge directions, like you can with gpiozero for pi?
        if self.jetson_gpio.enable_pin_enabled():
            self._set_enable(True)
        else:
            self._set_enable(False)

    def _set_enable(self, enabled: bool):
        for addr in TC.I2C_ADDRESSES:
            self.jetson_gpio.safe_write_i2c_block_data(
                addr, TC.ENABLE_REG, list(TC.TRUE if enabled else TC.FALSE)
            )
