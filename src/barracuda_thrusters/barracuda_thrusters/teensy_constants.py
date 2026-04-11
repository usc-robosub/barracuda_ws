import struct
from enum import IntEnum 

try:
    from .t200_thruster_constants import T200
except ImportError:
    # non-module import here so that the update_teensy_config_header.py
    # can use the constants in this file when run as a standalone script
    # in teensy_pwm_output/compile.sh compile/flashing script
    from t200_thruster_constants import T200

# pwm config constants (see https://www.pjrc.com/teensy/td_pulse.html for more) #
#################################################################################

# the pwm output pins that are used on each teensy
# PWM_PINS[0] is connected to leftmost esc connector on thruster board,
# PWM_PINS[3] is connected to rightmost esc connector on thruster board
PWM_PINS = [5, 4, 0, 1]

# pwm frequency in Hz (1/f = T = width of 100% duty cycle in microseconds)
PWM_FREQ = 500

# pwm bit resolution
PWM_RES = 8

# precalculated dc value to achieve 1500us pulse width (init width)
# 1500e-6 / T = INIT_DC / 2^(PWM_RES)
# 1500e-6 / (1 / PWM_FREQ) = INIT_DC / 2^(PWM_RES)
# 1500e-6 * PWM_FREQ = INIT_DC / 2^(PWM_RES)
# INIT_DC = 1500e-6 * PWM_FREQ * 2^(PWM_RES)
INIT_DC = round((T200.INIT_PW * (PWM_FREQ / 1e6)) * 2**PWM_RES)


# i2c/comms constants #
#######################
I2C_BUS = 1
class I2C_ADDRESSES(IntEnum):
    PORT = 0x2d
    STARBOARD = 0x2e

DCS_REG = 0
ENABLE_REG = 8

FALSE = struct.pack("<B", 0) # little-endian uint8
TRUE = struct.pack ("<B", 1) # little-endian uint8

BYTES_FMT_STR = "<" + len(PWM_PINS) * "H" # little-endian, 16-bit uint * len(PWM_PINS)
