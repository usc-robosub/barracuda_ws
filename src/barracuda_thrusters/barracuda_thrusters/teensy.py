from rclpy.logging import get_logger
from smbus import SMBus
import struct
from collections import namedtuple

logger = get_logger("Teensy")

i2c_addresses = [0x2D, 0x2E]

KILLSWITCH_REG = 16

# thruster register:  12,   8,   0,   4
# teensy pwm pin   :   5,   4,   0,   1
# thruster index   : 0/4, 1/5, 2/6, 3/7
thruster_registers = [12, 8, 0, 4]


def write_i2c_float(addr, reg, val):
    if bus is None:
        return

    logger.info(f"sending {val} to address {addr:02x}, reg {reg}")

    # f is for float (16-bit)
    data = list(struct.pack("<f", val))
    try:
        bus.write_i2c_block_data(addr, reg, data)
    except Exception as e:
        logger.error(f"I2C float write failed at addr {addr:#04x}, reg {reg}: {e}")


def write_i2c_char(addr, reg, val):
    if bus is None:
        return

    logger.info(f"sending {val} to address {addr:02x}, reg {reg}")

    # c is for float (8-bit)
    data = list(struct.pack("<c", val))
    try:
        bus.write_i2c_block_data(addr, reg, data)
    except Exception as e:
        logger.error(f"I2C char write failed at addr {addr:#04x}, reg {reg}: {e}")


def read_i2c_char(addr, reg):
    if bus is None:
        return None

    logger.info(f"reading from address {addr:02x}, reg {reg}")

    try:
        val = struct.unpack("<c", bytes(bus.read_i2c_block_data(addr, reg, 2)))[0]
        return val
    except Exception as e:
        logger.error(f"I2C char read failed at addr {addr:#04x}, reg {reg}: {e}")
        return None


# run on module import
try:
    # bus = SMBus(7) # for jetson
    bus = SMBus(1)  # for pi
except Exception as e:
    bus = None
    logger.warn(f"exception initializing i2c bus: {e}")
