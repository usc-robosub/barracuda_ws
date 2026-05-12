# for logging
from rclpy.logging import get_logger

# i2c comms
from smbus import SMBus

# for reading from/setting up callbacks for thruster pwm enable/killswitch pin on jetson
from Jetson import GPIO

I2C_BUS = 1
ENABLE_GPIO_PIN = 7

class JetsonGpioUtils:
    def __init__(self):
        self.logger = get_logger("JetsonGpioUtils")
        try:
            from Jetson import GPIO
            self.gpio = GPIO
        except Exception as e:
            self.gpio = None
            self.logger.error(f"could not import Jetson.GPIO: {e}")
            return
        

        self.gpio.setmode(self.gpio.BOARD)
        self.gpio.setup(ENABLE_GPIO_PIN, self.gpio.IN)

        self.bus = self._setup_i2c_bus()

    def setup_enable_pin_callback(self, callback):
        if self.gpio is None:
            self.logger.warn("setup_enable_pin_callback called, but jetson gpio was not set up")

        # potential TODO: use __enter__, __exit__, call self.gpio.remove_event_detect(ENABLE_GPIO_PIN),
        # in teensys.py, use "with JetsonGpioUtils() as self.jetson_gpio" (clean up resources)
        self.gpio.add_event_detect(
            ENABLE_GPIO_PIN,
            self.gpio.BOTH,
            callback = callback
        )

    def enable_pin_enabled(self):
        if self.gpio is None:
            self.logger.warn("enable_pin_enabled called, but jetson gpio was not set up")

        # thrusters enabled when gpio pin lo
        return self.gpio.input(ENABLE_GPIO_PIN) == self.gpio.LOW

    # wrapper for smbus write_i2c_block_data fn, includes try/except block
    # and logs warnings if needed
    def safe_write_i2c_block_data(self, addr, reg, val):
        self.logger.info(f"sending {val} to address {addr:02x}, reg {reg}")
        if self.bus is None:
            self.logger.warn("i2c bus not initialized properly")
            return
        try:
            self.bus.write_i2c_block_data(addr, reg, val)
        except Exception as e:
            self.logger.error(f"I2C write failed at addr {addr:#04x}, reg {reg}: {e}")
    

    def _setup_i2c_bus(self):
            try:
                return SMBus(I2C_BUS)
            except Exception as e:
                return None
                self.logger.warn(f"exception initializing i2c bus: {e}")

                
         
