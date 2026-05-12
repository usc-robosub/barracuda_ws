import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
import teensy_constants as TC

HEADER_FILENAME = "teensy_config.h"

out = f"""
#include <cstdint>

constexpr uint8_t PORT_I2C_ADDR = {format(TC.I2C_ADDRESSES.PORT, '#04x')};
constexpr uint8_t STARBOARD_I2C_ADDR = {format(TC.I2C_ADDRESSES.STARBOARD, '#04x')};
constexpr uint8_t ENABLE_REG = {TC.ENABLE_REG};
constexpr uint8_t DCS_REG = {TC.DCS_REG};

constexpr uint8_t PWM_PINS[] = {{ {", ".join(map(str, TC.PWM_PINS))} }};
constexpr float PWM_FREQ = {TC.PWM_FREQ};
constexpr uint32_t PWM_RES = {TC.PWM_RES};
constexpr uint16_t INIT_DC = {TC.INIT_DC};
"""
out = out.lstrip() # remove leading whitespace

with open(HEADER_FILENAME, 'w') as f:
    f.write(out)


