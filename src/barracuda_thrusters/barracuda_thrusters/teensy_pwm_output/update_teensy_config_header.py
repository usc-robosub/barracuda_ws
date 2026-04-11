import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
import teensy_constants as TC

HEADER_FILENAME = "teensy_config.h"

out = f"""
#include <cstdint>

#define PORT_I2C_ADDR {format(TC.I2C_ADDRESSES.PORT, '#04x')}
#define STARBOARD_I2C_ADDR {format(TC.I2C_ADDRESSES.STARBOARD, '#04x')}
#define ENABLE_REG {TC.ENABLE_REG}
#define DCS_REG {TC.DCS_REG}

#define N_PWM_PINS {len(TC.PWM_PINS)}
constexpr uint8_t PWM_PINS[] = {{ {", ".join(map(str, TC.PWM_PINS))} }};
constexpr float PWM_FREQ = {TC.PWM_FREQ};
constexpr uint32_t PWM_RES = {TC.PWM_RES};
constexpr uint16_t INIT_DC = {TC.INIT_DC};
"""
out = out.lstrip() # remove leading whitespace

with open(HEADER_FILENAME, 'w') as f:
    f.write(out)


