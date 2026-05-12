#include <cstdint>

constexpr uint8_t PORT_I2C_ADDR = 0x2d;
constexpr uint8_t STARBOARD_I2C_ADDR = 0x2e;
constexpr uint8_t ENABLE_REG = 8;
constexpr uint8_t DCS_REG = 0;

constexpr uint8_t PWM_PINS[] = { 5, 4, 0, 1 };
constexpr float PWM_FREQ = 500;
constexpr uint32_t PWM_RES = 8;
constexpr uint16_t INIT_DC = 192;
