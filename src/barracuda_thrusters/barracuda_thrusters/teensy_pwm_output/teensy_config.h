#include <cstdint>

#define PORT_I2C_ADDR 0x2d
#define STARBOARD_I2C_ADDR 0x2e
#define ENABLE_REG 8
#define DCS_REG 0

#define N_PWM_PINS 4
constexpr uint8_t PWM_PINS[] = { 5, 4, 0, 1 };
constexpr float PWM_FREQ = 500;
constexpr uint32_t PWM_RES = 8;
constexpr uint16_t INIT_DC = 192;
