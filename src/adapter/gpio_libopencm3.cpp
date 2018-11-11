#include <libopencm3/stm32/gpio.h>

#include "../inc/libnrf24l01/gpio_libopencm3.hpp"

namespace libnrf24l01
{

Gpio_libopencm3::Gpio_libopencm3(uint32_t gpioport, uint16_t gpios)
    : _gpioport(gpioport), _gpios(gpios) {}

Gpio_libopencm3::~Gpio_libopencm3() {}

void Gpio_libopencm3::set(bool enable)
{
  enable ? gpio_set(_gpioport, _gpios) : gpio_clear(_gpioport, _gpios);
}

} // namespace libnrf24l01
