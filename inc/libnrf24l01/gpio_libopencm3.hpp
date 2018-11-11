#ifndef __GPIO_libopencm3_HPP
#define __GPIO_libopencm3_HPP

#include <cstdint>

#include "igpio.hpp"

class Gpio_libopencm3 : public IGpio
{
public:
  Gpio_libopencm3(uint32_t gpioport, uint16_t gpios);
  virtual ~Gpio_libopencm3();

  void set(bool enable = true);

private:
  uint32_t _gpioport;
  uint16_t _gpios;
};

#endif /* __GPIO_libopencm3_HPP */
