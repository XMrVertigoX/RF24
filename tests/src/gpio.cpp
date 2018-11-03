#include <gpio.hpp>

Gpio::Gpio() {}

Gpio::~Gpio() {}

void Gpio::clear()
{
  _state = false;
}

void Gpio::set(bool enable)
{
  _state = enable;
}

bool Gpio::getState()
{
  return _state;
}
