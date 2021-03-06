#ifndef __NnRF24L01_GPIO_HPP__
#define __NnRF24L01_GPIO_HPP__

#include <libnrf24l01/igpio.hpp>

class Gpio : public IGpio
{
public:
  Gpio();
  ~Gpio();

  void clear();
  void set(bool enable = true);

  bool getState();

private:
  bool _state;
};

#endif /* __NnRF24L01_GPIO_HPP__ */
