#ifndef __ISPI_HPP
#define __ISPI_HPP

#include <cstdint>

class ISpi
{
public:
  virtual uint8_t transmit_receive(const uint8_t* txBytes, uint8_t* rxBytes, uint32_t numBytes) = 0;
};

#endif /* __ISPI_HPP */
