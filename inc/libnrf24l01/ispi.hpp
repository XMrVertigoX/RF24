#ifndef __ISPI_HPP
#define __ISPI_HPP

#include <cstddef>
#include <cstdint>

class ISpi
{
public:
  virtual uint8_t transceive(const void* txBytes, void* rxBytes, size_t numBytes) = 0;
};

#endif /* __ISPI_HPP */
