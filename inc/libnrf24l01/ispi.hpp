#ifndef __ISPI_HPP
#define __ISPI_HPP

#include <cstddef>
#include <cstdint>

namespace libnrf24l01
{

class ISpi
{
public:
  virtual uint8_t transceive(const void* txBytes, void* rxBytes, size_t numBytes) = 0;
};

} // namespace libnrf24l01

#endif /* __ISPI_HPP */
