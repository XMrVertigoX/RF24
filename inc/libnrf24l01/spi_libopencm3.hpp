#ifndef __SPI_libopencm3_HPP
#define __SPI_libopencm3_HPP

#include "igpio.hpp"
#include "ispi.hpp"

namespace libnrf24l01
{

class Spi_libopencm3 : public ISpi
{
public:
  Spi_libopencm3(uint32_t spi, IGpio& ss);
  virtual ~Spi_libopencm3();

  uint8_t transceive(const void* txBytes, void* rxBytes, size_t numBytes);

private:
  uint32_t _spi;
  IGpio& _ss;
};

} // namespace libnrf24l01

#endif /* __SPI_libopencm3_HPP */
