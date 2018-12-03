#include <libopencm3/stm32/spi.h>

#include "../inc/libnrf24l01/spi_libopencm3.hpp"

namespace libnrf24l01
{

Spi_libopencm3::Spi_libopencm3(uint32_t spi, IGpio& ss)
    : _spi(spi), _ss(ss)
{
  _ss.set();
}

Spi_libopencm3::~Spi_libopencm3()
{
  _ss.set(false);
}

uint8_t Spi_libopencm3::transceive(const void* txBytes, void* rxBytes, size_t numBytes)
{
  _ss.set(false);

  for (int i = 0; i < numBytes; i++)
  {
    spi_send8(_spi, static_cast<const uint8_t*>(txBytes)[i]);
    static_cast<uint8_t*>(rxBytes)[i] = spi_read8(_spi);
  }

  _ss.set();

  return 0;
}

} // namespace libnrf24l01
