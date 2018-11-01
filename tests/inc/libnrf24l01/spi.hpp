#ifndef __NnRF24L01_SPI_HPP__
#define __NnRF24L01_SPI_HPP__

#include <vector>

#include <libnrf24l01/igpio.hpp>
#include <libnrf24l01/ispi.hpp>

using namespace std;

class Spi : public ISpi
{
public:
  Spi();
  ~Spi();

  virtual uint8_t transceive(const uint8_t txBytes[], uint8_t rxBytes[], uint32_t numBytes) = 0;
};

#endif /* __NnRF24L01_SPI_HPP__ */
