#ifndef __NnRF24L01_SPI_HPP__
#define __NnRF24L01_SPI_HPP__

#include <vector>

#include <libnrf24l01/ispi.hpp>

using namespace std;

class Spi : public ISpi
{
public:
  Spi();
  ~Spi();

  uint8_t transceive(const uint8_t txBytes[], uint8_t rxBytes[], uint32_t numBytes);

  vector<char>& getTxBytes();
  void setRxBytes(vector<char> rxBytes);

private:
  vector<char> _rxBytes;
  vector<char> _txBytes;
};

#endif /* __NnRF24L01_SPI_HPP__ */
