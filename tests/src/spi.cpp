#include <cstring>
#include <stdexcept>

#include <spi.hpp>

Spi::Spi() {}

Spi::~Spi() {}

uint8_t Spi::transceive(const uint8_t txBytes[], uint8_t rxBytes[], uint32_t numBytes)
{
  _txBytes = vector<char>(txBytes, txBytes + numBytes);
  memcpy(rxBytes, _rxBytes.data(), numBytes);

  return 0;
}

vector<char>& Spi::getTxBytes()
{
  return _txBytes;
}

void Spi::setRxBytes(vector<char> rxBytes)
{
  _rxBytes = rxBytes;
}
