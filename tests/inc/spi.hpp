#ifndef __NnRF24L01_SPI_HPP__
#define __NnRF24L01_SPI_HPP__

#include <queue>
#include <vector>

#include <libnrf24l01/ispi.hpp>

using namespace std;

class Spi : public ISpi
{
public:
  Spi(queue<vector<uint8_t>>& rxQueue, queue<vector<uint8_t>>& txQueue);
  ~Spi();

  uint8_t transceive(const uint8_t txBytes[], uint8_t rxBytes[], uint32_t numBytes);

private:
  queue<vector<uint8_t>>& _rxQueue;
  queue<vector<uint8_t>>& _txQueue;
};

#endif /* __NnRF24L01_SPI_HPP__ */
