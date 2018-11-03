#include <cstring>
#include <stdexcept>

#include <spi.hpp>

Spi::Spi(queue<vector<uint8_t>>& rxQueue, queue<vector<uint8_t>>& txQueue)
    : _rxQueue(rxQueue), _txQueue(txQueue) {}

Spi::~Spi() {}

uint8_t Spi::transceive(const uint8_t txBytes[], uint8_t rxBytes[], uint32_t numBytes)
{
  // Push tx bytes to queue
  _txQueue.push(vector<uint8_t>(txBytes, txBytes + numBytes));

  // Return next data set in line
  memcpy(rxBytes, _rxQueue.front().data(), numBytes);
  _rxQueue.pop();

  return 0;
}
