#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "../inc/libnrf24l01/igpio.hpp"
#include "../inc/libnrf24l01/ispi.hpp"
#include "../inc/libnrf24l01/nrf24.hpp"

namespace libnrf24l01
{

using namespace std;

nRF24::nRF24(ISpi& spi, IGpio& ce) : nRF24_LL(spi), _ce(ce) {}

nRF24::~nRF24() {}

void nRF24::init()
{
  clearInterrupts();

  FLUSH_RX();
  FLUSH_TX();
}

void nRF24::process()
{
  if (notification)
  {
    uint8_t status = NOP();

    if (status & STATUS_RX_DR_MASK)
    {
      handleDataReady(status);
    }

    if (status & STATUS_TX_DS_MASK)
    {
      handleDataSent(status);
    }

    if (status & STATUS_MAX_RT_MASK)
    {
      handleMaxRetransmission(status);
    }

    notification = false;
  }
}

static inline uint8_t __getPipeFromStatus(uint8_t status)
{
  return ((status & STATUS_RX_P_NO_MASK) >> STATUS_RX_P_NO);
}

void nRF24::handleDataReady(uint8_t status)
{
  uint8_t bytes[__MAX_FIFO_SIZE];
  size_t numBytes = readRxFifo(bytes, sizeof(bytes));

  if (numBytes < 0)
  {
    FLUSH_RX();
  }

  if (numBytes && rxCallback)
  {
    rxCallback(__getPipeFromStatus(status), bytes, numBytes, rxContext);
  }

  writeShort(nRF24_Register::STATUS, STATUS_RX_DR_MASK);
}

void nRF24::handleDataSent(uint8_t status)
{
  if (txCallback)
  {
    txCallback(txContext);
  }

  writeShort(nRF24_Register::STATUS, STATUS_TX_DS_MASK);
}

void nRF24::handleMaxRetransmission(uint8_t status)
{
  FLUSH_TX();
  writeShort(nRF24_Register::STATUS, STATUS_MAX_RT_MASK);
}

void nRF24::enterShutdownMode()
{
  uint8_t config = readShort(nRF24_Register::CONFIG);

  if (_isBitSet(config, CONFIG_PWR_UP) == true)
  {
    _clearBit(config, CONFIG_PWR_UP);
    writeShort(nRF24_Register::CONFIG, config);
  }
}

void nRF24::enterStandbyMode()
{
  _ce.set(false);

  uint8_t config = readShort(nRF24_Register::CONFIG);

  if (_isBitSet(config, CONFIG_PWR_UP) == false)
  {
    _setBit(config, CONFIG_PWR_UP);
    writeShort(nRF24_Register::CONFIG, config);
  }
}

void nRF24::enterRxMode()
{
  enterStandbyMode();

  uint8_t config = readShort(nRF24_Register::CONFIG);
  _setBit(config, CONFIG_PRIM_RX);
  writeShort(nRF24_Register::CONFIG, config);

  _ce.set();
}

void nRF24::enterTxMode()
{
  enterStandbyMode();

  uint8_t config = readShort(nRF24_Register::CONFIG);
  _clearBit(config, CONFIG_PRIM_RX);
  writeShort(nRF24_Register::CONFIG, config);

  _ce.set();
}

int nRF24::enqueueData(void* bytes, size_t numBytes)
{
  return writeTxFifo(bytes, numBytes);
}

void nRF24::notify()
{
  notification = true;
}

void nRF24::setRxCallback(nRF24_RxCallback_t callback, void* context)
{
  rxCallback = callback;
  rxContext = context;
}

void nRF24::setTxCallback(nRF24_TxCallback_t callback, void* context)
{
  txCallback = callback;
  txContext = context;
}

} // namespace libnrf24l01
