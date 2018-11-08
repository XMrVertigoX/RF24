#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <libnrf24l01/igpio.hpp>
#include <libnrf24l01/ispi.hpp>
#include <libnrf24l01/nrf24.hpp>

using namespace std;

nRF24::nRF24(ISpi& spi, IGpio& ce) : nRF24_LL(spi), _ce(ce) {}

nRF24::~nRF24() {}

void __attribute__((weak)) delayUs(int us) {}

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
      writeShort(nRF24_Register::STATUS, STATUS_RX_DR_MASK);
    }

    if (status & STATUS_TX_DS_MASK)
    {
      handleDataSent(status);
      writeShort(nRF24_Register::STATUS, STATUS_TX_DS_MASK);
    }

    if (status & STATUS_MAX_RT_MASK)
    {
      handleMaxRetransmission(status);
      writeShort(nRF24_Register::STATUS, STATUS_MAX_RT_MASK);
    }

    notification = false;
  }

  if (rxBuffer.empty() == false)
  {
    if (rxCallback)
    {
      rxCallback(rxBuffer.front(), rxContext);
    }

    rxBuffer.pop_front();
  }
}

static inline uint8_t __getPipe(uint8_t status)
{
  return ((status & STATUS_RX_P_NO_MASK) >> STATUS_RX_P_NO);
}

void nRF24::handleDataReady(uint8_t status)
{
  nRF24_Datagram_t data;
  data.pipe = __getPipe(status);

  if (readRxFifo(data))
  {
    FLUSH_RX();
  }
  else
  {
    rxBuffer.push_back(data);
  }
}

void nRF24::handleDataSent(uint8_t status)
{
  if (txCallback)
  {
    txCallback(txContext);
  }
}

void nRF24::handleMaxRetransmission(uint8_t status)
{
  FLUSH_TX();
}

void nRF24::enterRxMode()
{
  uint8_t config = readShort(nRF24_Register::CONFIG);
  _setBit(config, CONFIG_PWR_UP);
  _setBit(config, CONFIG_PRIM_RX);
  writeShort(nRF24_Register::CONFIG, config);

  _ce.set();

  delayUs(rxSettling);
}

void nRF24::enterShutdownMode()
{
  _ce.clear();

  uint8_t config = readShort(nRF24_Register::CONFIG);
  _clearBit(config, CONFIG_PWR_UP);
  writeShort(nRF24_Register::CONFIG, config);
}

void nRF24::enterStandbyMode()
{
  _ce.clear();

  uint8_t config = readShort(nRF24_Register::CONFIG);
  _setBit(config, CONFIG_PWR_UP);
  writeShort(nRF24_Register::CONFIG, config);
}

void nRF24::enterTxMode()
{
  uint8_t config = readShort(nRF24_Register::CONFIG);
  _setBit(config, CONFIG_PWR_UP);
  _clearBit(config, CONFIG_PRIM_RX);
  writeShort(nRF24_Register::CONFIG, config);

  _ce.set();

  delayUs(txSettling);
}

void nRF24::startListening(uint8_t pipe)
{
  enableDynamicPayloadLength(pipe);
  enableAutoAcknowledgment(pipe);
  enableDataPipe(pipe);
}

void nRF24::stopListening(uint8_t pipe)
{
  enableDynamicPayloadLength(pipe, false);
  enableAutoAcknowledgment(pipe, false);
  enableDataPipe(pipe, false);
}

bool nRF24::enqueueData(nRF24_Datagram_t& data)
{
  return writeTxFifo(data);
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
