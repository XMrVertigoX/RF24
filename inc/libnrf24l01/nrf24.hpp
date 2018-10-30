#ifndef nRF24_HPP
#define nRF24_HPP

#include <cstdint>

#include <libnrf24l01/circularbuffer.hpp>
#include <libnrf24l01/igpio.hpp>
#include <libnrf24l01/ispi.hpp>
#include <libnrf24l01/nrf24_ll.hpp>
#include <libnrf24l01/types.hpp>

using namespace std;

static inline void delayUs(uint32_t us) {}

class nRF24 : private nRF24_LL
{
public:
  virtual ~nRF24() = default;

  nRF24(ISpi& spi, IGpio& ce);

  void setup();
  void loop();

  void enterRxMode();
  void enterShutdownMode();
  void enterStandbyMode();
  void enterTxMode();

  void setRxCallback(nRF24_RxCallback_t callback, void* context);
  void setTxCallback(nRF24_TxCallback_t callback, void* context);

  void startListening(uint8_t pipe = 0);
  void stopListening(uint8_t pipe = 0);

  bool enqueueData(nRF24_Datagram_t& data);

  void enableDynamicPayloadLength(uint8_t pipe, bool enable = true);
  void enableDataPipe(uint8_t pipe, bool enable = true);
  void enableAutoAcknowledgment(uint8_t pipe, bool enable = true);

  int getRetransmissionCounter();

  uint32_t readRxBaseAddress(uint8_t pipe);
  void writeRxBaseAddress(uint8_t pipe, uint32_t baseAddress);

  uint32_t readTxBaseAddress();
  void writeTxBaseAddress(uint32_t baseAddress);

  uint8_t readRxAddress(uint8_t pipe);
  void writeRxAddress(uint8_t pipe, uint8_t address);

  uint8_t readTxAddress();
  void writeTxAddress(uint8_t address);

  uint8_t getChannel();
  void setChannel(uint8_t channel);

  uint8_t getRetryCount();
  void setRetryCount(uint8_t count);

  uint8_t getRetryDelay();
  void setRetryDelay(uint8_t delay);

  nRF24_CRCConfig_t getCrcConfig();
  void setCrcConfig(nRF24_CRCConfig_t crcConfig);

  nRF24_DataRate_t getDataRate();
  void setDataRate(nRF24_DataRate_t dataRate);

  nRF24_OutputPower_t getOutputPower();
  void setOutputPower(nRF24_OutputPower_t level);

private:
  IGpio& _ce;

  nRF24_RxCallback_t rxCallback = NULL;
  void* rxContext = NULL;

  nRF24_TxCallback_t txCallback = NULL;
  void* txContext = NULL;

  int addressLength = 5;

  CircularBuffer<nRF24_Datagram_t> rxBuffer = CircularBuffer<nRF24_Datagram_t>(3);
  CircularBuffer<nRF24_Datagram_t> txBuffer = CircularBuffer<nRF24_Datagram_t>(3);

  uint8_t readShort(nRF24_Register reg);
  void writeShort(nRF24_Register reg, uint8_t val);

  void handleDataReady(uint8_t status);
  void handleDataSent(uint8_t status);
  void handleMaxRetransmission(uint8_t status);

  int readRxFifo(nRF24_Datagram_t& data);
  int writeTxFifo(nRF24_Datagram_t& data);

  int getPackageLossCounter();
};

#endif // nRF24_HPP
