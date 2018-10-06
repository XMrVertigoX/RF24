#ifndef RF24_HPP
#define RF24_HPP

#include <cstdint>

#include <nrf24l01/circularbuffer.hpp>
#include <nrf24l01/igpio.hpp>
#include <nrf24l01/ispi.hpp>
#include <nrf24l01/rf24_ll.hpp>
#include <nrf24l01/types.hpp>

// using namespace std;

static inline void delayUs(uint32_t us) {}

class RF24 : public RF24_LL
{
public:
  RF24(ISpi& spi, IGpio& ce);
  virtual ~RF24();

  void setup();
  void loop();

  void enterRxMode();
  void enterShutdownMode();
  void enterStandbyMode();
  void enterTxMode();

  void setRxCallback(RF24_RxCallback_t callback, void* context);
  void setTxCallback(RF24_TxCallback_t callback, void* context);

  void startListening(uint8_t pipe = 0);
  void stopListening(uint8_t pipe = 0);

  bool enqueueData(RF24_Datagram_t& data);

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

  RF24_CRCConfig_t getCrcConfig();
  void setCrcConfig(RF24_CRCConfig_t crcConfig);

  RF24_DataRate_t getDataRate();
  void setDataRate(RF24_DataRate_t dataRate);

  RF24_OutputPower_t getOutputPower();
  void setOutputPower(RF24_OutputPower_t level);

  uint8_t getRetryCount();
  void setRetryCount(uint8_t count);

  uint8_t getRetryDelay();
  void setRetryDelay(uint8_t delay);

private:
  IGpio& _ce;

  RF24_RxCallback_t rxCallback = NULL;
  void* rxContext = NULL;

  RF24_TxCallback_t txCallback = NULL;
  void* txContext = NULL;

  int addressLength = 5;

  CircularBuffer<RF24_Datagram_t> rxBuffer = CircularBuffer<RF24_Datagram_t>(1);
  CircularBuffer<RF24_Datagram_t> txBuffer = CircularBuffer<RF24_Datagram_t>(1);

  uint8_t readShort(RF24_Register reg);
  void writeShort(RF24_Register reg, uint8_t val);

  void handleDataReady(uint8_t status);
  void handleDataSent(uint8_t status);
  void handleMaxRetransmission(uint8_t status);

  int readRxFifo(RF24_Datagram_t& data);
  int writeTxFifo(RF24_Datagram_t& data);

  int getPackageLossCounter();
};

#endif // RF24_HPP
