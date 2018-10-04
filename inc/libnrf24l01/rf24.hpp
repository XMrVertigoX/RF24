#ifndef RF24_HPP
#define RF24_HPP

#include <cstdint>

#include <libnrf24l01/circularbuffer.hpp>
#include <libnrf24l01/igpio.hpp>
#include <libnrf24l01/ispi.hpp>
#include <libnrf24l01/rf24_ll.hpp>
#include <libnrf24l01/types.hpp>

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

  RF24_Status startListening(uint8_t pipe = 0);
  RF24_Status stopListening(uint8_t pipe = 0);

  bool enqueueData(RF24_Datagram_t& data);

  RF24_Status enableDynamicPayloadLength(uint8_t pipe, bool enable = true);
  RF24_Status enableDataPipe(uint8_t pipe, bool enable = true);
  RF24_Status enableAutoAcknowledgment(uint8_t pipe, bool enable = true);

  RF24_Status readRxBaseAddress(uint8_t pipe, uint32_t& baseAddress);
  RF24_Status writeRxBaseAddress(uint8_t pipe, uint32_t baseAddress);

  RF24_Status readTxBaseAddress(uint32_t& baseAddress);
  RF24_Status writeTxBaseAddress(uint32_t baseAddress);

  RF24_Status readRxAddress(uint8_t pipe, uint8_t& address);
  RF24_Status writeRxAddress(uint8_t pipe, uint8_t address);

  RF24_Status readTxAddress(uint8_t& address);
  RF24_Status writeTxAddress(uint8_t address);

  uint8_t getChannel();
  RF24_Status setChannel(uint8_t channel);

  RF24_CRCConfig_t getCrcConfig();
  RF24_Status setCrcConfig(RF24_CRCConfig_t crcConfig);

  RF24_DataRate_t getDataRate();
  RF24_Status setDataRate(RF24_DataRate_t dataRate);

  RF24_OutputPower_t getOutputPower();
  RF24_Status setOutputPower(RF24_OutputPower_t level);

  uint8_t getRetryCount();
  RF24_Status setRetryCount(uint8_t count);

  uint8_t getRetryDelay();
  RF24_Status setRetryDelay(uint8_t delay);

private:
  IGpio& ce;

  RF24_RxCallback_t rxCallback = NULL;
  void* rxContext = NULL;

  RF24_TxCallback_t txCallback = NULL;
  void* txContext = NULL;

  int addressLength = 5;

  CircularBuffer<RF24_Datagram_t> rxBuffer = CircularBuffer<RF24_Datagram_t>(3);
  CircularBuffer<RF24_Datagram_t> txBuffer = CircularBuffer<RF24_Datagram_t>(3);

  void
  handleDataReady(uint8_t status);
  void handleDataSent(uint8_t status);

  int readRxFifo(RF24_Datagram_t& data);
  int writeTxFifo(RF24_Datagram_t& data);

  int getRetransmissionCounter();
  int getPackageLossCounter();
};

#endif // RF24_HPP
