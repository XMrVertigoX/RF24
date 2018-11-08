#ifndef nRF24_HPP
#define nRF24_HPP

#include <atomic>
#include <cstdint>

#include <libnrf24l01/circularbuffer.hpp>
#include <libnrf24l01/igpio.hpp>
#include <libnrf24l01/ispi.hpp>
#include <libnrf24l01/nrf24_ll.hpp>
#include <libnrf24l01/types.hpp>

using namespace std;

class nRF24 : public nRF24_LL
{
public:
  nRF24(ISpi& spi, IGpio& ce);
  virtual ~nRF24();

  void init();
  void process();

  void enterRxMode();
  void enterShutdownMode();
  void enterStandbyMode();
  void enterTxMode();

  void setRxCallback(nRF24_RxCallback_t callback, void* context);
  void setTxCallback(nRF24_TxCallback_t callback, void* context);

  void startListening(uint8_t pipe = 0);
  void stopListening(uint8_t pipe = 0);

  bool enqueueData(nRF24_Datagram_t& data);

  void notify();

private:
  IGpio& _ce;

  nRF24_RxCallback_t rxCallback = NULL;
  void* rxContext = NULL;

  nRF24_TxCallback_t txCallback = NULL;
  void* txContext = NULL;

  atomic_bool notification = false;

  CircularBuffer<nRF24_Datagram_t> rxBuffer = CircularBuffer<nRF24_Datagram_t>(1);

  void handleDataReady(uint8_t status);
  void handleDataSent(uint8_t status);
  void handleMaxRetransmission(uint8_t status);
};

#endif // nRF24_HPP
