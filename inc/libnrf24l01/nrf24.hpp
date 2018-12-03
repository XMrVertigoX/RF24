#ifndef nRF24_HPP
#define nRF24_HPP

#include <atomic>
#include <cstdint>

#include "definitions.hpp"
#include "igpio.hpp"
#include "ispi.hpp"
#include "nrf24_ll.hpp"

namespace libnrf24l01
{

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

  int enqueueData(void* bytes, size_t numBytes);

  void notify();

private:
  IGpio& _ce;

  nRF24_RxCallback_t rxCallback = NULL;
  void* rxContext = NULL;

  nRF24_TxCallback_t txCallback = NULL;
  void* txContext = NULL;

  std::atomic_bool notification = false;

  void handleDataReady(uint8_t status);
  void handleDataSent(uint8_t status);
  void handleMaxRetransmission(uint8_t status);
};

} // namespace libnrf24l01

#endif // nRF24_HPP
