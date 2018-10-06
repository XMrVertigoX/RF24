#ifndef RF24_LL_HPP
#define RF24_LL_HPP

#include <cstdint>

#include <nrf24l01/ispi.hpp>
#include <nrf24l01/types.hpp>

class RF24_LL
{
protected:
  RF24_LL(ISpi& spi);
  virtual ~RF24_LL();

  uint8_t R_REGISTER(RF24_Register reg, uint8_t bytes[], uint8_t numBytes = 1);
  uint8_t W_REGISTER(RF24_Register reg, const uint8_t bytes[], uint8_t numBytes = 1);
  uint8_t R_RX_PAYLOAD(uint8_t bytes[], uint8_t numBytes);
  uint8_t W_TX_PAYLOAD(const uint8_t bytes[], uint8_t numBytes);
  uint8_t FLUSH_TX();
  uint8_t FLUSH_RX();
  uint8_t REUSE_TX_PL();
  uint8_t R_RX_PL_WID(uint8_t& payloadLength);
  uint8_t W_ACK_PAYLOAD(uint8_t pipe, const uint8_t bytes[], uint8_t numBytes);
  uint8_t W_TX_PAYLOAD_NOACK(const uint8_t bytes[], uint8_t numBytes);
  uint8_t NOP();

private:
  ISpi& _spi;

  uint8_t transmit(uint8_t command, const uint8_t txBytes[], uint8_t rxBytes[], uint8_t numBytes);
};

#endif /* RF24_LL_HPP */
