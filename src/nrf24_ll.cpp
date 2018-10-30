#include <cstdint>
#include <cstring>
#include <type_traits>

#include <libnrf24l01/ispi.hpp>
#include <libnrf24l01/nrf24_ll.hpp>
#include <libnrf24l01/types.hpp>

using namespace std;

static const uint8_t dummyByte = 0xFF;

template <typename TYPE>
static constexpr typename underlying_type<TYPE>::type asUnderlyingType(TYPE value)
{
  return static_cast<typename underlying_type<TYPE>::type>(value);
}

nRF24_LL::nRF24_LL(ISpi& spi) : _spi(spi) {}

uint8_t nRF24_LL::R_REGISTER(nRF24_Register reg, uint8_t bytes[], uint8_t numBytes)
{
  uint8_t command = asUnderlyingType(nRF24_Command::R_REGISTER) | asUnderlyingType(reg);
  uint8_t status = transmit(command, NULL, bytes, numBytes);

  return (status);
}

uint8_t nRF24_LL::W_REGISTER(nRF24_Register reg, const uint8_t bytes[], uint8_t numBytes)
{
  uint8_t command = asUnderlyingType(nRF24_Command::W_REGISTER) | asUnderlyingType(reg);
  uint8_t status = transmit(command, bytes, NULL, numBytes);

  return (status);
}

uint8_t nRF24_LL::R_RX_PAYLOAD(uint8_t bytes[], uint8_t numBytes)
{
  uint8_t command = asUnderlyingType(nRF24_Command::R_RX_PAYLOAD);
  uint8_t status = transmit(command, NULL, bytes, numBytes);

  return (status);
}

uint8_t nRF24_LL::W_TX_PAYLOAD(const uint8_t bytes[], uint8_t numBytes)
{
  uint8_t command = asUnderlyingType(nRF24_Command::W_TX_PAYLOAD);
  uint8_t status = transmit(command, bytes, NULL, numBytes);

  return (status);
}

uint8_t nRF24_LL::FLUSH_TX()
{
  uint8_t command = asUnderlyingType(nRF24_Command::FLUSH_TX);
  uint8_t status = transmit(command, NULL, NULL, 0);

  return (status);
}

uint8_t nRF24_LL::FLUSH_RX()
{
  uint8_t command = asUnderlyingType(nRF24_Command::FLUSH_RX);
  uint8_t status = transmit(command, NULL, NULL, 0);

  return (status);
}

uint8_t nRF24_LL::REUSE_TX_PL()
{
  uint8_t command = asUnderlyingType(nRF24_Command::REUSE_TX_PL);
  uint8_t status = transmit(command, NULL, NULL, 0);

  return (status);
}

uint8_t nRF24_LL::R_RX_PL_WID(uint8_t& payloadLength)
{
  uint8_t command = asUnderlyingType(nRF24_Command::R_RX_PL_WID);
  uint8_t status = transmit(command, NULL, &payloadLength, 1);

  return (status);
}

uint8_t nRF24_LL::W_ACK_PAYLOAD(uint8_t pipe, const uint8_t bytes[], uint8_t numBytes)
{
  uint8_t command = asUnderlyingType(nRF24_Command::W_ACK_PAYLOAD) | pipe;
  uint8_t status = transmit(command, bytes, NULL, numBytes);

  return (status);
}

uint8_t nRF24_LL::W_TX_PAYLOAD_NOACK(const uint8_t bytes[], uint8_t numBytes)
{
  uint8_t command = asUnderlyingType(nRF24_Command::W_TX_PAYLOAD_NOACK);
  uint8_t status = transmit(command, bytes, NULL, numBytes);

  return (status);
}

uint8_t nRF24_LL::NOP()
{
  uint8_t command = asUnderlyingType(nRF24_Command::NOP);
  uint8_t status = transmit(command, NULL, NULL, 0);

  return (status);
}

// ----- private --------------------------------------------------------------

uint8_t nRF24_LL::transmit(
    uint8_t command,
    const uint8_t txBytes[],
    uint8_t rxBytes[],
    uint8_t numBytes)
{
  uint8_t buffer[numBytes + 1];
  buffer[0] = command;

  if (txBytes != NULL)
  {
    memcpy(&buffer[1], txBytes, numBytes);
  }
  else
  {
    memset(&buffer[1], dummyByte, numBytes);
  }

  _spi.transmit_receive(buffer, buffer, numBytes + 1);

  if (rxBytes != NULL)
  {
    memcpy(rxBytes, &buffer[1], numBytes);
  }

  return buffer[0];
}
