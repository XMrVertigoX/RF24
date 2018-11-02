#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <type_traits>

#include <libnrf24l01/ispi.hpp>
#include <libnrf24l01/nrf24_ll.hpp>
#include <libnrf24l01/types.hpp>

using namespace std;

static const uint8_t __DUMMY_BYTE  = 0xFF;
static const uint8_t __MAX_CHANNEL = 127; // (6:0)

template <typename TYPE>
static constexpr typename underlying_type<TYPE>::type asUnderlyingType(TYPE value)
{
  return static_cast<typename underlying_type<TYPE>::type>(value);
}

nRF24_LL::nRF24_LL(ISpi& spi) : _spi(spi) {}
nRF24_LL::~nRF24_LL() {}

/*
 * ############################################################################
 * Commands
 * ############################################################################
 */

uint8_t nRF24_LL::R_REGISTER(nRF24_Register reg, uint8_t bytes[], uint8_t numBytes)
{
  uint8_t command = asUnderlyingType(nRF24_Command::R_REGISTER) | asUnderlyingType(reg);
  uint8_t status  = transmit(command, NULL, bytes, numBytes);

  return (status);
}

uint8_t nRF24_LL::W_REGISTER(nRF24_Register reg, const uint8_t bytes[], uint8_t numBytes)
{
  uint8_t command = asUnderlyingType(nRF24_Command::W_REGISTER) | asUnderlyingType(reg);
  uint8_t status  = transmit(command, bytes, NULL, numBytes);

  return (status);
}

uint8_t nRF24_LL::R_RX_PAYLOAD(uint8_t bytes[], uint8_t numBytes)
{
  uint8_t command = asUnderlyingType(nRF24_Command::R_RX_PAYLOAD);
  uint8_t status  = transmit(command, NULL, bytes, numBytes);

  return (status);
}

uint8_t nRF24_LL::W_TX_PAYLOAD(const uint8_t bytes[], uint8_t numBytes)
{
  uint8_t command = asUnderlyingType(nRF24_Command::W_TX_PAYLOAD);
  uint8_t status  = transmit(command, bytes, NULL, numBytes);

  return (status);
}

uint8_t nRF24_LL::FLUSH_TX()
{
  uint8_t command = asUnderlyingType(nRF24_Command::FLUSH_TX);
  uint8_t status  = transmit(command, NULL, NULL, 0);

  return (status);
}

uint8_t nRF24_LL::FLUSH_RX()
{
  uint8_t command = asUnderlyingType(nRF24_Command::FLUSH_RX);
  uint8_t status  = transmit(command, NULL, NULL, 0);

  return (status);
}

uint8_t nRF24_LL::REUSE_TX_PL()
{
  uint8_t command = asUnderlyingType(nRF24_Command::REUSE_TX_PL);
  uint8_t status  = transmit(command, NULL, NULL, 0);

  return (status);
}

uint8_t nRF24_LL::R_RX_PL_WID(uint8_t& payloadLength)
{
  uint8_t command = asUnderlyingType(nRF24_Command::R_RX_PL_WID);
  uint8_t status  = transmit(command, NULL, &payloadLength, 1);

  return (status);
}

uint8_t nRF24_LL::W_ACK_PAYLOAD(uint8_t pipe, const uint8_t bytes[], uint8_t numBytes)
{
  uint8_t command = asUnderlyingType(nRF24_Command::W_ACK_PAYLOAD) | pipe;
  uint8_t status  = transmit(command, bytes, NULL, numBytes);

  return (status);
}

uint8_t nRF24_LL::W_TX_PAYLOAD_NOACK(const uint8_t bytes[], uint8_t numBytes)
{
  uint8_t command = asUnderlyingType(nRF24_Command::W_TX_PAYLOAD_NOACK);
  uint8_t status  = transmit(command, bytes, NULL, numBytes);

  return (status);
}

uint8_t nRF24_LL::NOP()
{
  uint8_t command = asUnderlyingType(nRF24_Command::NOP);
  uint8_t status  = transmit(command, NULL, NULL, 0);

  return (status);
}

/*
 * ############################################################################
 * Register modification
 * ############################################################################
 */

int nRF24_LL::getPackageLossCounter()
{
  uint8_t observe_tx = readShort(nRF24_Register::OBSERVE_TX);
  observe_tx &= OBSERVE_TX_PLOS_CNT_MASK;
  observe_tx >>= OBSERVE_TX_PLOS_CNT;

  return observe_tx;
}

int nRF24_LL::getRetransmissionCounter()
{
  uint8_t observe_tx = readShort(nRF24_Register::OBSERVE_TX);
  observe_tx &= OBSERVE_TX_ARC_CNT_MASK;
  observe_tx >>= OBSERVE_TX_ARC_CNT;

  return observe_tx;
}

void nRF24_LL::enableDynamicPayloadLength(uint8_t pipe, bool enable)
{
  uint8_t dynpd = readShort(nRF24_Register::DYNPD);

  if (enable)
  {
    _setBit(dynpd, pipe);
  }
  else
  {
    _clearBit(dynpd, pipe);
  }

  writeShort(nRF24_Register::DYNPD, dynpd);
}

nRF24_CRCConfig_t nRF24_LL::getCrcConfig()
{
  uint8_t config = readShort(nRF24_Register::CONFIG);

  if (_isBitSet(config, CONFIG_EN_CRC) == false)
  {
    return (nRF24_CRCConfig_t::CRC_DISABLED);
  }

  if (_isBitSet(config, CONFIG_CRCO))
  {
    return (nRF24_CRCConfig_t::CRC_2Bytes);
  }
  else
  {
    return (nRF24_CRCConfig_t::CRC_1Byte);
  }
}

void nRF24_LL::setCrcConfig(nRF24_CRCConfig_t crcConfig)
{
  uint8_t config = readShort(nRF24_Register::CONFIG);

  switch (crcConfig)
  {
  case nRF24_CRCConfig_t::CRC_DISABLED:
  {
    _clearBit(config, CONFIG_EN_CRC);
    _clearBit(config, CONFIG_CRCO);
    break;
  }
  case nRF24_CRCConfig_t::CRC_1Byte:
  {
    _setBit(config, CONFIG_EN_CRC);
    _clearBit(config, CONFIG_CRCO);
    break;
  }
  case nRF24_CRCConfig_t::CRC_2Bytes:
  {
    _setBit(config, CONFIG_EN_CRC);
    _setBit(config, CONFIG_CRCO);
    break;
  }
  }

  writeShort(nRF24_Register::CONFIG, config);
}

uint8_t nRF24_LL::getAddressWidth()
{
  uint8_t setup_aw = readShort(nRF24_Register::SETUP_AW);

  setup_aw &= SETUP_AW_MASK;
  setup_aw += 2;

  return setup_aw;
}

void nRF24_LL::setAddressWidth(uint8_t addressWidth)
{
  if (addressWidth < 3 || addressWidth > 5)
  {
    return;
  }

  addressWidth -= 2;

  writeShort(nRF24_Register::SETUP_AW, addressWidth);
}

uint8_t nRF24_LL::getChannel()
{
  return readShort(nRF24_Register::RF_CH);
}

void nRF24_LL::setChannel(uint8_t channel)
{
  if (channel > __MAX_CHANNEL)
  {
    return;
  }

  writeShort(nRF24_Register::RF_CH, channel);
}

nRF24_DataRate_t nRF24_LL::getDataRate()
{
  uint8_t rf_setup = readShort(nRF24_Register::RF_SETUP);

  if (_isBitSet(rf_setup, RF_SETUP_RF_DR_LOW))
  {
    return (nRF24_DataRate_t::DR_250KBPS);
  }

  if (_isBitSet(rf_setup, RF_SETUP_RF_DR_HIGH))
  {
    return (nRF24_DataRate_t::DR_2MBPS);
  }

  return (nRF24_DataRate_t::DR_1MBPS);
}

void nRF24_LL::setDataRate(nRF24_DataRate_t dataRate)
{
  uint8_t rf_setup = readShort(nRF24_Register::RF_SETUP);

  switch (dataRate)
  {
  case nRF24_DataRate_t::DR_1MBPS:
  {
    _clearBit(rf_setup, RF_SETUP_RF_DR_LOW);
    _clearBit(rf_setup, RF_SETUP_RF_DR_HIGH);
    break;
  }
  case nRF24_DataRate_t::DR_2MBPS:
  {
    _clearBit(rf_setup, RF_SETUP_RF_DR_LOW);
    _setBit(rf_setup, RF_SETUP_RF_DR_HIGH);
    break;
  }
  case nRF24_DataRate_t::DR_250KBPS:
  {
    _setBit(rf_setup, RF_SETUP_RF_DR_LOW);
    _clearBit(rf_setup, RF_SETUP_RF_DR_HIGH);
    break;
  }
  }

  writeShort(nRF24_Register::RF_SETUP, rf_setup);
}

nRF24_OutputPower_t nRF24_LL::getOutputPower()
{
  uint8_t rf_setup = readShort(nRF24_Register::RF_SETUP);

  rf_setup &= RF_SETUP_RF_PWR_MASK;
  rf_setup >>= RF_SETUP_RF_PWR;

  switch (rf_setup)
  {
  case 0:
  {
    return (nRF24_OutputPower_t::PWR_18dBm);
    break;
  }
  case 1:
  {
    return (nRF24_OutputPower_t::PWR_12dBm);
    break;
  }
  case 2:
  {
    return (nRF24_OutputPower_t::PWR_6dBm);
    break;
  }
  default:
  {
    return (nRF24_OutputPower_t::PWR_0dBm);
    break;
  }
  }
}

void nRF24_LL::setOutputPower(nRF24_OutputPower_t outputPower)
{
  uint8_t rf_setup = readShort(nRF24_Register::RF_SETUP);

  switch (outputPower)
  {
  case nRF24_OutputPower_t::PWR_18dBm:
  {
    _clearBit(rf_setup, 1);
    _clearBit(rf_setup, 2);
  }
  break;
  case nRF24_OutputPower_t::PWR_12dBm:
  {
    _setBit(rf_setup, 1);
    _clearBit(rf_setup, 2);
  }
  break;
  case nRF24_OutputPower_t::PWR_6dBm:
  {
    _clearBit(rf_setup, 1);
    _setBit(rf_setup, 2);
  }
  break;
  case nRF24_OutputPower_t::PWR_0dBm:
  {
    _setBit(rf_setup, 1);
    _setBit(rf_setup, 2);
  }
  break;
  }

  writeShort(nRF24_Register::RF_SETUP, rf_setup);
}

uint8_t nRF24_LL::getRetryCount()
{
  uint8_t setup_retr = readShort(nRF24_Register::SETUP_RETR);
  setup_retr &= SETUP_RETR_ARC_MASK;
  setup_retr >>= SETUP_RETR_ARC;

  return (setup_retr);
}

void nRF24_LL::setRetryCount(uint8_t count)
{
  if (count > 0xF)
  {
    return;
  }

  uint8_t setup_retr = readShort(nRF24_Register::SETUP_RETR);
  setup_retr &= ~(SETUP_RETR_ARC_MASK);
  setup_retr |= (count << SETUP_RETR_ARC);
  writeShort(nRF24_Register::SETUP_RETR, setup_retr);
}

uint8_t nRF24_LL::getRetryDelay()
{
  uint8_t setup_retr = readShort(nRF24_Register::SETUP_RETR);
  setup_retr &= SETUP_RETR_ARD_MASK;
  setup_retr >>= SETUP_RETR_ARD;

  return (setup_retr);
}

void nRF24_LL::setRetryDelay(uint8_t delay)
{
  if (delay > 0xF)
  {
    return;
  }

  uint8_t setup_retr = readShort(nRF24_Register::SETUP_RETR);
  setup_retr &= ~(SETUP_RETR_ARD_MASK);
  setup_retr |= (delay << SETUP_RETR_ARD);
  writeShort(nRF24_Register::SETUP_RETR, setup_retr);
}

uint32_t nRF24_LL::readRxBaseAddress(uint8_t pipe)
{
  uint8_t addressWidth = getAddressWidth();
  uint8_t buffer[addressWidth];

  if (pipe == 0)
  {
    R_REGISTER(nRF24_Register::RX_ADDR_P0, buffer, addressWidth);
  }
  else
  {
    R_REGISTER(nRF24_Register::RX_ADDR_P1, buffer, addressWidth);
  }

  return (reinterpret_cast<uint32_t>(buffer + 1));
}

void nRF24_LL::writeRxBaseAddress(uint8_t pipe, uint32_t baseAddress)
{
  uint8_t addressWidth = getAddressWidth();
  uint8_t buffer[addressWidth];

  if (pipe == 0)
  {
    R_REGISTER(nRF24_Register::RX_ADDR_P0, buffer, addressWidth);
  }
  else
  {
    R_REGISTER(nRF24_Register::RX_ADDR_P1, buffer, addressWidth);
  }

  memcpy(buffer + 1, &baseAddress, addressWidth - 1);

  if (pipe == 0)
  {
    W_REGISTER(nRF24_Register::RX_ADDR_P0, buffer, addressWidth);
  }
  else
  {
    W_REGISTER(nRF24_Register::RX_ADDR_P1, buffer, addressWidth);
  }
}

uint32_t nRF24_LL::readTxBaseAddress()
{
  uint8_t addressWidth = getAddressWidth();
  uint8_t buffer[addressWidth];

  R_REGISTER(nRF24_Register::TX_ADDR, buffer, addressWidth);

  return (reinterpret_cast<uint32_t>(buffer + 1));
}

void nRF24_LL::writeTxBaseAddress(uint32_t baseAddress)
{
  uint8_t addressWidth = getAddressWidth();
  uint8_t buffer[addressWidth];

  R_REGISTER(nRF24_Register::TX_ADDR, buffer, addressWidth);
  memcpy(buffer + 1, &baseAddress, addressWidth - 1);
  W_REGISTER(nRF24_Register::TX_ADDR, buffer, addressWidth);
}

uint8_t nRF24_LL::readRxAddress(uint8_t pipe)
{
  switch (pipe)
  {
  case 0:
  {
    return readShort(nRF24_Register::RX_ADDR_P0);
    break;
  }
  case 1:
  {
    return readShort(nRF24_Register::RX_ADDR_P1);
    break;
  }
  case 2:
  {
    return readShort(nRF24_Register::RX_ADDR_P2);
    break;
  }
  case 3:
  {
    return readShort(nRF24_Register::RX_ADDR_P3);
    break;
  }
  case 4:
  {
    return readShort(nRF24_Register::RX_ADDR_P4);
    break;
  }
  case 5:
  {
    return readShort(nRF24_Register::RX_ADDR_P5);
    break;
  }
  default:
  {
    return 0;
    break;
  }
  }
}

void nRF24_LL::writeRxAddress(uint8_t pipe, uint8_t address)
{
  switch (pipe)
  {
  case 0:
  {
    writeShort(nRF24_Register::RX_ADDR_P0, address);
    break;
  }
  case 1:
  {
    writeShort(nRF24_Register::RX_ADDR_P1, address);
    break;
  }
  case 2:
  {
    writeShort(nRF24_Register::RX_ADDR_P2, address);
    break;
  }
  case 3:
  {
    writeShort(nRF24_Register::RX_ADDR_P3, address);
    break;
  }
  case 4:
  {
    writeShort(nRF24_Register::RX_ADDR_P4, address);
    break;
  }
  case 5:
  {
    writeShort(nRF24_Register::RX_ADDR_P5, address);
    break;
  }
  }
}

uint8_t nRF24_LL::readTxAddress()
{
  return readShort(nRF24_Register::TX_ADDR);
}

void nRF24_LL::writeTxAddress(uint8_t address)
{
  writeShort(nRF24_Register::TX_ADDR, address);
}

void nRF24_LL::enableAutoAcknowledgment(uint8_t pipe, bool enable)
{
  uint8_t en_aa = readShort(nRF24_Register::EN_AA);

  if (enable)
  {
    _setBit(en_aa, pipe);
  }
  else
  {
    _clearBit(en_aa, pipe);
  }

  writeShort(nRF24_Register::EN_AA, en_aa);
}

void nRF24_LL::enableDataPipe(uint8_t pipe, bool enable)
{
  uint8_t en_rxaddr = readShort(nRF24_Register::EN_RXADDR);

  if (enable)
  {
    _setBit(en_rxaddr, pipe);
  }
  else
  {
    _clearBit(en_rxaddr, pipe);
  }

  writeShort(nRF24_Register::EN_RXADDR, en_rxaddr);
}

/*
 * ############################################################################
 * Utility functions
 * ############################################################################
 */

uint8_t nRF24_LL::readShort(nRF24_Register reg)
{
  uint8_t tmp;

  R_REGISTER(reg, &tmp);

  return tmp;
}

void nRF24_LL::writeShort(nRF24_Register reg, uint8_t val)
{
  W_REGISTER(reg, &val);
}

int nRF24_LL::readRxFifo(nRF24_Datagram_t& data)
{
  uint8_t fifo_status = readShort(nRF24_Register::FIFO_STATUS);

  if (_isBitSet(fifo_status, FIFO_STATUS_RX_EMPTY))
  {
    return EXIT_FAILURE;
  }

  R_RX_PL_WID(data.numBytes);

  if (data.numBytes > rxFifoSize)
  {
    return EXIT_FAILURE;
  }

  R_RX_PAYLOAD(data.bytes, data.numBytes);

  return EXIT_SUCCESS;
}

int nRF24_LL::writeTxFifo(nRF24_Datagram_t& data)
{
  uint8_t fifo_status = readShort(nRF24_Register::FIFO_STATUS);

  if (_isBitSet(fifo_status, FIFO_STATUS_TX_FULL))
  {
    return EXIT_FAILURE;
  }

  W_TX_PAYLOAD(data.bytes, data.numBytes);

  return EXIT_SUCCESS;
}

void nRF24_LL::clearInterrupts()
{
  writeShort(nRF24_Register::STATUS, (STATUS_MAX_RT_MASK | STATUS_RX_DR_MASK | STATUS_TX_DS_MASK));
}

/*
 * ############################################################################
 * Private
 * ############################################################################
 */

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
    memset(&buffer[1], __DUMMY_BYTE, numBytes);
  }

  _spi.transceive(buffer, buffer, numBytes + 1);

  if (rxBytes != NULL)
  {
    memcpy(rxBytes, &buffer[1], numBytes);
  }

  return buffer[0];
}
