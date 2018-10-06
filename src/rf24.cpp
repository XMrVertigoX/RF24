#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <nrf24l01/igpio.hpp>
#include <nrf24l01/ispi.hpp>
#include <nrf24l01/rf24.hpp>

using namespace std;

#define BOUNCE(expression, statement) \
  if (expression)                     \
  return (statement)

static const uint8_t maxChannels = 120;
static const uint8_t numPipes = 5;

static inline void __attribute__((weak)) delayUs(int us) {}

static inline void _clearBit(uint8_t& byte, uint8_t bit)
{
  byte and_eq compl((1 << bit));
}

static inline void _setBit(uint8_t& byte, uint8_t bit)
{
  byte or_eq (1 << bit);
}

static inline bool _isBitSet(uint8_t byte, uint8_t bit)
{
  return (byte bitand (1 << bit));
}

static inline void _clip(uint8_t& value, uint8_t max)
{
  if (value > max)
  {
    value = max;
  }
}

static inline uint8_t _getPipe(uint8_t status)
{
  status and_eq STATUS_RX_P_NO_MASK;
  status >>= STATUS_RX_P_NO;

  return (status);
}

RF24::RF24(ISpi& spi, IGpio& ce) : RF24_LL(spi), _ce(ce) {}

RF24::~RF24() {}

void RF24::setup()
{
  // Enable dynamic payload length
  uint8_t feature = readShort(RF24_Register::FEATURE);
  _clearBit(feature, FEATURE_EN_DYN_ACK);
  _clearBit(feature, FEATURE_EN_ACK_PAY);
  _setBit(feature, FEATURE_EN_DPL);
  writeShort(RF24_Register::FEATURE, feature);

  // Clear interrupts
  uint8_t status = readShort(RF24_Register::STATUS);
  _setBit(status, STATUS_MAX_RT);
  _setBit(status, STATUS_RX_DR);
  _setBit(status, STATUS_TX_DS);
  writeShort(RF24_Register::STATUS, status);

  FLUSH_RX();
  FLUSH_TX();
}

void RF24::loop()
{
  uint8_t status = NOP();

  if (_isBitSet(status, STATUS_RX_DR))
  {
    handleDataReady(status);
    writeShort(RF24_Register::STATUS, STATUS_RX_DR);
  }
  else if (_isBitSet(status, STATUS_TX_DS))
  {
    handleDataSent(status);
    writeShort(RF24_Register::STATUS, STATUS_RX_DR);
  }
  else if (_isBitSet(status, STATUS_MAX_RT))
  {
    handleMaxRetransmission(status);
    writeShort(RF24_Register::STATUS, STATUS_RX_DR);
  }

  if (!rxBuffer.empty())
  {
    if (rxCallback)
    {
      rxCallback(rxBuffer.front(), rxContext);
    }

    rxBuffer.pop_front();
  }

  if (!txBuffer.empty())
  {
    if (!writeTxFifo(txBuffer.front()))
    {
      txBuffer.pop_front();
    }
  }
}

void RF24::handleDataReady(uint8_t status)
{
  RF24_Datagram_t data;
  data.pipe = _getPipe(status);
  int error = readRxFifo(data);

  if (error)
  {
    FLUSH_RX();
  }
  else
  {
    rxBuffer.push_back(data);
  }
}

void RF24::handleDataSent(uint8_t status)
{
  if (txCallback)
  {
    txCallback(txContext);
  }
}

void RF24::handleMaxRetransmission(uint8_t status)
{
  FLUSH_TX();
}

int RF24::readRxFifo(RF24_Datagram_t& data)
{
  R_RX_PL_WID(data.numBytes);
  BOUNCE(data.numBytes > rxFifoSize, EXIT_FAILURE);
  R_RX_PAYLOAD(data.bytes, data.numBytes);

  return EXIT_SUCCESS;
}

int RF24::writeTxFifo(RF24_Datagram_t& data)
{
  BOUNCE(_isBitSet(NOP(), STATUS_TX_FULL), EXIT_FAILURE);
  W_TX_PAYLOAD(data.bytes, data.numBytes);

  return EXIT_SUCCESS;
}

void RF24::enterRxMode()
{
  uint8_t config = readShort(RF24_Register::CONFIG);
  _setBit(config, CONFIG_PWR_UP);
  _setBit(config, CONFIG_PRIM_RX);
  writeShort(RF24_Register::CONFIG, config);

  _ce.set();

  delayUs(rxSettling);
}

void RF24::enterShutdownMode()
{
  _ce.clear();

  uint8_t config = readShort(RF24_Register::CONFIG);
  _clearBit(config, CONFIG_PWR_UP);
  writeShort(RF24_Register::CONFIG, config);
}

void RF24::enterStandbyMode()
{
  _ce.clear();

  uint8_t config = readShort(RF24_Register::CONFIG);
  _setBit(config, CONFIG_PWR_UP);
  writeShort(RF24_Register::CONFIG, config);
}

void RF24::enterTxMode()
{
  uint8_t config = readShort(RF24_Register::CONFIG);
  _setBit(config, CONFIG_PWR_UP);
  _clearBit(config, CONFIG_PRIM_RX);
  writeShort(RF24_Register::CONFIG, config);

  _ce.set();

  delayUs(txSettling);
}

void RF24::startListening(uint8_t pipe)
{
  enableDynamicPayloadLength(pipe);
  enableAutoAcknowledgment(pipe);
  enableDataPipe(pipe);
}

void RF24::stopListening(uint8_t pipe)
{
  enableDynamicPayloadLength(pipe, false);
  enableAutoAcknowledgment(pipe, false);
  enableDataPipe(pipe, false);
}

bool RF24::enqueueData(RF24_Datagram_t& data)
{
  return txBuffer.push_back(data);
}

int RF24::getPackageLossCounter()
{
  uint8_t observe_tx = readShort(RF24_Register::OBSERVE_TX);
  observe_tx and_eq OBSERVE_TX_PLOS_CNT_MASK;
  observe_tx >>= OBSERVE_TX_PLOS_CNT;

  return observe_tx;
}

int RF24::getRetransmissionCounter()
{
  uint8_t observe_tx = readShort(RF24_Register::OBSERVE_TX);
  observe_tx and_eq OBSERVE_TX_ARC_CNT_MASK;
  observe_tx >>= OBSERVE_TX_ARC_CNT;

  return observe_tx;
}

void RF24::enableDynamicPayloadLength(uint8_t pipe, bool enable)
{
  uint8_t dynpd = readShort(RF24_Register::DYNPD);

  if (enable)
  {
    _setBit(dynpd, pipe);
  }
  else
  {
    _clearBit(dynpd, pipe);
  }

  writeShort(RF24_Register::DYNPD, dynpd);
}

RF24_CRCConfig_t RF24::getCrcConfig()
{
  uint8_t config = readShort(RF24_Register::CONFIG);

  if (_isBitSet(config, CONFIG_EN_CRC) == false)
  {
    return (RF24_CRCConfig_t::CRC_DISABLED);
  }

  if (_isBitSet(config, CONFIG_CRCO))
  {
    return (RF24_CRCConfig_t::CRC_2Bytes);
  }
  else
  {
    return (RF24_CRCConfig_t::CRC_1Byte);
  }
}

void RF24::setCrcConfig(RF24_CRCConfig_t crcConfig)
{
  uint8_t config = readShort(RF24_Register::CONFIG);

  switch (crcConfig)
  {
  case RF24_CRCConfig_t::CRC_DISABLED:
  {
    _clearBit(config, CONFIG_EN_CRC);
  }
  break;
  case RF24_CRCConfig_t::CRC_1Byte:
  {
    _setBit(config, CONFIG_EN_CRC);
    _clearBit(config, CONFIG_CRCO);
  }
  break;
  case RF24_CRCConfig_t::CRC_2Bytes:
  {
    _setBit(config, CONFIG_EN_CRC);
    _setBit(config, CONFIG_CRCO);
  }
  break;
  }

  writeShort(RF24_Register::CONFIG, config);
}

uint8_t RF24::getChannel()
{
  return readShort(RF24_Register::RF_CH);
}

void RF24::setChannel(uint8_t channel)
{
  if (channel > maxChannels)
  {
    return;
  }

  writeShort(RF24_Register::RF_CH, channel);
}

RF24_DataRate_t RF24::getDataRate()
{
  uint8_t rf_setup = readShort(RF24_Register::RF_SETUP);

  if (_isBitSet(rf_setup, RF_SETUP_RF_DR_LOW))
  {
    return (RF24_DataRate_t::DR_250KBPS);
  }

  if (_isBitSet(rf_setup, RF_SETUP_RF_DR_HIGH))
  {
    return (RF24_DataRate_t::DR_2MBPS);
  }

  return (RF24_DataRate_t::DR_1MBPS);
}

void RF24::setDataRate(RF24_DataRate_t dataRate)
{
  uint8_t rf_setup = readShort(RF24_Register::RF_SETUP);

  switch (dataRate)
  {
  case RF24_DataRate_t::DR_1MBPS:
  {
    _clearBit(rf_setup, RF_SETUP_RF_DR_LOW);
    _clearBit(rf_setup, RF_SETUP_RF_DR_HIGH);
  }
  break;
  case RF24_DataRate_t::DR_2MBPS:
  {
    _clearBit(rf_setup, RF_SETUP_RF_DR_LOW);
    _setBit(rf_setup, RF_SETUP_RF_DR_HIGH);
  }
  break;
  case RF24_DataRate_t::DR_250KBPS:
  {
    _setBit(rf_setup, RF_SETUP_RF_DR_LOW);
    _clearBit(rf_setup, RF_SETUP_RF_DR_HIGH);
  }
  break;
  }

  writeShort(RF24_Register::RF_SETUP, rf_setup);
}

RF24_OutputPower_t RF24::getOutputPower()
{
  uint8_t rf_setup = readShort(RF24_Register::RF_SETUP);

  rf_setup and_eq RF_SETUP_RF_PWR_MASK;
  rf_setup >>= RF_SETUP_RF_PWR;

  switch (rf_setup)
  {
  case 0:
    return (RF24_OutputPower_t::PWR_18dBm);
    break;
  case 1:
    return (RF24_OutputPower_t::PWR_12dBm);
    break;
  case 2:
    return (RF24_OutputPower_t::PWR_6dBm);
    break;
  default:
    return (RF24_OutputPower_t::PWR_0dBm);
    break;
  }
}

void RF24::setOutputPower(RF24_OutputPower_t outputPower)
{
  uint8_t rf_setup = readShort(RF24_Register::RF_SETUP);

  // Default value
  rf_setup or_eq RF_SETUP_RF_PWR_MASK;

  switch (outputPower)
  {
  case RF24_OutputPower_t::PWR_18dBm:
  {
    _clearBit(rf_setup, 1);
    _clearBit(rf_setup, 2);
  }
  break;
  case RF24_OutputPower_t::PWR_12dBm:
  {
    _clearBit(rf_setup, 2);
  }
  break;
  case RF24_OutputPower_t::PWR_6dBm:
  {
    _clearBit(rf_setup, 1);
  }
  break;
  case RF24_OutputPower_t::PWR_0dBm:
  {
    // Default value
  }
  break;
  }

  writeShort(RF24_Register::RF_SETUP, rf_setup);
}

uint8_t RF24::getRetryCount()
{
  uint8_t setup_retr = readShort(RF24_Register::SETUP_RETR);
  setup_retr and_eq SETUP_RETR_ARC_MASK;
  setup_retr >>= SETUP_RETR_ARC;

  return (setup_retr);
}

void RF24::setRetryCount(uint8_t count)
{
  _clip(count, 0b1111);

  uint8_t setup_retr = readShort(RF24_Register::SETUP_RETR);
  setup_retr and_eq compl(SETUP_RETR_ARC_MASK);
  setup_retr or_eq (count << SETUP_RETR_ARC);
  writeShort(RF24_Register::SETUP_RETR, setup_retr);
}

uint8_t RF24::getRetryDelay()
{
  uint8_t setup_retr = readShort(RF24_Register::SETUP_RETR);
  setup_retr and_eq SETUP_RETR_ARD_MASK;
  setup_retr >>= SETUP_RETR_ARD;

  return (setup_retr);
}

void RF24::setRetryDelay(uint8_t delay)
{
  _clip(delay, 0b1111);

  uint8_t setup_retr = readShort(RF24_Register::SETUP_RETR);
  setup_retr and_eq compl(SETUP_RETR_ARD_MASK);
  setup_retr or_eq (delay << SETUP_RETR_ARD);
  writeShort(RF24_Register::SETUP_RETR, setup_retr);
}

uint32_t RF24::readRxBaseAddress(uint8_t pipe)
{
  uint32_t baseAddress = 0;
  uint8_t baseAddressLength = addressLength - 1;
  uint8_t buffer[addressLength];

  if (pipe == 0)
  {
    R_REGISTER(RF24_Register::RX_ADDR_P0, buffer, addressLength);
  }
  else
  {
    R_REGISTER(RF24_Register::RX_ADDR_P1, buffer, addressLength);
  }

  memcpy(&baseAddress, &buffer[1], baseAddressLength);

  return baseAddress;
}

void RF24::writeRxBaseAddress(uint8_t pipe, uint32_t baseAddress)
{
  uint8_t baseAddressLength = addressLength - 1;
  uint8_t buffer[addressLength];

  if (pipe == 0)
  {
    R_REGISTER(RF24_Register::RX_ADDR_P0, buffer, addressLength);
  }
  else
  {
    R_REGISTER(RF24_Register::RX_ADDR_P1, buffer, addressLength);
  }

  memcpy(&buffer[1], &baseAddress, baseAddressLength);

  if (pipe == 0)
  {
    W_REGISTER(RF24_Register::RX_ADDR_P0, buffer, addressLength);
  }
  else
  {
    W_REGISTER(RF24_Register::RX_ADDR_P1, buffer, addressLength);
  }
}

uint32_t RF24::readTxBaseAddress()
{
  uint32_t baseAddress;

  uint8_t baseAddressLength = addressLength - 1;
  uint8_t addressBuffer[addressLength];

  R_REGISTER(RF24_Register::TX_ADDR, addressBuffer, addressLength);
  memcpy(&baseAddress, &addressBuffer[1], baseAddressLength);

  return baseAddress;
}

void RF24::writeTxBaseAddress(uint32_t baseAddress)
{
  uint8_t baseAddressLength = addressLength - 1;
  uint8_t buffer[addressLength];

  R_REGISTER(RF24_Register::TX_ADDR, buffer, addressLength);
  memcpy(&buffer[1], &baseAddress, baseAddressLength);
  W_REGISTER(RF24_Register::TX_ADDR, buffer, addressLength);
}

uint8_t RF24::readRxAddress(uint8_t pipe)
{
  switch (pipe)
  {
  case 0:
  {
    return readShort(RF24_Register::RX_ADDR_P0);
    break;
  }
  case 1:
  {
    return readShort(RF24_Register::RX_ADDR_P1);
    break;
  }
  case 2:
  {
    return readShort(RF24_Register::RX_ADDR_P2);
    break;
  }
  case 3:
  {
    return readShort(RF24_Register::RX_ADDR_P3);
    break;
  }
  case 4:
  {
    return readShort(RF24_Register::RX_ADDR_P4);
    break;
  }
  case 5:
  {
    return readShort(RF24_Register::RX_ADDR_P5);
    break;
  }
  default:
  {
    return 0;
    break;
  }
  }
}

void RF24::writeRxAddress(uint8_t pipe, uint8_t address)
{
  switch (pipe)
  {
  case 0:
  {
    writeShort(RF24_Register::RX_ADDR_P0, address);
    break;
  }
  case 1:
  {
    writeShort(RF24_Register::RX_ADDR_P1, address);
    break;
  }
  case 2:
  {
    writeShort(RF24_Register::RX_ADDR_P2, address);
    break;
  }
  case 3:
  {
    writeShort(RF24_Register::RX_ADDR_P3, address);
    break;
  }
  case 4:
  {
    writeShort(RF24_Register::RX_ADDR_P4, address);
    break;
  }
  case 5:
  {
    writeShort(RF24_Register::RX_ADDR_P5, address);
    break;
  }
  }
}

uint8_t RF24::readTxAddress()
{
  return readShort(RF24_Register::TX_ADDR);
}

void RF24::writeTxAddress(uint8_t address)
{
  writeShort(RF24_Register::TX_ADDR, address);
}

void RF24::enableAutoAcknowledgment(uint8_t pipe, bool enable)
{
  uint8_t en_aa = readShort(RF24_Register::EN_AA);

  if (enable)
  {
    _setBit(en_aa, pipe);
  }
  else
  {
    _clearBit(en_aa, pipe);
  }

  writeShort(RF24_Register::EN_AA, en_aa);
}

void RF24::enableDataPipe(uint8_t pipe, bool enable)
{
  uint8_t en_rxaddr = readShort(RF24_Register::EN_RXADDR);

  if (enable)
  {
    _setBit(en_rxaddr, pipe);
  }
  else
  {
    _clearBit(en_rxaddr, pipe);
  }

  writeShort(RF24_Register::EN_RXADDR, en_rxaddr);
}

void RF24::setRxCallback(RF24_RxCallback_t callback, void* context)
{
  rxCallback = callback;
  rxContext = context;
}

void RF24::setTxCallback(RF24_TxCallback_t callback, void* context)
{
  txCallback = callback;
  txContext = context;
}

uint8_t RF24::readShort(RF24_Register reg)
{
  uint8_t val;

  R_REGISTER(reg, &val);

  return val;
}

void RF24::writeShort(RF24_Register reg, uint8_t val)
{
  W_REGISTER(reg, &val);
}
