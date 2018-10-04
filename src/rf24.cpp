#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <libnrf24l01/igpio.hpp>
#include <libnrf24l01/ispi.hpp>
#include <libnrf24l01/rf24.hpp>

using namespace std;

#define BOUNCE(expression, statement) \
  if (expression)                     \
  return (statement)

static const uint8_t addressPrefixLength = 1;
static const uint8_t baseAddressOffset = 1;
static const uint8_t numPipes = 5;

static inline void clearBit(uint8_t& byte, uint8_t bit)
{
  byte and_eq compl((1 << bit));
}

static inline void setBit(uint8_t& byte, uint8_t bit)
{
  byte or_eq (1 << bit);
}

static inline bool isBitSet(uint8_t byte, uint8_t bit)
{
  return (byte bitand (1 << bit));
}

static inline uint8_t extractPipe(uint8_t status)
{
  status and_eq STATUS_RX_P_NO_MASK;
  status >> STATUS_RX_P_NO;

  return (status);
}

static inline void __attribute__((weak)) delayUs(int us) {}

RF24::RF24(ISpi& spi, IGpio& ce) : RF24_LL(spi), ce(ce) {}

RF24::~RF24() {}

void RF24::setup()
{
  // Enable dynamic payload length only
  uint8_t feature;
  R_REGISTER(RF24_Register::FEATURE, &feature);
  clearBit(feature, FEATURE_EN_DYN_ACK);
  clearBit(feature, FEATURE_EN_ACK_PAY);
  setBit(feature, FEATURE_EN_DPL);
  W_REGISTER(RF24_Register::FEATURE, &feature);

  // Clear interrupts
  uint8_t status;
  R_REGISTER(RF24_Register::STATUS, &status);
  setBit(status, STATUS_MAX_RT);
  setBit(status, STATUS_RX_DR);
  setBit(status, STATUS_TX_DS);
  W_REGISTER(RF24_Register::STATUS, &status);

  FLUSH_RX();
  FLUSH_TX();
}

void RF24::loop()
{
  uint8_t status = NOP(); // equal to R_REGISTER(RF24_Register::STATUS, &status);

  if (isBitSet(status, STATUS_RX_DR))
    handleDataReady(status);

  if (isBitSet(status, STATUS_TX_DS) || isBitSet(status, STATUS_MAX_RT))
    handleDataSent(status);

  W_REGISTER(RF24_Register::STATUS, &status);

  if (!rxBuffer.empty())
  {
    if (rxCallback)
    {
      rxCallback(rxBuffer.front(), rxContext);
    }

    rxBuffer.pop_front();
  }

  while (!txBuffer.empty())
  {
    if (writeTxFifo(txBuffer.front()))
    {
      break;
    }
    else
    {
      txBuffer.pop_front();
    }
  }
}

void RF24::handleDataSent(uint8_t status)
{
  if (isBitSet(status, STATUS_MAX_RT))
  {
    FLUSH_TX();
  }

  if (txCallback)
  {
    txCallback(txContext);
  }
}

void RF24::handleDataReady(uint8_t status)
{
  RF24_Datagram_t data;
  data.pipe = extractPipe(status);
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

int RF24::readRxFifo(RF24_Datagram_t& data)
{
  R_RX_PL_WID(data.numBytes);
  BOUNCE(data.numBytes > rxFifoSize, EXIT_FAILURE);
  R_RX_PAYLOAD(data.bytes, data.numBytes);

  return EXIT_SUCCESS;
}

int RF24::writeTxFifo(RF24_Datagram_t& data)
{
  BOUNCE(isBitSet(NOP(), STATUS_TX_FULL), EXIT_FAILURE);
  W_TX_PAYLOAD(data.bytes, data.numBytes);

  return EXIT_SUCCESS;
}

void RF24::enterRxMode()
{
  uint8_t config;

  R_REGISTER(RF24_Register::CONFIG, &config);
  setBit(config, CONFIG_PWR_UP);
  setBit(config, CONFIG_PRIM_RX);
  W_REGISTER(RF24_Register::CONFIG, &config);

  ce.set();

  delayUs(rxSettling);
}

void RF24::enterShutdownMode()
{
  uint8_t config;

  ce.clear();

  R_REGISTER(RF24_Register::CONFIG, &config);
  clearBit(config, CONFIG_PWR_UP);
  W_REGISTER(RF24_Register::CONFIG, &config);
}

void RF24::enterStandbyMode()
{
  uint8_t config;

  ce.clear();

  R_REGISTER(RF24_Register::CONFIG, &config);
  setBit(config, CONFIG_PWR_UP);
  W_REGISTER(RF24_Register::CONFIG, &config);
}

void RF24::enterTxMode()
{
  uint8_t config;

  R_REGISTER(RF24_Register::CONFIG, &config);
  setBit(config, CONFIG_PWR_UP);
  clearBit(config, CONFIG_PRIM_RX);
  W_REGISTER(RF24_Register::CONFIG, &config);

  ce.set();

  delayUs(txSettling);
}

RF24_Status RF24::startListening(uint8_t pipe)
{
  BOUNCE(pipe > numPipes, RF24_Status::UnknownPipe);

  enableDynamicPayloadLength(pipe);
  enableAutoAcknowledgment(pipe);
  enableDataPipe(pipe);

  return (RF24_Status::Success);
}

RF24_Status RF24::stopListening(uint8_t pipe)
{
  BOUNCE(pipe > numPipes, RF24_Status::UnknownPipe);

  enableDynamicPayloadLength(pipe, false);
  enableAutoAcknowledgment(pipe, false);
  enableDataPipe(pipe, false);

  return (RF24_Status::Success);
}

bool RF24::enqueueData(RF24_Datagram_t& data)
{
  return txBuffer.push_back(data);
}

int RF24::getPackageLossCounter()
{
  uint8_t observe_tx;

  R_REGISTER(RF24_Register::OBSERVE_TX, &observe_tx);
  observe_tx and_eq OBSERVE_TX_PLOS_CNT_MASK;
  observe_tx >>= OBSERVE_TX_PLOS_CNT;

  return observe_tx;
}

int RF24::getRetransmissionCounter()
{
  uint8_t observe_tx;

  R_REGISTER(RF24_Register::OBSERVE_TX, &observe_tx);
  observe_tx and_eq OBSERVE_TX_ARC_CNT_MASK;
  observe_tx >>= OBSERVE_TX_ARC_CNT;

  return observe_tx;
}

RF24_Status RF24::enableDynamicPayloadLength(uint8_t pipe, bool enable)
{
  uint8_t dynpd;

  BOUNCE(pipe > numPipes, RF24_Status::UnknownPipe);

  R_REGISTER(RF24_Register::DYNPD, &dynpd);

  if (enable)
  {
    setBit(dynpd, pipe);
  }
  else
  {
    clearBit(dynpd, pipe);
  }

  W_REGISTER(RF24_Register::DYNPD, &dynpd);

  // TODO: Verify

  return (RF24_Status::Success);
}

RF24_CRCConfig_t RF24::getCrcConfig()
{
  uint8_t config;

  R_REGISTER(RF24_Register::CONFIG, &config);

  if (isBitSet(config, CONFIG_EN_CRC) == false)
  {
    return (RF24_CRCConfig_t::CRC_DISABLED);
  }

  if (isBitSet(config, CONFIG_CRCO))
  {
    return (RF24_CRCConfig_t::CRC_2Bytes);
  }
  else
  {
    return (RF24_CRCConfig_t::CRC_1Byte);
  }
}

RF24_Status RF24::setCrcConfig(RF24_CRCConfig_t crcConfig)
{
  uint8_t config;

  R_REGISTER(RF24_Register::CONFIG, &config);

  switch (crcConfig)
  {
  case RF24_CRCConfig_t::CRC_DISABLED:
  {
    clearBit(config, CONFIG_EN_CRC);
  }
  break;
  case RF24_CRCConfig_t::CRC_1Byte:
  {
    setBit(config, CONFIG_EN_CRC);
    clearBit(config, CONFIG_CRCO);
  }
  break;
  case RF24_CRCConfig_t::CRC_2Bytes:
  {
    setBit(config, CONFIG_EN_CRC);
    setBit(config, CONFIG_CRCO);
  }
  break;
  }

  W_REGISTER(RF24_Register::CONFIG, &config);

  BOUNCE(crcConfig != getCrcConfig(), RF24_Status::VerificationFailed);

  return (RF24_Status::Success);
}

uint8_t RF24::getChannel()
{
  uint8_t channel;

  R_REGISTER(RF24_Register::RF_CH, &channel);

  BOUNCE(channel > 127, UINT8_MAX);

  return (channel);
}

RF24_Status RF24::setChannel(uint8_t channel)
{
  BOUNCE(channel > 127, RF24_Status::UnknownChannel);

  W_REGISTER(RF24_Register::RF_CH, &channel);

  BOUNCE(channel != getChannel(), RF24_Status::VerificationFailed);

  return (RF24_Status::Success);
}

RF24_DataRate_t RF24::getDataRate()
{
  uint8_t rf_setup;

  R_REGISTER(RF24_Register::RF_SETUP, &rf_setup);

  if (isBitSet(rf_setup, RF_SETUP_RF_DR_LOW))
  {
    return (RF24_DataRate_t::DR_250KBPS);
  }

  if (isBitSet(rf_setup, RF_SETUP_RF_DR_HIGH))
  {
    return (RF24_DataRate_t::DR_2MBPS);
  }

  return (RF24_DataRate_t::DR_1MBPS);
}

RF24_Status RF24::setDataRate(RF24_DataRate_t dataRate)
{
  uint8_t rf_setup;

  R_REGISTER(RF24_Register::RF_SETUP, &rf_setup);

  switch (dataRate)
  {
  case RF24_DataRate_t::DR_1MBPS:
  {
    clearBit(rf_setup, RF_SETUP_RF_DR_LOW);
    clearBit(rf_setup, RF_SETUP_RF_DR_HIGH);
  }
  break;
  case RF24_DataRate_t::DR_2MBPS:
  {
    clearBit(rf_setup, RF_SETUP_RF_DR_LOW);
    setBit(rf_setup, RF_SETUP_RF_DR_HIGH);
  }
  break;
  case RF24_DataRate_t::DR_250KBPS:
  {
    setBit(rf_setup, RF_SETUP_RF_DR_LOW);
    clearBit(rf_setup, RF_SETUP_RF_DR_HIGH);
  }
  break;
  }

  W_REGISTER(RF24_Register::RF_SETUP, &rf_setup);

  BOUNCE(dataRate != getDataRate(), RF24_Status::VerificationFailed);

  return (RF24_Status::Success);
}

RF24_OutputPower_t RF24::getOutputPower()
{
  uint8_t rf_setup;

  R_REGISTER(RF24_Register::RF_SETUP, &rf_setup);

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

RF24_Status RF24::setOutputPower(RF24_OutputPower_t outputPower)
{
  uint8_t rf_setup;

  R_REGISTER(RF24_Register::RF_SETUP, &rf_setup);

  // Default value
  rf_setup or_eq RF_SETUP_RF_PWR_MASK;

  switch (outputPower)
  {
  case RF24_OutputPower_t::PWR_18dBm:
  {
    clearBit(rf_setup, 1);
    clearBit(rf_setup, 2);
  }
  break;
  case RF24_OutputPower_t::PWR_12dBm:
  {
    clearBit(rf_setup, 2);
  }
  break;
  case RF24_OutputPower_t::PWR_6dBm:
  {
    clearBit(rf_setup, 1);
  }
  break;
  case RF24_OutputPower_t::PWR_0dBm:
  {
    // Default value
  }
  break;
  }

  W_REGISTER(RF24_Register::RF_SETUP, &rf_setup);

  BOUNCE(outputPower != getOutputPower(), RF24_Status::VerificationFailed);

  return (RF24_Status::Success);
}

uint8_t RF24::getRetryCount()
{
  uint8_t setup_retr;

  R_REGISTER(RF24_Register::SETUP_RETR, &setup_retr);
  setup_retr and_eq SETUP_RETR_ARC_MASK;
  setup_retr >>= SETUP_RETR_ARC;

  return (setup_retr);
}

RF24_Status RF24::setRetryCount(uint8_t count)
{
  uint8_t setup_retr;

  BOUNCE(count > 0xF, RF24_Status::Failure);

  R_REGISTER(RF24_Register::SETUP_RETR, &setup_retr);
  setup_retr or_eq (count << SETUP_RETR_ARC);
  W_REGISTER(RF24_Register::SETUP_RETR, &setup_retr);

  BOUNCE(count != getRetryCount(), RF24_Status::VerificationFailed);

  return (RF24_Status::Success);
}

uint8_t RF24::getRetryDelay()
{
  uint8_t setup_retr;

  R_REGISTER(RF24_Register::SETUP_RETR, &setup_retr);
  setup_retr and_eq SETUP_RETR_ARD_MASK;
  setup_retr >>= SETUP_RETR_ARD;

  return (setup_retr);
}

RF24_Status RF24::setRetryDelay(uint8_t delay)
{
  uint8_t setup_retr;

  BOUNCE(delay > 0xF, RF24_Status::Failure);

  R_REGISTER(RF24_Register::SETUP_RETR, &setup_retr);
  setup_retr or_eq (delay << SETUP_RETR_ARD);
  W_REGISTER(RF24_Register::SETUP_RETR, &setup_retr);

  BOUNCE(delay != getRetryDelay(), RF24_Status::VerificationFailed);

  return (RF24_Status::Success);
}

RF24_Status RF24::readRxBaseAddress(uint8_t pipe, uint32_t& baseAddress)
{
  uint8_t baseAddressLength = addressLength - addressPrefixLength;
  uint8_t buffer[addressLength];

  BOUNCE(pipe > numPipes, RF24_Status::UnknownPipe);

  if (pipe > 0)
  {
    R_REGISTER(RF24_Register::RX_ADDR_P1, buffer, addressLength);
  }
  else
  {
    R_REGISTER(RF24_Register::RX_ADDR_P0, buffer, addressLength);
  }

  memcpy(&baseAddress, &buffer[baseAddressOffset], baseAddressLength);

  return (RF24_Status::Success);
}

RF24_Status RF24::writeRxBaseAddress(uint8_t pipe, uint32_t baseAddress)
{
  uint8_t baseAddressLength = addressLength - addressPrefixLength;
  uint8_t buffer[addressLength];

  BOUNCE(pipe > numPipes, RF24_Status::UnknownPipe);

  if (pipe > 0)
  {
    R_REGISTER(RF24_Register::RX_ADDR_P1, buffer, addressLength);
  }
  else
  {
    R_REGISTER(RF24_Register::RX_ADDR_P0, buffer, addressLength);
  }

  memcpy(&buffer[baseAddressOffset], &baseAddress, baseAddressLength);

  if (pipe > 0)
  {
    W_REGISTER(RF24_Register::RX_ADDR_P1, buffer, addressLength);
  }
  else
  {
    W_REGISTER(RF24_Register::RX_ADDR_P0, buffer, addressLength);
  }

#ifndef NDEBUG
  uint32_t newBaseAddress;
  RF24_Status status = readRxBaseAddress(pipe, newBaseAddress);
  BOUNCE(status != RF24_Status::Success, RF24_Status::VerificationFailed);
  BOUNCE(newBaseAddress != baseAddress, RF24_Status::VerificationFailed);
#endif

  return (RF24_Status::Success);
}

RF24_Status RF24::readTxBaseAddress(uint32_t& baseAddress)
{
  uint8_t baseAddressLength = addressLength - addressPrefixLength;
  uint8_t addressBuffer[addressLength];

  R_REGISTER(RF24_Register::TX_ADDR, addressBuffer, addressLength);
  memcpy(&baseAddress, &addressBuffer[baseAddressOffset], baseAddressLength);

  return (RF24_Status::Success);
}

RF24_Status RF24::writeTxBaseAddress(uint32_t baseAddress)
{
  uint8_t baseAddressLength = addressLength - addressPrefixLength;
  uint8_t buffer[addressLength];

  R_REGISTER(RF24_Register::TX_ADDR, buffer, addressLength);
  memcpy(&buffer[baseAddressOffset], &baseAddress, baseAddressLength);
  W_REGISTER(RF24_Register::TX_ADDR, buffer, addressLength);

#ifndef NDEBUG
  uint32_t newBaseAddress;
  RF24_Status status = readTxBaseAddress(newBaseAddress);
  BOUNCE(status != RF24_Status::Success, RF24_Status::VerificationFailed);
  BOUNCE(newBaseAddress != baseAddress, RF24_Status::VerificationFailed);
#endif

  return (RF24_Status::Success);
}

RF24_Status RF24::readRxAddress(uint8_t pipe, uint8_t& address)
{
  BOUNCE(pipe > numPipes, RF24_Status::UnknownPipe);

  switch (pipe)
  {
  case 0:
    R_REGISTER(RF24_Register::RX_ADDR_P0, &address, addressPrefixLength);
    break;
  case 1:
    R_REGISTER(RF24_Register::RX_ADDR_P1, &address, addressPrefixLength);
    break;
  case 2:
    R_REGISTER(RF24_Register::RX_ADDR_P2, &address, addressPrefixLength);
    break;
  case 3:
    R_REGISTER(RF24_Register::RX_ADDR_P3, &address, addressPrefixLength);
    break;
  case 4:
    R_REGISTER(RF24_Register::RX_ADDR_P4, &address, addressPrefixLength);
    break;
  case 5:
    R_REGISTER(RF24_Register::RX_ADDR_P5, &address, addressPrefixLength);
    break;
  }

  return (RF24_Status::Success);
}

RF24_Status RF24::writeRxAddress(uint8_t pipe, uint8_t address)
{
  BOUNCE(pipe > numPipes, RF24_Status::UnknownPipe);

  switch (pipe)
  {
  case 0:
    W_REGISTER(RF24_Register::RX_ADDR_P0, &address, addressPrefixLength);
    break;
  case 1:
    W_REGISTER(RF24_Register::RX_ADDR_P1, &address, addressPrefixLength);
    break;
  case 2:
    W_REGISTER(RF24_Register::RX_ADDR_P2, &address, addressPrefixLength);
    break;
  case 3:
    W_REGISTER(RF24_Register::RX_ADDR_P3, &address, addressPrefixLength);
    break;
  case 4:
    W_REGISTER(RF24_Register::RX_ADDR_P4, &address, addressPrefixLength);
    break;
  case 5:
    W_REGISTER(RF24_Register::RX_ADDR_P5, &address, addressPrefixLength);
    break;
  }

#ifndef NDEBUG
  uint8_t newAddress;
  RF24_Status status = readRxAddress(pipe, newAddress);
  BOUNCE(status != RF24_Status::Success, RF24_Status::VerificationFailed);
  BOUNCE(newAddress != address, RF24_Status::VerificationFailed);
#endif

  return (RF24_Status::Success);
}

RF24_Status RF24::readTxAddress(uint8_t& address)
{
  R_REGISTER(RF24_Register::TX_ADDR, &address, addressPrefixLength);

  return (RF24_Status::Success);
}

RF24_Status RF24::writeTxAddress(uint8_t address)
{
  W_REGISTER(RF24_Register::TX_ADDR, &address, addressPrefixLength);

#ifndef NDEBUG
  uint8_t newAddress;
  RF24_Status status = readTxAddress(newAddress);
  BOUNCE(status != RF24_Status::Success, RF24_Status::VerificationFailed);
  BOUNCE(newAddress != address, RF24_Status::VerificationFailed);
#endif

  return (RF24_Status::Success);
}

RF24_Status RF24::enableAutoAcknowledgment(uint8_t pipe, bool enable)
{
  uint8_t en_aa;

  BOUNCE(pipe > numPipes, RF24_Status::UnknownPipe);

  R_REGISTER(RF24_Register::EN_AA, &en_aa);

  if (enable)
  {
    setBit(en_aa, pipe);
  }
  else
  {
    clearBit(en_aa, pipe);
  }

  W_REGISTER(RF24_Register::EN_AA, &en_aa);

  R_REGISTER(RF24_Register::EN_AA, &en_aa);
  BOUNCE(isBitSet(en_aa, pipe), RF24_Status::VerificationFailed);

  return (RF24_Status::Success);
}

RF24_Status RF24::enableDataPipe(uint8_t pipe, bool enable)
{
  uint8_t en_rxaddr;

  BOUNCE(pipe > numPipes, RF24_Status::UnknownPipe);

  R_REGISTER(RF24_Register::EN_RXADDR, &en_rxaddr);

  if (enable)
  {
    setBit(en_rxaddr, pipe);
  }
  else
  {
    clearBit(en_rxaddr, pipe);
  }

  W_REGISTER(RF24_Register::EN_RXADDR, &en_rxaddr);

  R_REGISTER(RF24_Register::EN_AA, &en_rxaddr);
  BOUNCE(isBitSet(en_rxaddr, pipe), RF24_Status::VerificationFailed);

  return (RF24_Status::Success);
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
