#ifndef nRF24_LL_HPP
#define nRF24_LL_HPP

#include <cstdint>

#include <libnrf24l01/ispi.hpp>
#include <libnrf24l01/types.hpp>

inline void _clearBit(uint8_t& byte, uint8_t bit)
{
  byte &= ~((1 << bit));
}

inline void _setBit(uint8_t& byte, uint8_t bit)
{
  byte |= (1 << bit);
}

inline bool _isBitSet(uint8_t byte, uint8_t bit)
{
  return (byte & (1 << bit));
}

class nRF24_LL
{
public:
  nRF24_LL(ISpi& spi);
  virtual ~nRF24_LL();

  /*
   * ##########################################################################
   * Register modification
   * ##########################################################################
   */

  nRF24_CRCConfig_t getCrcConfig();
  void setCrcConfig(nRF24_CRCConfig_t crcConfig);

  nRF24_DataRate_t getDataRate();
  void setDataRate(nRF24_DataRate_t dataRate);

  nRF24_OutputPower_t getOutputPower();
  void setOutputPower(nRF24_OutputPower_t level);

  /*
   * Set the address width in bytes
   *
   * @param addressWidth The address width (3-5); other values are ignored.
   */
  void setAddressWidth(uint8_t addressWidth);
  uint8_t getAddressWidth();

  /*
   * The nRF24L01+ has six RX pipes each with a configurable (three to) five
   * bytes address. Pipes two to five share the upper (two to) four address
   * bytes with pipe one. Therefore the addresses are organized as a base
   * address (bytes 1:4) and the address (byte 0).
   */
  uint8_t readRxAddress(uint8_t pipe);
  uint32_t readRxBaseAddress(uint8_t pipe);
  void writeRxAddress(uint8_t pipe, uint8_t address);
  void writeRxBaseAddress(uint8_t pipe, uint32_t baseAddress);
  uint8_t readTxAddress();
  uint32_t readTxBaseAddress();
  void writeTxAddress(uint8_t address);
  void writeTxBaseAddress(uint32_t baseAddress);

  /*
   * Set the channel
   *
   * @param channel Channel (0-127)
   */
  void setChannel(uint8_t channel);
  uint8_t getChannel();

  /*
   * Set the number of retries
   *
   * @param count Number of retries (0-15), higher values are ignored.
   */
  void setRetryCount(uint8_t count);
  uint8_t getRetryCount();

  /*
   * Set the delay between retries (250µs - 4000µs)
   *
   * @param delay Delay between retries (0-15), higher values are ignored.
   */
  void setRetryDelay(uint8_t delay);
  uint8_t getRetryDelay();

  int getPackageLossCounter();
  int getRetransmissionCounter();

  void enableDynamicPayloadLength(uint8_t pipe, bool enable = true);
  void enableDataPipe(uint8_t pipe, bool enable = true);
  void enableAutoAcknowledgment(uint8_t pipe, bool enable = true);

  /*
   * ##########################################################################
   * Commands
   * ##########################################################################
   */

  uint8_t R_REGISTER(nRF24_Register reg, uint8_t bytes[], uint8_t numBytes = 1);
  uint8_t W_REGISTER(nRF24_Register reg, const uint8_t bytes[], uint8_t numBytes = 1);
  uint8_t R_RX_PAYLOAD(uint8_t bytes[], uint8_t numBytes);
  uint8_t W_TX_PAYLOAD(const uint8_t bytes[], uint8_t numBytes);
  uint8_t FLUSH_TX();
  uint8_t FLUSH_RX();
  uint8_t REUSE_TX_PL();
  uint8_t R_RX_PL_WID(uint8_t& payloadLength);
  uint8_t W_ACK_PAYLOAD(uint8_t pipe, const uint8_t bytes[], uint8_t numBytes);
  uint8_t W_TX_PAYLOAD_NOACK(const uint8_t bytes[], uint8_t numBytes);
  uint8_t NOP();

protected:
  /*
   * ##########################################################################
   * Utility functions
   * ##########################################################################
   */

  /*
   * Read single byte register
   */
  uint8_t readShort(nRF24_Register reg);

  /*
   * Write single byte register
   */
  void writeShort(nRF24_Register reg, uint8_t val);

  /*
   * Read data from rx fifo.
   *
   * @return EXIT_FAILURE if fifo is empty or data is corrupted; EXIT_SUCCESS otherwise
   */
  int readRxFifo(nRF24_Datagram_t& data);

  /*
   * Write data to tx fifo.
   *
   * @return EXIT_FAILURE if fifo is full; EXIT_SUCCESS otherwise
   */
  int writeTxFifo(nRF24_Datagram_t& data);

  /*
   * Clear all interrupt flags
   */
  void clearInterrupts();

private:
  ISpi& _spi;

  /*
   * Perform the actual spi transmission.
   *
   * @param command The final command (including the register)
   * @param txBytes TX buffer; NULL if RX only
   * @param rxBytes RX buffer; NULL if TX only
   * @param numBytes Length of the buffer(s)
   *
   * @return First byte of the rx buffer aka status byte
   */
  uint8_t transmit(uint8_t command, const uint8_t txBytes[], uint8_t rxBytes[], uint8_t numBytes);
};

#endif /* nRF24_LL_HPP */
