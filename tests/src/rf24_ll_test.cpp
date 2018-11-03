#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>

#include <catch.hpp>
#include <gpio.hpp>
#include <hexdump.hpp>
#include <spi.hpp>

#include <libnrf24l01/nrf24_ll.hpp>

using namespace std;

template <typename TYPE>
static constexpr typename underlying_type<TYPE>::type asUnderlyingType(TYPE value)
{
  return static_cast<typename underlying_type<TYPE>::type>(value);
}

TEST_CASE("Creation", "")
{
  queue<vector<uint8_t>> rxQueue;
  queue<vector<uint8_t>> txQueue;

  Spi spi(rxQueue, txQueue);
  nRF24_LL nRF24(spi);

  REQUIRE(&spi != NULL);
  REQUIRE(&nRF24 != NULL);
}

TEST_CASE("Commands", "")
{
  queue<vector<uint8_t>> rxQueue;
  queue<vector<uint8_t>> txQueue;

  Spi spi(rxQueue, txQueue);
  nRF24_LL nRF24(spi);

  REQUIRE(&spi != NULL);
  REQUIRE(&nRF24 != NULL);

  SECTION("R_REGISTER")
  {
    uint8_t expectedStatus = 0xaa;
    uint8_t actualStatus = 0;
    uint8_t expectedData[] = {0xaa, 0x55, 0xaa, 0x55};
    uint8_t actualData[sizeof(expectedData)] = {0};

    vector<uint8_t> rx;
    rx.insert(rx.end(), expectedStatus);
    rx.insert(rx.end(), expectedData, expectedData + sizeof(expectedData));
    rxQueue.push(rx);

    actualStatus = nRF24.R_REGISTER(nRF24_Register::CONFIG, actualData, sizeof(actualData));

    CHECK(txQueue.front()[0] == (asUnderlyingType(nRF24_Register::CONFIG) | asUnderlyingType(nRF24_Command::R_REGISTER)));

    txQueue.pop();

    CHECK(actualStatus == expectedStatus);
    CHECK_FALSE(memcmp(expectedData, actualData, sizeof(expectedData)));

    CHECK_FALSE(rxQueue.size());
    CHECK_FALSE(txQueue.size());
  }
}
