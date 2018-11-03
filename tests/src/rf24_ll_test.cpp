#include <cstdint>
#include <iostream>
#include <stdexcept>

#include <libnrf24l01/nrf24_ll.hpp>

#include <catch.hpp>
#include <gpio.hpp>
#include <hexdump.hpp>
#include <spi.hpp>

Spi spi;
nRF24_LL nRF24(spi);

// SECTION("resizing bigger changes size and capacity") {}

TEST_CASE("Creation", "")
{
  REQUIRE(&spi != NULL);
  REQUIRE(&nRF24 != NULL);
}

TEST_CASE("writeShort", "")
{
  char val = 0xAA;
  spi.setRxBytes(vector<char>(2));

  nRF24.writeShort(nRF24_Register::CONFIG, val);

  std::cout << Hexdump(spi.getTxBytes().data(), spi.getTxBytes().size()) << std::endl;
}
