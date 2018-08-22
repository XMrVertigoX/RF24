#ifndef ISPI_HPP
#define ISPI_HPP

#include <stddef.h>
#include <stdint.h>

class ISpi {
public:
    virtual uint8_t transmit_receive(uint8_t *txBytes, uint8_t *rxBytes, size_t numBytes) = 0;
};

#endif /* ISPI_HPP */
