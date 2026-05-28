/*
 * AP_CH9434.cpp - CH9434 SPI to 4xUART driver for ArduPilot
 *
 * This library provides support for the CH9434 SPI-to-quad-UART chip
 * allowing expansion of serial ports via SPI bus.
 */

#include "AP_CH9434.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL &hal;

AP_CH9434 *AP_CH9434::_singleton = nullptr;

const uint8_t AP_CH9434::UART_OFFSET[CH9434_UART_COUNT] = {
    0x00,
    0x10,
    0x20,
    0x30
};

AP_CH9434::AP_CH9434(void) :
    _connected(false)
{
    for (uint8_t i = 0; i < CH9434_UART_COUNT; i++) {
        _uart[i].baud_rate = 0;
        _uart[i].initialized = false;
    }
    _singleton = this;
}

bool AP_CH9434::init(void)
{
    if (!_init_spi()) {
        hal.console->printf("CH9434: SPI init failed\n");
        return false;
    }

    for (uint8_t i = 0; i < CH9434_UART_COUNT; i++) {
        if (!_init_uart(i, DEFAULT_BAUD)) {
            hal.console->printf("CH9434: UART%u init failed\n", i);
            return false;
        }
    }

    _connected = true;
    hal.console->printf("CH9434: initialized successfully\n");
    return true;
}

bool AP_CH9434::_init_spi(void)
{
    _dev = std::move(hal.spi->get_device("ch9434"));
    if (!_dev) {
        hal.console->printf("CH9434: get device failed\n");
        return false;
    }

    uint8_t tx[2] = {0x00, 0x00};
    uint8_t rx[2] = {0x00, 0x00};
    _dev->transfer(tx, sizeof(tx), rx, sizeof(rx));

    return true;
}

bool AP_CH9434::_init_uart(uint8_t uart_idx, uint32_t baud)
{
    if (uart_idx >= CH9434_UART_COUNT) {
        return false;
    }

    _write_reg(uart_idx, UART_REG_LCR, 0x03);

    _write_reg(uart_idx, UART_REG_IER, 0x00);

    _write_reg(uart_idx, UART_REG_FCR, 0x07);

    _write_reg(uart_idx, UART_REG_MCR, 0x00);

    if (!_set_baud(uart_idx, baud)) {
        return false;
    }

    _write_reg(uart_idx, UART_REG_LCR, 0x03);

    _uart[uart_idx].baud_rate = baud;
    _uart[uart_idx].initialized = true;

    return true;
}

bool AP_CH9434::_set_baud(uint8_t uart_idx, uint32_t baud)
{
    uint32_t div = 48000000 / 16 / baud;

    uint8_t lcr = _read_reg(uart_idx, UART_REG_LCR);
    lcr |= (1 << 7);
    _write_reg(uart_idx, UART_REG_LCR, lcr);

    _write_reg(uart_idx, UART_REG_DLL, div & 0xFF);
    _write_reg(uart_idx, UART_REG_DLM, (div >> 8) & 0xFF);

    lcr &= ~(1 << 7);
    _write_reg(uart_idx, UART_REG_LCR, lcr);

    return true;
}

bool AP_CH9434::_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (!_dev) {
        return false;
    }
    _dev->transfer(tx, len, rx, len);
    return true;
}

bool AP_CH9434::_write_reg(uint8_t uart_idx, uint8_t reg, uint8_t val)
{
    uint8_t addr = UART_OFFSET[uart_idx] | reg | REG_OP_WRITE;
    uint8_t tx[2] = {addr, val};
    uint8_t rx[2] = {0, 0};

    _spi_transfer(tx, rx, 2);

    return rx[1] == val;
}

uint8_t AP_CH9434::_read_reg(uint8_t uart_idx, uint8_t reg)
{
    uint8_t addr = UART_OFFSET[uart_idx] | reg | REG_OP_READ;
    uint8_t tx[2] = {addr, 0xFF};
    uint8_t rx[2] = {0, 0};

    _spi_transfer(tx, rx, 2);

    return rx[1];
}

bool AP_CH9434::read_uart(uint8_t uart_idx, uint8_t *buf, uint16_t len, uint16_t *read_len)
{
    if (uart_idx >= CH9434_UART_COUNT || !buf || !read_len) {
        return false;
    }

    if (!_uart[uart_idx].initialized) {
        return false;
    }

    uint8_t lsr = _read_reg(uart_idx, UART_REG_LSR);
    if (!(lsr & 0x01)) {
        *read_len = 0;
        return true;
    }

    uint16_t count = 0;
    for (uint16_t i = 0; i < len; i++) {
        lsr = _read_reg(uart_idx, UART_REG_LSR);
        if (!(lsr & 0x01)) {
            break;
        }
        buf[i] = _read_reg(uart_idx, UART_REG_RBR);
        count++;
    }

    *read_len = count;
    return true;
}

bool AP_CH9434::write_uart(uint8_t uart_idx, const uint8_t *buf, uint16_t len)
{
    if (uart_idx >= CH9434_UART_COUNT || !buf) {
        return false;
    }

    if (!_uart[uart_idx].initialized) {
        return false;
    }

    for (uint16_t i = 0; i < len; i++) {
        _write_reg(uart_idx, UART_REG_THR, buf[i]);

        for (volatile uint16_t j = 0; j < 100; j++) {
            uint8_t lsr = _read_reg(uart_idx, UART_REG_LSR);
            if (lsr & 0x20) {
                break;
            }
        }
    }

    return true;
}

bool AP_CH9434::set_baud(uint8_t uart_idx, uint32_t baud)
{
    if (uart_idx >= CH9434_UART_COUNT) {
        return false;
    }

    if (!_uart[uart_idx].initialized) {
        return false;
    }

    return _set_baud(uart_idx, baud);
}
