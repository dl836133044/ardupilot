/*
 * AP_CH9434.h - CH9434 SPI to 4xUART driver for ArduPilot
 *
 * This library provides support for the CH9434 SPI-to-quad-UART chip
 * allowing expansion of serial ports via SPI bus.
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#define CH9434_UART_COUNT 4

class AP_CH9434 {
public:
    AP_CH9434(void);

    static AP_CH9434 *get_singleton(void) { return _singleton; }

    bool init(void);

    void update(void);

    bool read_uart(uint8_t uart_idx, uint8_t *buf, uint16_t len, uint16_t *read_len);
    bool write_uart(uint8_t uart_idx, const uint8_t *buf, uint16_t len);
    bool set_baud(uint8_t uart_idx, uint32_t baud);

    bool is_connected(void) const { return _connected; }

private:
    static AP_CH9434 *_singleton;

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    bool _connected;

    struct {
        uint16_t baud_rate;
        bool initialized;
    } _uart[CH9434_UART_COUNT];

    bool _init_spi(void);
    bool _init_uart(uint8_t uart_idx, uint32_t baud);

    uint8_t _spi_write_read(uint8_t data);
    bool _spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len);

    bool _write_reg(uint8_t uart_idx, uint8_t reg, uint8_t val);
    uint8_t _read_reg(uint8_t uart_idx, uint8_t reg);

    bool _set_baud(uint8_t uart_idx, uint32_t baud);

    static const uint8_t UART_REG_RBR = 0;
    static const uint8_t UART_REG_THR = 0;
    static const uint8_t UART_REG_IER = 1;
    static const uint8_t UART_REG_IIR = 2;
    static const uint8_t UART_REG_FCR = 2;
    static const uint8_t UART_REG_LCR = 3;
    static const uint8_t UART_REG_MCR = 4;
    static const uint8_t UART_REG_LSR = 5;
    static const uint8_t UART_REG_MSR = 6;
    static const uint8_t UART_REG_SCR = 7;
    static const uint8_t UART_REG_DLL = 0;
    static const uint8_t UART_REG_DLM = 1;

    static const uint8_t REG_OP_WRITE = 0x80;
    static const uint8_t REG_OP_READ = 0x00;

    static const uint8_t UART_OFFSET[CH9434_UART_COUNT];

    static const uint32_t DEFAULT_BAUD = 115200;
};
