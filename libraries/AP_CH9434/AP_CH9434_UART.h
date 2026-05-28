/*
 * AP_CH9434_UART.h - CH9434 UART Application Layer
 *
 * This module provides UART communication via CH9434 SPI-to-quad-UART chip
 * - UART0, UART1: Rangefinder (TF-Luna)
 * - UART2, UART3: Communication
 */

#pragma once

#include "AP_CH9434.h"

class AP_CH9434_UART {
public:
    AP_CH9434_UART(void);

    bool init(void);
    void update(void);

    bool read_rangefinder(uint8_t uart_idx, float &distance);
    bool write_comm(uint8_t uart_idx, const uint8_t *data, uint16_t len);
    bool read_comm(uint8_t uart_idx, uint8_t *data, uint16_t len, uint16_t &read_len);

    bool is_connected(void) const { return _ch9434 && _ch9434->is_connected(); }

    enum class UARTPort : uint8_t {
        RANGEFINDER_1 = 0,
        RANGEFINDER_2 = 1,
        COMM_1 = 2,
        COMM_2 = 3
    };

private:
    AP_CH9434 *_ch9434;

    uint32_t _last_update_ms;

    static const uint32_t UPDATE_INTERVAL_MS = 10;
    static const uint32_t DEFAULT_BAUD = 115200;
};

class AP_CH9434_COMM {
public:
    AP_CH9434_COMM(void);

    bool init(uint8_t uart_idx, uint32_t baud = 115200);
    void update(void);

    bool send_message(const uint8_t *data, uint16_t len);
    bool receive_message(uint8_t *data, uint16_t max_len, uint16_t &recv_len);

    bool isInitialized(void) const { return _initialized; }

private:
    uint8_t _uart_idx;
    uint32_t _baud;
    bool _initialized;
};
