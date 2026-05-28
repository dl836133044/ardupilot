/*
 * AP_CH9434_UART.cpp - CH9434 UART Application Layer
 *
 * This module provides UART communication via CH9434 SPI-to-quad-UART chip
 * - UART0, UART1: Rangefinder (TF-Luna)
 * - UART2, UART3: Communication
 */

#include "AP_CH9434_UART.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

AP_CH9434_UART::AP_CH9434_UART(void) :
    _ch9434(nullptr),
    _last_update_ms(0)
{
}

bool AP_CH9434_UART::init(void)
{
    _ch9434 = AP_CH9434::get_singleton();
    if (!_ch9434) {
        return false;
    }

    if (!_ch9434->init()) {
        return false;
    }

    return true;
}

void AP_CH9434_UART::update(void)
{
    if (!_ch9434 || !_ch9434->is_connected()) {
        return;
    }

    uint32_t now = AP_HAL::millis();
    if (now - _last_update_ms < UPDATE_INTERVAL_MS) {
        return;
    }
    _last_update_ms = now;
}

bool AP_CH9434_UART::read_rangefinder(uint8_t uart_idx, float &distance)
{
    if (uart_idx >= 2) {
        return false;
    }

    if (!_ch9434 || !_ch9434->is_connected()) {
        return false;
    }

    uint8_t buf[32];
    uint16_t read_len = 0;

    if (!_ch9434->read_uart(uart_idx, buf, sizeof(buf), &read_len)) {
        return false;
    }

    if (read_len == 0) {
        return false;
    }

    distance = 0.0f;
    return true;
}

bool AP_CH9434_UART::write_comm(uint8_t uart_idx, const uint8_t *data, uint16_t len)
{
    if (uart_idx < 2 || uart_idx >= 4) {
        return false;
    }

    if (!_ch9434 || !_ch9434->is_connected()) {
        return false;
    }

    return _ch9434->write_uart(uart_idx, data, len);
}

bool AP_CH9434_UART::read_comm(uint8_t uart_idx, uint8_t *data, uint16_t max_len, uint16_t &read_len)
{
    if (uart_idx < 2 || uart_idx >= 4) {
        return false;
    }

    if (!_ch9434 || !_ch9434->is_connected()) {
        return false;
    }

    return _ch9434->read_uart(uart_idx, data, max_len, &read_len);
}

AP_CH9434_COMM::AP_CH9434_COMM(void) :
    _uart_idx(0),
    _baud(115200),
    _initialized(false)
{
}

bool AP_CH9434_COMM::init(uint8_t uart_idx, uint32_t baud)
{
    if (uart_idx < 2 || uart_idx >= 4) {
        return false;
    }

    _uart_idx = uart_idx;
    _baud = baud;
    _initialized = true;

    return true;
}

void AP_CH9434_COMM::update(void)
{
}

bool AP_CH9434_COMM::send_message(const uint8_t *data, uint16_t len)
{
    if (!_initialized) {
        return false;
    }

    AP_CH9434 *ch9434 = AP_CH9434::get_singleton();
    if (!ch9434 || !ch9434->is_connected()) {
        return false;
    }

    return ch9434->write_uart(_uart_idx, data, len);
}

bool AP_CH9434_COMM::receive_message(uint8_t *data, uint16_t max_len, uint16_t &recv_len)
{
    if (!_initialized) {
        return false;
    }

    AP_CH9434 *ch9434 = AP_CH9434::get_singleton();
    if (!ch9434 || !ch9434->is_connected()) {
        return false;
    }

    return ch9434->read_uart(_uart_idx, data, max_len, &recv_len);
}
