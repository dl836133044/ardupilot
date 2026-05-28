/*
 * AP_CH9434_UARTDriver.h - CH9434 SPI to 4xUART UARTDriver for ArduPilot
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_CH9434/AP_CH9434.h>
#include <AP_Param/AP_Param.h>

#ifndef HAL_CH9434_ENABLED
#define HAL_CH9434_ENABLED 1
#endif

#if HAL_CH9434_ENABLED

#define CH9434_NUM_UARTS 4

class AP_CH9434_UARTDriver : public AP_SerialManager::RegisteredPort {
public:
    AP_CH9434_UARTDriver(uint8_t uart_idx);

    CLASS_NO_COPY(AP_CH9434_UARTDriver);

    static AP_CH9434_UARTDriver *get_singleton(uint8_t uart_idx) {
        if (uart_idx >= CH9434_NUM_UARTS) {
            return nullptr;
        }
        return _singleton[uart_idx];
    }

    bool init(void);
    void update(void);

    bool is_initialized() override { return _initialised; }
    bool tx_pending() override;

    uint32_t txspace() override;
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override;
    uint32_t _available() override;
    void _end() override;
    void _flush() override;
    bool _discard_input() override;

    enum flow_control get_flow_control(void) override {
        return FLOW_CONTROL_DISABLE;
    }

    uint32_t bw_in_bytes_per_second() const override {
        return _baudrate > 0 ? _baudrate / 10 : 5760;
    }

    uint32_t get_baud_rate() const override {
        return _baudrate;
    }

private:
    uint8_t _uart_idx;
    AP_CH9434 *_chip;
    uint32_t _baudrate;
    bool _initialised;
    uint16_t _rx_buffer_size;
    uint16_t _tx_buffer_size;

    ByteBuffer *readbuffer;
    ByteBuffer *writebuffer;

    static AP_CH9434_UARTDriver *_singleton[CH9434_NUM_UARTS];

    bool _configure_baud(uint32_t baud);
};

class AP_CH9434_Manager {
public:
    AP_CH9434_Manager(void);

    static AP_CH9434_Manager *get_singleton(void) { return _singleton; }

    bool init(void);
    void update(void);

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_CH9434_Manager *_singleton;
    AP_CH9434 *_chip;
    bool _initialised;
    AP_CH9434_UARTDriver *_ports[CH9434_NUM_UARTS];

    AP_Int8 _enabled;
};

namespace AP {
    AP_CH9434_Manager *ch9434();
};

#endif // HAL_CH9434_ENABLED