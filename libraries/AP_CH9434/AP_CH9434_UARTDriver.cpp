/*
 * AP_CH9434_UARTDriver.cpp - CH9434 SPI to 4xUART UARTDriver for ArduPilot
 */

#include "AP_CH9434_UARTDriver.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

extern const AP_HAL::HAL &hal;

#if HAL_CH9434_ENABLED

AP_CH9434_UARTDriver *AP_CH9434_UARTDriver::_singleton[CH9434_NUM_UARTS];
AP_CH9434_Manager *AP_CH9434_Manager::_singleton;

const AP_Param::GroupInfo AP_CH9434_Manager::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: CH9434 Enable
    // @Description: Enable CH9434 SPI-to-4UART expansion module
    // @Values: 0:Disable,1:Enable
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE",  1, AP_CH9434_Manager, _enabled, 1, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

AP_CH9434_UARTDriver::AP_CH9434_UARTDriver(uint8_t uart_idx) :
    _uart_idx(uart_idx),
    _chip(nullptr),
    _baudrate(0),
    _initialised(false),
    _rx_buffer_size(0),
    _tx_buffer_size(0),
    readbuffer(nullptr),
    writebuffer(nullptr)
{
    if (uart_idx < CH9434_NUM_UARTS) {
        _singleton[uart_idx] = this;
    }
}

bool AP_CH9434_UARTDriver::init(void)
{
    if (_initialised) {
        return true;
    }

    _chip = AP_CH9434::get_singleton();
    if (_chip == nullptr || !_chip->is_connected()) {
        return false;
    }

    _initialised = true;
    return true;
}

void AP_CH9434_UARTDriver::update(void)
{
    if (!_initialised) {
        return;
    }

    if (writebuffer != nullptr) {
        uint8_t buf[64];
        uint16_t len = writebuffer->available();
        if (len > 0) {
            len = MIN(len, sizeof(buf));
            if (writebuffer->read(buf, len) == len) {
                _chip->write_uart(_uart_idx, buf, len);
            }
        }
    }
}

void AP_CH9434_UARTDriver::_begin(uint32_t baud, uint16_t rxS, uint16_t txS)
{
    if (!_initialised && !init()) {
        return;
    }

    if (readbuffer == nullptr && rxS > 0) {
        readbuffer = NEW_NOTHROW ByteBuffer(rxS);
    }
    if (writebuffer == nullptr && txS > 0) {
        writebuffer = NEW_NOTHROW ByteBuffer(txS);
    }

    _rx_buffer_size = rxS;
    _tx_buffer_size = txS;

    if (baud != _baudrate) {
        _configure_baud(baud);
    }
}

bool AP_CH9434_UARTDriver::_configure_baud(uint32_t baud)
{
    if (_chip == nullptr || !_chip->is_connected()) {
        return false;
    }

    _baudrate = baud;
    _chip->set_baud(_uart_idx, baud);
    return true;
}

uint32_t AP_CH9434_UARTDriver::txspace()
{
    if (!_initialised) {
        return 0;
    }

    return writebuffer != nullptr ? writebuffer->space() : 0;
}

size_t AP_CH9434_UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialised || buffer == nullptr || size == 0) {
        return 0;
    }

    if (writebuffer == nullptr) {
        return 0;
    }

    size_t written = 0;
    while (written < size) {
        uint16_t space = writebuffer->space();
        if (space == 0) {
            break;
        }
        uint16_t to_write = MIN(space, size - written);
        if (writebuffer->write(buffer + written, to_write) != to_write) {
            break;
        }
        written += to_write;
    }

    return written;
}

ssize_t AP_CH9434_UARTDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!_initialised || buffer == nullptr || count == 0) {
        return 0;
    }

    if (readbuffer == nullptr) {
        return 0;
    }

    uint16_t available = readbuffer->available();
    if (available == 0) {
        return 0;
    }

    uint16_t to_read = MIN(available, count);
    uint16_t read = readbuffer->read(buffer, to_read);

    return read;
}

uint32_t AP_CH9434_UARTDriver::_available()
{
    if (!_initialised) {
        return 0;
    }

    if (readbuffer == nullptr) {
        return 0;
    }

    uint8_t buf[64];
    uint16_t read_len = 0;

    if (_chip != nullptr && _chip->is_connected()) {
        if (_chip->read_uart(_uart_idx, buf, sizeof(buf), &read_len)) {
            if (read_len > 0) {
                readbuffer->write(buf, read_len);
            }
        }
    }

    return readbuffer->available();
}

void AP_CH9434_UARTDriver::_end()
{
    _initialised = false;

    if (readbuffer != nullptr) {
        delete readbuffer;
        readbuffer = nullptr;
    }

    if (writebuffer != nullptr) {
        delete writebuffer;
        writebuffer = nullptr;
    }
}

void AP_CH9434_UARTDriver::_flush()
{
    if (!_initialised || writebuffer == nullptr) {
        return;
    }

    uint8_t buf[64];
    uint16_t len = writebuffer->available();
    while (len > 0) {
        uint16_t to_send = MIN(len, sizeof(buf));
        if (writebuffer->read(buf, to_send) != to_send) {
            break;
        }
        if (_chip != nullptr && _chip->is_connected()) {
            _chip->write_uart(_uart_idx, buf, to_send);
        }
        len -= to_send;
    }
}

bool AP_CH9434_UARTDriver::_discard_input()
{
    if (!_initialised || readbuffer == nullptr) {
        return true;
    }

    readbuffer->clear();
    return true;
}

bool AP_CH9434_UARTDriver::tx_pending()
{
    if (!_initialised || writebuffer == nullptr) {
        return false;
    }

    return writebuffer->available() > 0;
}

AP_CH9434_Manager::AP_CH9434_Manager(void)
{
    _singleton = this;
    _chip = nullptr;
    _initialised = false;
    for (uint8_t i = 0; i < CH9434_NUM_UARTS; i++) {
        _ports[i] = nullptr;
    }
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_CH9434_Manager::init(void)
{
    if (_initialised) {
        hal.console->printf("CH9434: already initialized\n");
        return true;
    }

    hal.console->printf("\n");
    hal.console->printf("=====================================\n");
    hal.console->printf("CH9434 SPI-to-4UART Module Init\n");
    hal.console->printf("=====================================\n");

    if (_enabled.get() == 0) {
        hal.console->printf("CH9434: DISABLED (CH9434_ENABLE=0)\n");
        hal.console->printf("=====================================\n");
        return false;
    }

    hal.console->printf("CH9434: ENABLED (CH9434_ENABLE=1)\n");

    _chip = AP_CH9434::get_singleton();
    if (_chip == nullptr) {
        hal.console->printf("CH9434: ERROR - chip singleton not found!\n");
        hal.console->printf("=====================================\n");
        return false;
    }

    hal.console->printf("CH9434: chip singleton found\n");

    if (!_chip->init()) {
        hal.console->printf("CH9434: ERROR - chip SPI init failed!\n");
        hal.console->printf("        Check wiring: CS, SCK, MOSI, MISO\n");
        hal.console->printf("=====================================\n");
        return false;
    }

    hal.console->printf("CH9434: SPI init successful\n");
    hal.console->printf("CH9434: registering %u UART ports...\n", CH9434_NUM_UARTS);

    uint8_t registered_count = 0;
    for (uint8_t i = 0; i < CH9434_NUM_UARTS; i++) {
        _ports[i] = new AP_CH9434_UARTDriver(i);
        if (_ports[i] != nullptr) {
            _ports[i]->state.idx = 8 + i;  // CH9434 ports map to SERIAL8-SERIAL11
            _ports[i]->state.protocol.set(-1);
            _ports[i]->state.baud.set(115);
            _ports[i]->state.options.set(0);
            AP::serialmanager().register_port(_ports[i]);
            hal.console->printf("CH9434: Port %u registered (SERIAL%u, Default BAUD: 115200)\n", 
                               i+1, _ports[i]->state.idx);
            registered_count++;
        } else {
            hal.console->printf("CH9434: ERROR - failed to create port %u\n", i+1);
        }
    }

    _initialised = true;
    hal.console->printf("CH9434: %u/%u ports registered successfully\n", registered_count, CH9434_NUM_UARTS);
    hal.console->printf("CH9434: Module initialization COMPLETE\n");
    hal.console->printf("=====================================\n");
    hal.console->printf("\n");
    return true;
}

void AP_CH9434_Manager::update(void)
{
    if (!_initialised) {
        return;
    }

    for (uint8_t i = 0; i < CH9434_NUM_UARTS; i++) {
        if (_ports[i] != nullptr) {
            _ports[i]->update();
        }
    }
}

namespace AP {
    AP_CH9434_Manager *ch9434()
    {
        return AP_CH9434_Manager::get_singleton();
    }
};

#endif // HAL_CH9434_ENABLED
