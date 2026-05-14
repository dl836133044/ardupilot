#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

class AP_RC_Switch {
public:
    AP_RC_Switch();
    void update();

private:
    // 状态变量
    uint8_t _last_pos_value;  // 上一次的位置值
    uint32_t _last_print_ms;  // 上一次打印时间
    const uint32_t PRINT_INTERVAL_MS = 5000;  // 打印间隔（毫秒）
};