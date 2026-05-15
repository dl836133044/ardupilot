#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_RC_Switch {
public:
    AP_RC_Switch();
    void init();
    void update();

    static const struct AP_Param::GroupInfo var_info[];

private:
    // 参数
    AP_Int8 _enabled;           // 是否启用模块
    AP_Int8 _channel;           // 监控的通道号（1-18）
    AP_Int16 _low_threshold;    // 低位阈值（PWM）
    AP_Int16 _high_threshold;   // 高位阈值（PWM）
    AP_Int8 _enable_rangefinder; // 是否启用雷达数据获取

    // 状态变量
    uint8_t _last_pos_value;     // 上一次的位置值
    uint32_t _last_print_ms;     // RC通道上一次打印时间
    uint32_t _last_rng_print_ms; // 雷达上一次打印时间
    float _last_rng1_distance;   // 上一次雷达1距离
    float _last_rng2_distance;   // 上一次雷达2距离
    
    static const uint32_t PRINT_INTERVAL_MS = 5000;  // 打印间隔（毫秒）
    
    // 获取雷达数据
    void read_rangefinder_data();
};