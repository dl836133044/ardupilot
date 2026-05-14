#include "AP_RC_Switch.h"
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <cstdarg>

const AP_Param::GroupInfo AP_RC_Switch::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: RC Switch Monitor Enable
    // @Description: Enable RC switch monitor module
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_RC_Switch, _enabled, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: CHANNEL
    // @DisplayName: RC Switch Channel
    // @Description: RC channel number to monitor (1-18)
    // @Range: 1 18
    // @User: Standard
    AP_GROUPINFO("CHANNEL", 2, AP_RC_Switch, _channel, 7),

    // @Param: LOW_THR
    // @DisplayName: Low Threshold
    // @Description: PWM threshold for LOW position
    // @Range: 1000 1500
    // @User: Standard
    AP_GROUPINFO("LOW_THR", 3, AP_RC_Switch, _low_threshold, 1200),

    // @Param: HIGH_THR
    // @DisplayName: High Threshold
    // @Description: PWM threshold for HIGH position
    // @Range: 1500 2000
    // @User: Standard
    AP_GROUPINFO("HIGH_THR", 4, AP_RC_Switch, _high_threshold, 1800),

    AP_GROUPEND
};

AP_RC_Switch::AP_RC_Switch() :
    _last_pos_value(255),
    _last_print_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_RC_Switch::init()
{
    // 参数已在构造函数中初始化
}

// 辅助函数：发送消息到MAVLink（地面站可见）
static void send_msg(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    gcs().send_textv(MAV_SEVERITY_INFO, format, args);
    va_end(args);
}

void AP_RC_Switch::update()
{
    // 如果模块未启用，直接返回
    if (!_enabled.get()) {
        return;
    }

    // 获取通道号（转换为0-based索引）
    uint8_t ch_index = _channel.get() - 1;
    
    // 检查通道号是否有效
    if (ch_index >= NUM_RC_CHANNELS) {
        return;
    }

    // 读取RC通道
    RC_Channel *ch = RC_Channels::rc_channel(ch_index);
    if (ch == nullptr) {
        return;
    }

    int16_t pwm_signed = ch->get_radio_in();
    uint16_t pwm = (pwm_signed < 0) ? 0 : (uint16_t)pwm_signed;

    // 判断开关位置
    const char* pos_name;
    uint8_t pos_value;
    if (pwm < _low_threshold.get()) {
        pos_name = "LOW";
        pos_value = 0;
    } else if (pwm > _high_threshold.get()) {
        pos_name = "HIGH";
        pos_value = 2;
    } else {
        pos_name = "MIDDLE";
        pos_value = 1;
    }

    uint32_t now = AP_HAL::millis();
    
    // 判断是否需要打印：状态变化 或 超过打印间隔
    bool need_print = false;
    
    // 状态变化时打印
    if (pos_value != _last_pos_value) {
        need_print = true;
        _last_pos_value = pos_value;
        _last_print_ms = now;
    }
    // 超过间隔时间打印（默认5秒）
    else if (now - _last_print_ms >= PRINT_INTERVAL_MS) {
        need_print = true;
        _last_print_ms = now;
    }

    if (need_print) {
        // 消息1: 人类可读文本
        send_msg("RC Ch%d: %s (PWM:%d)", _channel.get(), pos_name, pwm);

        // 消息2: 数值化数据
        gcs().send_named_float("RC_CH%d_POS", (float)pos_value);
        gcs().send_named_float("RC_CH%d_PWM", (float)pwm);

        // 消息3: 自定义格式
        send_msg("RC_CH%d:POS=%s,PWM=%d", _channel.get(), pos_name, pwm);
    }
}