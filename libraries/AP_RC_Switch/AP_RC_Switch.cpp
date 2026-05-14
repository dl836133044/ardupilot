#include "AP_RC_Switch.h"
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <cstdarg>

AP_RC_Switch::AP_RC_Switch() :
    _last_pos_value(255),  // 初始化为无效值
    _last_print_ms(0)
{}

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
    // ========== 读取RC通道7 ==========
    RC_Channel *ch = RC_Channels::rc_channel(6);
    if (ch == nullptr) {
        return;
    }

    int16_t pwm_signed = ch->get_radio_in();
    uint16_t pwm = (pwm_signed < 0) ? 0 : (uint16_t)pwm_signed;

    // 判断开关位置
    const char* pos_name;
    uint8_t pos_value;
    if (pwm < 1200) {
        pos_name = "LOW";
        pos_value = 0;
    } else if (pwm > 1800) {
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
    // 超过间隔时间打印（默认1秒）
    else if (now - _last_print_ms >= PRINT_INTERVAL_MS) {
        need_print = true;
        _last_print_ms = now;
    }

    if (need_print) {
        // ========== 消息1: 人类可读文本 ==========
        send_msg("RC Ch7: %s (PWM:%d)", pos_name, pwm);

        // ========== 消息2: 数值化数据 ==========
        gcs().send_named_float("RC_CH7_POS", (float)pos_value);
        gcs().send_named_float("RC_CH7_PWM", (float)pwm);

        // ========== 消息3: 自定义格式 ==========
        send_msg("RC_CH7:POS=%s,PWM=%d", pos_name, pwm);

        // ========== 示例：未来添加自定义传感器数据 ==========
        /*
        float temperature = 25.5f;
        gcs().send_named_float("TEMP_SENSOR", temperature);
        send_msg("Temp: %.1fC", temperature);
        */
    }
}