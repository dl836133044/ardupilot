-- 无人船推进模式切换脚本
-- 通过CH7通道切换无人船推进模式
-- 切换流程：低通道1 -> 中位 -> 低通道2 -> 确认 -> 无人船模式

-- 配置参数
local CH7_PIN = 7  -- RC通道7（模式切换）
local CH2_PIN = 2  -- RC通道2（油门/前进）
local CH3_PIN = 3  -- RC通道3（方向/转向）

-- 通道位置阈值
local CH_LOW_THRESHOLD = 1300
local CH_HIGH_THRESHOLD = 1700
local CH_MIDDLE_MIN = 1400
local CH_MIDDLE_MAX = 1600

-- 状态定义
local STATE_NORMAL = 0
local STATE_LOW1 = 1
local STATE_MIDDLE = 2
local STATE_LOW2 = 3
local STATE_BOAT_MODE = 4

-- 状态变量
local current_state = STATE_NORMAL
local state_start_time = 0
local boot_stable = false
local last_print_time = 0
local PRINT_INTERVAL = 2000  -- 打印间隔(ms)
local last_debug_time = 0
local DEBUG_INTERVAL = 10000  -- 调试信息打印间隔(ms)

-- 从飞控参数 SCR_USER2 读取日志文件写入设置
local function get_log_to_file()
    if param then
        local user2 = param:get("SCR_USER2")
        return (user2 and user2 > 0)
    end
    return false
end

-- 写入日志到SD卡文件
local function write_log_file(message)
    if not get_log_to_file() then
        return
    end
    
    -- 使用固定文件名
    local filename = "/APM/logs/boat_log.txt"
    
    local file = io.open(filename, "a")
    if file then
        file:write(string.format("%s\n", message))
        file:close()
        gcs:send_text(0, string.format("LOG: %s", filename))
    end
end

-- 统一日志函数
local function log_message(message)
    gcs:send_text(0, message)
    write_log_file(message)
end

-- 获取通道位置
local function get_channel_position(ch_value)
    if not ch_value then return "UNKNOWN" end
    if ch_value < CH_LOW_THRESHOLD then return "LOW" end
    if ch_value > CH_HIGH_THRESHOLD then return "HIGH" end
    if ch_value >= CH_MIDDLE_MIN and ch_value <= CH_MIDDLE_MAX then return "MIDDLE" end
    return "UNKNOWN"
end

-- 检查是否在飞行状态
local function is_flying()
    if not arming:is_armed() then return false end
    
    local throttle = rc:get_pwm(3)
    if throttle and throttle > 1100 then return true end
    
    return false
end

-- 状态机处理
local function handle_state_machine(ch7_value, ch2_value, ch3_value)
    local ch7_pos = get_channel_position(ch7_value)
    local now = millis() or 0
    
    -- 打印调试信息（限制频率）
    if now - last_debug_time > DEBUG_INTERVAL then
        local state_name = "NORMAL"
        if current_state == STATE_LOW1 then state_name = "LOW1"
        elseif current_state == STATE_MIDDLE then state_name = "MIDDLE"
        elseif current_state == STATE_LOW2 then state_name = "LOW2"
        elseif current_state == STATE_BOAT_MODE then state_name = "BOAT_MODE" end
        
        gcs:send_text(0, string.format("DEBUG: State=%s, CH7=%d(%s), boot=%s, flying=%s",
            state_name, ch7_value or 0, ch7_pos, tostring(boot_stable), tostring(is_flying())))
        last_debug_time = now
    end

    -- 开机稳定检测
    if not boot_stable then
        if ch7_pos == "MIDDLE" then
            boot_stable = true
            gcs:send_text(0, "系统就绪 - CH7已检测到中位")
        else
            return
        end
    end

    -- 状态机逻辑
    if current_state == STATE_NORMAL then
        if ch7_pos == "LOW" then
            if not is_flying() then
                current_state = STATE_LOW1
                state_start_time = now
                log_message("检测到CH7低位 - 请回到中位确认")
            else
                gcs:send_text(1, "无法切换：飞机正在飞行")
            end
        end

    elseif current_state == STATE_LOW1 then
        if ch7_pos == "MIDDLE" then
            current_state = STATE_MIDDLE
            log_message("已回到中位 - 请再次拨到低位")
        elseif ch7_pos == "HIGH" then
            current_state = STATE_NORMAL
            log_message("已取消切换")
        end

    elseif current_state == STATE_MIDDLE then
        if ch7_pos == "LOW" then
            current_state = STATE_LOW2
            state_start_time = now
            log_message("检测到第二次低位 - 确认切换...")
        elseif ch7_pos == "HIGH" then
            current_state = STATE_NORMAL
            log_message("已取消切换")
        end

    elseif current_state == STATE_LOW2 then
        if ch7_pos == "LOW" and (now - state_start_time > 500) then
            -- 检查PWM1-4是否有输出
            local motors_active = false
            for chan = 1, 4 do
                local pwm_val = SRV_Channels:get_output_pwm(chan-1) or 0
                if pwm_val ~= 0 then
                    motors_active = true
                    break
                end
            end
            
            if motors_active then
                gcs:send_text(1, "无法切换：电机仍在运转，请先停止")
                current_state = STATE_NORMAL
            else
                -- 禁用PWM1-4（设置为0）
                for chan = 1, 4 do
                    SRV_Channels:set_output_pwm_chan_timeout(chan-1, 0, 0)
                end
                -- 解锁PWM5-6（设置为1000停止）
            SRV_Channels:set_output_pwm_chan_timeout(4, 1000, 0)  -- PWM5
            SRV_Channels:set_output_pwm_chan_timeout(5, 1000, 0)  -- PWM6
                current_state = STATE_BOAT_MODE
                log_message("✓ 已切换到无人船模式 - PWM1-4已禁用，PWM5-6已解锁")
                gcs:send_text(0, "PWM1-4已设置为0，PWM5-6已解锁")
            end
        elseif ch7_pos ~= "LOW" then
            current_state = STATE_NORMAL
            log_message("已取消切换")
        end

    elseif current_state == STATE_BOAT_MODE then
        if ch7_pos == "HIGH" then
            current_state = STATE_NORMAL
            for chan = 1, 4 do
                SRV_Channels:set_output_pwm_chan_timeout(chan-1, 1500, 0)
            end
            -- 禁用PWM5和PWM6
            SRV_Channels:set_output_pwm_chan_timeout(4, 0, 0)  -- PWM5
            SRV_Channels:set_output_pwm_chan_timeout(5, 0, 0)  -- PWM6
            log_message("✓ 已退出无人船模式 - PWM5-6已禁用")
        else
            -- CH2映射到PWM6：1500时输出1000停止，小于1500时正向输出
            -- CH2<1500 → PWM6=1000+4×(1500-CH2)，CH2≥1500 → PWM6=1000
            local pwm6_value = 1000
            if ch2_value < 1500 then
                pwm6_value = 1000 + 4 * (1500 - ch2_value)
            end
            pwm6_value = math.min(2000, pwm6_value)
            SRV_Channels:set_output_pwm_chan_timeout(5, pwm6_value, 0)  -- PWM6
            
            -- CH3映射到PWM5：1500时输出1000停止，大于1500时正向输出
            -- CH3>1500 → PWM5=1000+4×(CH3-1500)，CH3≤1500 → PWM5=1000
            local pwm5_value = 1000
            if ch3_value > 1500 then
                pwm5_value = 1000 + 4 * (ch3_value - 1500)
            end
            pwm5_value = math.min(2000, pwm5_value)
            SRV_Channels:set_output_pwm_chan_timeout(4, pwm5_value, 0)  -- PWM5
        end
        
        -- 无人船模式下打印通道数据和PWM输出
        if now - last_print_time > PRINT_INTERVAL then
            local ch2_percent = ((ch2_value - 1500) / 500) * 100
            local ch3_percent = ((ch3_value - 1500) / 500) * 100
            local pwm6_value = 1000
            if ch2_value < 1500 then
                pwm6_value = 1000 + 4 * (1500 - ch2_value)
            end
            pwm6_value = math.min(2000, pwm6_value)
            local pwm5_value = 1000
            if ch3_value > 1500 then
                pwm5_value = 1000 + 4 * (ch3_value - 1500)
            end
            pwm5_value = math.min(2000, pwm5_value)
            gcs:send_text(0, string.format("CH2:%d(%.0f%%)->PWM6:%d CH3:%d(%.0f%%)->PWM5:%d",
                ch2_value, ch2_percent, pwm6_value, ch3_value, ch3_percent, pwm5_value))
            last_print_time = now
        end
    end
end

-- 主更新函数
function update()
    local ch7_value = rc:get_pwm(CH7_PIN)
    local ch2_value = rc:get_pwm(CH2_PIN) or 1500
    local ch3_value = rc:get_pwm(CH3_PIN) or 1500

    handle_state_machine(ch7_value, ch2_value, ch3_value)

    return update, 100
end

-- 启动消息
gcs:send_text(0, "=== 无人船推进模式脚本已启动 ===")
local scr_user2_val = param and tostring(param:get("SCR_USER2")) or "N/A"
gcs:send_text(0, string.format("日志写入: SCR_USER2=%s", scr_user2_val))

return update()