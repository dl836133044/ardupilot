-- 无人船推进模式切换脚本（PID控制版）
-- 通过CH7通道切换无人船推进模式
-- 切换流程：低通道1 -> 中位 -> 低通道2 -> 确认 -> 无人船模式

-- 配置参数
local CH7_PIN = 7
local CH1_PIN = 1
local CH2_PIN = 2
local CH3_PIN = 3

-- PID参数
local PID_KP = 3.0
local PID_KI = 0.2
local PID_KD = 0.5

-- 姿态辅助参数
local ROLL_KP = 2.0
local PITCH_KP = 2.0

-- 电机最小转速比例（防止转向时电机停止）
local MIN_MOTOR_RATIO = 0.5

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
local PRINT_INTERVAL = 2000
local last_debug_time = 0
local DEBUG_INTERVAL = 10000

-- PID变量
local last_time = 0
local last_yaw_value = 0
local integral = 0

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
    local filename = "/APM/logs/boat_pid_log.txt"
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

-- PID控制
local function pid_control(ch1_value, ch2_value, ch3_value, now)
    local ch1_num = tonumber(ch1_value) or 1500
    local ch2_num = tonumber(ch2_value) or 1500
    local ch3_num = tonumber(ch3_value) or 1500
    
    local throttle = ((ch3_num - 1500) / 500) * 100
    local steering_ch1 = ((ch1_num - 1500) / 500) * 100
    local ch2_offset = ch2_num - 1500
    
    -- 提前计算转向因子
    local turn_factor = 0
    if ch2_offset < 0 then
        turn_factor = -ch2_offset / 500
    end
    local effective_steering = steering_ch1 * turn_factor
    
    local speed = 0
    if throttle > 5 then
        speed = throttle
    end
    
    local speed_pwm = 1000 + (speed / 100) * 1000
    speed_pwm = math.max(1000, math.min(2000, speed_pwm))
    speed_pwm = tonumber(speed_pwm) or 1000
    
    local now_num = tonumber(now) or 0
    local last_time_num = tonumber(last_time) or 0
    local dt = (now_num - last_time_num) / 1000
    
    local yaw = ahrs:get_yaw_rad() or 0
    local yaw_num = tonumber(yaw) or 0
    
    local yaw_rate = 0
    if dt > 0 and last_time_num > 0 then
        yaw_rate = (yaw_num - last_yaw_value) / dt
    end
    last_time = now
    last_yaw_value = yaw_num
    
    local roll = ahrs:get_roll_rad() or 0
    local pitch = ahrs:get_pitch_rad() or 0
    local roll_deg = tonumber(roll) * 57.3
    local pitch_deg = tonumber(pitch) * 57.3
    
    local target_yaw_change = effective_steering * 0.05
    local error = target_yaw_change - yaw_rate
    
    -- 方向回正时重置积分项
    if math.abs(effective_steering) < 5 then
        integral = 0
    else
        integral = math.max(-5, math.min(5, integral + error * dt))
    end
    local derivative = -yaw_rate
    
    local pid_output = PID_KP * error + PID_KI * integral + PID_KD * derivative
    pid_output = math.max(-100, math.min(100, pid_output))
    
    local left_pwm = speed_pwm
    local right_pwm = speed_pwm
    
    -- 转向时：根据油门大小动态调整转向范围，保证电机不停止
    -- 只有当有效转向大于死区阈值时才执行转向
    if turn_factor > 0 and speed_pwm > 1000 and math.abs(effective_steering) > 5 then
        local speed_factor = (speed_pwm - 1000) / 1000
        local max_turn_ratio = speed_factor * 0.4
        local turn_ratio = math.abs(effective_steering) / 100 * max_turn_ratio
        
        if effective_steering > 0 then
            right_pwm = speed_pwm * (1 - turn_ratio)
            left_pwm = speed_pwm
        elseif effective_steering < 0 then
            left_pwm = speed_pwm * (1 - turn_ratio)
            right_pwm = speed_pwm
        end
    end
    
    -- 最低速度保护：任何时候都不能低于当前速度的70%
    if speed_pwm > 1000 then
        local min_speed = 1000 + (speed_pwm - 1000) * 0.7
        left_pwm = math.max(min_speed, left_pwm)
        right_pwm = math.max(min_speed, right_pwm)
    end
    
    -- 油门为0时，禁用PID和姿态辅助，电机完全停止
    if speed_pwm <= 1000 then
        return 1000, 1000, throttle, steering_ch1, ch2_offset / 5, 0, roll_deg, pitch_deg, speed_pwm, effective_steering
    end
    
    -- PID辅助转向（只增加，不减少）
    if pid_output > 0 then
        left_pwm = math.min(2000, left_pwm + (pid_output / 100) * 100)
    elseif pid_output < 0 then
        right_pwm = math.min(2000, right_pwm - (pid_output / 100) * 100)
    end
    
    -- 姿态辅助（只微调，不影响主速度）
    local roll_correction = roll_deg * ROLL_KP
    left_pwm = math.max(1000, math.min(2000, left_pwm - roll_correction))
    right_pwm = math.max(1000, math.min(2000, right_pwm + roll_correction))
    
    local pitch_correction = pitch_deg * PITCH_KP
    left_pwm = math.max(1000, math.min(2000, left_pwm - pitch_correction))
    right_pwm = math.max(1000, math.min(2000, right_pwm - pitch_correction))
    
    return left_pwm, right_pwm, throttle, steering_ch1, ch2_offset / 5, pid_output, roll_deg, pitch_deg, speed_pwm, effective_steering
end

-- 状态机处理
local function handle_state_machine(ch7_value, ch1_value, ch2_value, ch3_value)
    local ch7_pos = get_channel_position(ch7_value)
    local now = millis() or 0
    
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

    if not boot_stable then
        if ch7_pos == "MIDDLE" then
            boot_stable = true
            gcs:send_text(0, "系统就绪 - CH7已检测到中位")
        else
            return
        end
    end

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
                for chan = 1, 4 do
                    SRV_Channels:set_output_pwm_chan_timeout(chan-1, 0, 0)
                end
                SRV_Channels:set_output_pwm_chan_timeout(4, 1000, 0)
                SRV_Channels:set_output_pwm_chan_timeout(5, 1000, 0)
                current_state = STATE_BOAT_MODE
                log_message("✓ 已切换到无人船PID模式 - PWM1-4已禁用，PWM5-6已解锁")
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
            SRV_Channels:set_output_pwm_chan_timeout(4, 0, 0)
            SRV_Channels:set_output_pwm_chan_timeout(5, 0, 0)
            log_message("✓ 已退出无人船PID模式 - PWM5-6已禁用")
        else
            local left_pwm, right_pwm, throttle, steering, forward, pid_output, roll_deg, pitch_deg, speed_pwm, effective_steering = pid_control(ch1_value, ch2_value, ch3_value, now)
            SRV_Channels:set_output_pwm_chan_timeout(4, math.floor(left_pwm), 0)
            SRV_Channels:set_output_pwm_chan_timeout(5, math.floor(right_pwm), 0)
            
            if now - last_print_time > PRINT_INTERVAL then
                gcs:send_text(0, string.format("CH1:%d(%.0f%%) CH2:%.0f%% TH:%d(%.0f%%) -> L:%d R:%d PID:%.1f | Roll:%.1f Pitch:%.1f | speed:%d turn:%d",
                    ch1_value, steering, forward, ch3_value, throttle, math.floor(left_pwm), math.floor(right_pwm), pid_output, roll_deg, pitch_deg, math.floor(speed_pwm), math.floor(effective_steering)))
                last_print_time = now
            end
        end
    end
end

function update()
    local ch7_value = rc:get_pwm(CH7_PIN)
    local ch1_value = rc:get_pwm(CH1_PIN) or 1500
    local ch2_value = rc:get_pwm(CH2_PIN) or 1500
    local ch3_value = rc:get_pwm(CH3_PIN) or 1500

    handle_state_machine(ch7_value, ch1_value, ch2_value, ch3_value)

    return update, 100
end

gcs:send_text(0, "=== 无人船PID推进模式脚本已启动 ===")
local scr_user2_val = param and tostring(param:get("SCR_USER2")) or "N/A"
gcs:send_text(0, string.format("日志写入: SCR_USER2=%s", scr_user2_val))

return update()