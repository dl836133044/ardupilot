# AP_RC_Switch - 遥控器通道监控模块

用于监控遥控器通道状态，通过MAVLink发送消息到地面站。

## 功能特性

- 监控指定RC通道的PWM值
- 判断开关位置（LOW/MIDDLE/HIGH）
- 通过MAVLink发送状态消息
- 可配置参数（通道号、阈值等）
- 支持状态变化时立即打印
- 支持定时打印（避免刷屏）

## 参数说明

| 参数 | 说明 | 默认值 | 取值范围 |
|------|------|--------|----------|
| RC_SW_ENABLE | 启用模块 | 1 | 0=禁用, 1=启用 |
| RC_SW_CHANNEL | 监控通道号 | 7 | 1-18 |
| RC_SW_LOW_THR | 低位阈值(PWM) | 1200 | 1000-1500 |
| RC_SW_HIGH_THR | 高位阈值(PWM) | 1800 | 1500-2000 |

## 消息格式

### 1. 人类可读文本
```
RC Ch7: HIGH (PWM:1900)
```

### 2. 数值化数据 (NAMED_VALUE_FLOAT)
- `RC_CH7_POS`: 开关位置值 (0=LOW, 1=MIDDLE, 2=HIGH)
- `RC_CH7_PWM`: PWM原始值

### 3. 自定义格式
```
RC_CH7:POS=HIGH,PWM=1900
```

## 修改的文件

### 新建文件
- `libraries/AP_RC_Switch/AP_RC_Switch.h` - 头文件
- `libraries/AP_RC_Switch/AP_RC_Switch.cpp` - 实现文件
- `libraries/AP_RC_Switch/CMakeLists.txt` - CMake配置
- `libraries/AP_RC_Switch/README.md` - 本文档

### 修改的文件
| 文件 | 修改内容 |
|------|----------|
| `Tools/ardupilotwaf/ardupilotwaf.py` | 添加AP_RC_Switch到库列表 |
| `ArduCopter/Copter.h` | 添加AP_RC_Switch头文件和成员变量 |
| `ArduCopter/Copter.cpp` | 添加调度任务 |
| `ArduCopter/Parameters.h` | 添加rc_switch_ptr指针声明 |
| `ArduCopter/Parameters.cpp` | 注册参数组 |

## 编译说明

```bash
./waf copter
```

## 使用方法

1. 编译固件并烧录到飞控
2. 通过Mission Planner连接飞控
3. 在Full Parameter List中找到RC_SW_开头的参数
4. 根据需要修改参数值
5. 重启飞控使设置生效

## 通道阈值说明

| 位置 | PWM范围 |
|------|---------|
| LOW | PWM < RC_SW_LOW_THR |
| MIDDLE | RC_SW_LOW_THR <= PWM <= RC_SW_HIGH_THR |
| HIGH | PWM > RC_SW_HIGH_THR |

## 扩展自定义数据

在`AP_RC_Switch.cpp`的`update()`函数中可以添加自定义传感器数据：

```cpp
// 示例：添加温度传感器数据
float temperature = 25.5f;
gcs().send_named_float("TEMP_SENSOR", temperature);
send_msg("Temp: %.1fC", temperature);
```

## 作者

Created: 2026-05-14
