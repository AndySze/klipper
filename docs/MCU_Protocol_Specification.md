# Klipper MCU 通信协议规范

本文档详细描述了 Klipper 主机与 MCU 之间的通信协议，使开发者无需阅读 Klipper 源代码即可实现兼容的 MCU 固件。

## 目录

1. [概述](#概述)
2. [物理层](#物理层)
3. [帧格式](#帧格式)
4. [消息编码](#消息编码)
5. [握手流程](#握手流程)
6. [时钟同步](#时钟同步)
7. [命令参考](#命令参考)
8. [响应消息](#响应消息)
9. [错误处理](#错误处理)
10. [实现示例](#实现示例)

---

## 概述

### 架构

```
┌─────────────────┐                    ┌─────────────────┐
│   Klipper Host  │ ◄──── Serial ────► │   MCU Firmware  │
│   (Python/Go)   │      (250000 baud) │   (C/Rust/etc)  │
└─────────────────┘                    └─────────────────┘
```

### 关键特性

- **异步消息**: 主机发送命令，MCU 异步响应
- **时钟同步**: 所有定时操作基于 MCU 时钟
- **二进制协议**: 紧凑的 VLQ 编码减少带宽
- **CRC 校验**: 每帧包含 CRC16 校验

---

## 物理层

### 串口配置

| 参数 | 值 |
|------|-----|
| 波特率 | 250000 (默认) |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |
| 流控 | 无 |

### 支持的连接类型

1. **USB CDC** - 最常用 (STM32, RP2040)
2. **硬件 UART** - 用于 AVR 等
3. **Unix Socket** - Linux MCU 模拟器
4. **CAN Bus** - 需要额外的 CAN 桥接

---

## 帧格式

每个通信帧的结构：

```
┌──────────┬──────────────────┬────────────┬──────────┐
│  Length  │     Sequence     │   Payload  │   CRC16  │
│  1 byte  │     1 byte       │  N bytes   │  2 bytes │
└──────────┴──────────────────┴────────────┴──────────┘
```

### 字段说明

| 字段 | 大小 | 描述 |
|------|------|------|
| Length | 1 字节 | 帧总长度 (包含所有字段) |
| Sequence | 1 字节 | 高 4 位: 序列号, 低 4 位: 保留 |
| Payload | 可变 | 一个或多个编码后的消息 |
| CRC16 | 2 字节 | CRC-CCITT (多项式 0x1021, 初始值 0xFFFF) |

### 长度限制

- **最小帧长度**: 5 字节 (Length + Seq + CRC16 + 最小消息)
- **最大帧长度**: 64 字节 (MESSAGE_MAX = 64)
- **最大 Payload**: 60 字节

### 序列号

```
Sequence byte: [SEQ3][SEQ2][SEQ1][SEQ0][0][0][0][0]
               └─────── 序列号 ───────┘└── 保留 ──┘
```

- 序列号范围: 0-15 (4 位循环)
- 用于检测丢包和重传

### CRC16 计算

```c
uint16_t crc16_ccitt(uint8_t *buf, uint8_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*buf++) << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;
}
```

---

## 消息编码

### VLQ (Variable Length Quantity) 编码

Klipper 使用 VLQ 编码整数以节省空间：

```
值范围            编码字节数    格式
-64 到 63         1 字节       0XXXXXXX (7位有符号)
-8192 到 8191     2 字节       1XXXXXXX 0XXXXXXX
更大的值          3-5 字节     继续扩展...
```

**编码规则**:
- 最高位为 1 表示后续还有字节
- 最高位为 0 表示这是最后一个字节
- 使用补码表示负数

**编码示例**:

| 值 | 编码 (十六进制) |
|----|-----------------|
| 0 | 00 |
| 1 | 01 |
| 127 | 7F |
| 128 | 81 00 |
| -1 | 7F (补码) |
| -64 | 40 |
| 1000 | 87 68 |

**编码实现**:

```c
// 编码有符号整数到 VLQ
int encode_vlq(uint8_t *buf, int32_t v) {
    int len = 0;
    if (v >= -64 && v < 64) {
        buf[len++] = v & 0x7F;
    } else {
        // 多字节编码
        uint32_t uv = v;
        while (uv >= 0x80 || uv < 0xFFFFFF80) {
            buf[len++] = (uv & 0x7F) | 0x80;
            uv >>= 7;
        }
        buf[len++] = uv & 0x7F;
    }
    return len;
}

// 解码 VLQ 到有符号整数
int32_t decode_vlq(uint8_t **pp) {
    uint8_t *p = *pp;
    uint8_t c = *p++;
    int32_t v = c & 0x7F;
    if (c & 0x80) {
        c = *p++;
        v |= (c & 0x7F) << 7;
        if (c & 0x80) {
            c = *p++;
            v |= (c & 0x7F) << 14;
            if (c & 0x80) {
                c = *p++;
                v |= (c & 0x7F) << 21;
                if (c & 0x80) {
                    c = *p++;
                    v |= c << 28;
                }
            }
        }
    }
    // 符号扩展
    if (v & 0x40000000)
        v |= 0x80000000;
    *pp = p;
    return v;
}
```

### 消息结构

每条消息由 **命令 ID** + **参数** 组成：

```
┌────────────┬─────────┬─────────┬─────────┐
│ Command ID │ Param 1 │ Param 2 │   ...   │
│   (VLQ)    │  (VLQ)  │  (VLQ)  │         │
└────────────┴─────────┴─────────┴─────────┘
```

### 参数类型

| 类型 | 格式字符 | 编码方式 |
|------|----------|----------|
| uint32 | `%u` | VLQ 编码 |
| int32 | `%i` | VLQ 编码 (有符号) |
| uint16 | `%hu` | VLQ 编码 |
| uint8 (字节) | `%c` | 直接 1 字节 |
| 字符串 | `%s` | 长度(VLQ) + 数据 |
| 字节数组 | `%*s` | 长度(VLQ) + 数据 |

---

## 握手流程

### 步骤 1: 同步

MCU 启动后，主机发送同步请求：

```
发送: 0x00 0x00 0x00 0x00 0x00 ... (多个 0x00)
```

MCU 在收到足够多的 0x00 后发送 SYNC 响应。

### 步骤 2: IDENTIFY 请求

主机请求 MCU 标识信息：

```
命令: identify offset=%u count=%c
响应: identify_response offset=%u data=%*s
```

**IDENTIFY 数据格式** (压缩的 JSON):

```json
{
  "version": "v0.12.0-xxx",
  "build_versions": "gcc: ...",
  "mcu": "stm32f103",
  "freq": 72000000,
  "config": {
    "ADC_MAX": 4095,
    "BUS_PINS_i2c1": "PB6,PB7",
    "BUS_PINS_spi1": "PA6,PA7,PA5",
    "CLOCK_FREQ": 72000000,
    "MCU": "stm32f103",
    "PWM_MAX": 255,
    "RESERVE_PINS_serial": "PA9,PA10",
    "STATS_SUMSQ_BASE": 256,
    "STEPPER_BOTH_EDGE": 1
  },
  "commands": {
    "config_endstop oid=%c pin=%c pull_up=%c": 17,
    "config_stepper oid=%c step_pin=%c dir_pin=%c ...": 23,
    "queue_step oid=%c interval=%u count=%hu add=%hi": 45,
    ...
  },
  "responses": {
    "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c": 52,
    "clock clock=%u": 89,
    ...
  },
  "enumerations": {
    "pin": {
      "PA0": 0, "PA1": 1, "PA2": 2, ...
    },
    "bus": {
      "spi1": 0, "spi2": 1, "i2c1": 0, "i2c2": 1
    }
  }
}
```

**关键字段说明**:

| 字段 | 描述 |
|------|------|
| `version` | 固件版本号 |
| `mcu` | MCU 型号 |
| `freq` | MCU 时钟频率 (Hz) |
| `commands` | 支持的命令及其 ID |
| `responses` | MCU 响应消息及其 ID |
| `enumerations` | 引脚/总线名称到数值的映射 |
| `config` | MCU 配置常量 |

### 步骤 3: 配置阶段

主机发送配置命令序列：

```
1. allocate_oids count=%c
2. config_* 命令 (配置各种外设)
3. finalize_config crc=%u
```

### 步骤 4: 开始运行

配置完成后，MCU 进入运行状态，开始处理运动命令。

---

## 时钟同步

### 目的

主机需要知道 MCU 的当前时钟值，以便：
1. 调度定时操作 (步进脉冲、PWM 更新等)
2. 检测通信延迟
3. 检测 MCU 是否正常运行

### 同步协议

```
主机发送: get_clock
MCU响应:  clock clock=%u
```

### 时钟转换

**MCU 时钟 → 主机时间**:
```
host_time = mcu_clock / mcu_freq + clock_offset
```

**主机时间 → MCU 时钟**:
```
mcu_clock = (host_time - clock_offset) * mcu_freq
```

### 同步算法

1. 主机定期 (~1 秒) 发送 `get_clock`
2. 记录发送时间 `t_send` 和接收时间 `t_recv`
3. 计算往返时间 `rtt = t_recv - t_send`
4. 估计 MCU 时钟对应的主机时间: `t_mcu = (t_send + t_recv) / 2`
5. 使用多个样本进行线性回归，得到精确的 `clock_offset` 和 `mcu_freq`

### 心跳检测

- 主机每 ~0.98 秒发送 `get_clock`
- 如果连续 4 次无响应，视为 MCU 断连
- 触发紧急停止流程

---

## 命令参考

### 系统命令

#### identify
```
命令: identify offset=%u count=%c
响应: identify_response offset=%u data=%*s
```
分块读取 MCU 标识数据。

#### get_clock
```
命令: get_clock
响应: clock clock=%u
```
获取 MCU 当前时钟值。

#### get_uptime
```
命令: get_uptime
响应: uptime high=%u clock=%u
```
获取 MCU 运行时间 (64 位时钟值)。

#### emergency_stop
```
命令: emergency_stop
```
紧急停止，MCU 立即停止所有操作并进入关闭状态。

#### allocate_oids
```
命令: allocate_oids count=%c
```
预分配对象 ID (OID) 数量。

#### finalize_config
```
命令: finalize_config crc=%u
```
完成配置阶段，CRC 用于验证配置一致性。

#### get_config
```
命令: get_config
响应: config is_config=%c crc=%u is_shutdown=%c move_count=%hu
```
查询当前配置状态。

#### config_reset
```
命令: config_reset
```
重置 MCU 配置。

### GPIO 命令

#### config_digital_out
```
命令: config_digital_out oid=%c pin=%c value=%c default_value=%c max_duration=%u
```
配置数字输出引脚。

| 参数 | 描述 |
|------|------|
| oid | 对象 ID |
| pin | 引脚编号 (参考 enumerations) |
| value | 初始值 (0/1) |
| default_value | 超时后的默认值 |
| max_duration | 最大持续时钟数 (0=无限) |

#### queue_digital_out
```
命令: queue_digital_out oid=%c clock=%u on_ticks=%u
```
调度数字输出变化。

| 参数 | 描述 |
|------|------|
| oid | 对象 ID |
| clock | 执行时的 MCU 时钟值 |
| on_ticks | 高电平持续时钟数 |

#### config_pwm_out
```
命令: config_pwm_out oid=%c pin=%c cycle_ticks=%u value=%hu default_value=%hu max_duration=%u
```
配置 PWM 输出。

#### queue_pwm_out / schedule_pwm_out
```
命令: queue_pwm_out oid=%c clock=%u value=%hu
命令: schedule_pwm_out oid=%c clock=%u value=%hu
```
调度 PWM 值变化。

#### config_digital_in
```
命令: config_digital_in oid=%c pin=%c pull_up=%c
```
配置数字输入引脚。

### 步进电机命令

#### config_stepper
```
命令: config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%u
```
配置步进电机。

| 参数 | 描述 |
|------|------|
| oid | 对象 ID |
| step_pin | 步进脉冲引脚 |
| dir_pin | 方向引脚 (-1 表示无) |
| invert_step | 是否反转步进脉冲 |
| step_pulse_ticks | 脉冲持续时钟数 |

#### queue_step
```
命令: queue_step oid=%c interval=%u count=%hu add=%hi
```
调度步进脉冲序列。

| 参数 | 描述 |
|------|------|
| oid | 对象 ID |
| interval | 初始脉冲间隔 (时钟数) |
| count | 脉冲数量 |
| add | 每次脉冲后间隔的增量 (加速/减速) |

**时序计算**:
```
第 n 个脉冲时间 = start_clock + interval + (interval+add) + (interval+2*add) + ...
                = start_clock + n*interval + add*(0+1+2+...+(n-1))
                = start_clock + n*interval + add*n*(n-1)/2
```

#### set_next_step_dir
```
命令: set_next_step_dir oid=%c dir=%c
```
设置下一组步进的方向。

#### reset_step_clock
```
命令: reset_step_clock oid=%c clock=%u
```
重置步进时钟基准。

#### stepper_get_position
```
命令: stepper_get_position oid=%c
响应: stepper_position oid=%c pos=%i
```
获取步进电机当前位置。

### 限位开关命令

#### config_endstop
```
命令: config_endstop oid=%c pin=%c pull_up=%c stepper_count=%c
```
配置限位开关。

#### endstop_home
```
命令: endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c
```
开始归零操作。

#### endstop_query_state
```
命令: endstop_query_state oid=%c
响应: endstop_state oid=%c homing=%c next_clock=%u pin_value=%c
```
查询限位开关状态。

### 加热器命令

#### config_thermocouple / config_adc
```
命令: config_thermocouple oid=%c spi_oid=%c thermocouple_type=%c
命令: config_adc oid=%c pin=%c
```
配置温度传感器。

#### query_adc
```
命令: query_adc oid=%c clock=%u sample_ticks=%u sample_count=%c min_value=%hu max_value=%hu
响应: adc_state oid=%c next_clock=%u value=%hu
```
查询 ADC 值。

### SPI/I2C 命令

#### config_spi
```
命令: config_spi oid=%c bus=%c pin=%c mode=%c rate=%u
```
配置 SPI 总线。

#### spi_transfer
```
命令: spi_transfer oid=%c data=%*s
响应: spi_transfer_response oid=%c response=%*s
```
SPI 数据传输。

#### config_i2c
```
命令: config_i2c oid=%c bus=%c rate=%u addr=%c
```
配置 I2C 总线。

#### i2c_write
```
命令: i2c_write oid=%c data=%*s
```
I2C 写入数据。

#### i2c_read
```
命令: i2c_read oid=%c reg=%*s read_len=%c
响应: i2c_read_response oid=%c response=%*s
```
I2C 读取数据。

### 触发同步 (TRSync)

#### config_trsync
```
命令: config_trsync oid=%c
```
配置触发同步对象。

#### trsync_start
```
命令: trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c
```
启动触发同步。

#### trsync_set_timeout
```
命令: trsync_set_timeout oid=%c clock=%u
```
设置触发超时时间。

#### trsync_trigger
```
命令: trsync_trigger oid=%c reason=%c
```
手动触发。

---

## 响应消息

### 系统响应

#### clock
```
响应: clock clock=%u
```
当前 MCU 时钟值。

#### identify_response
```
响应: identify_response offset=%u data=%*s
```
标识数据块。

#### config
```
响应: config is_config=%c crc=%u is_shutdown=%c move_count=%hu
```
配置状态。

#### shutdown
```
响应: shutdown clock=%u static_string_id=%hu
```
MCU 关闭事件。

| static_string_id | 含义 |
|------------------|------|
| 0 | 命令请求 |
| 1 | Timer too close |
| 2 | ADC out of range |
| 3 | Config CRC mismatch |
| ... | (参考固件源码) |

### 状态响应

#### stepper_position
```
响应: stepper_position oid=%c pos=%i
```
步进电机位置。

#### endstop_state
```
响应: endstop_state oid=%c homing=%c next_clock=%u pin_value=%c
```
限位开关状态。

#### trsync_state
```
响应: trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u
```
触发同步状态。

#### adc_state
```
响应: adc_state oid=%c next_clock=%u value=%hu
```
ADC 读数。

---

## 错误处理

### 关闭 (Shutdown) 状态

MCU 进入 shutdown 状态的原因：

| 原因 | 描述 | 恢复方法 |
|------|------|----------|
| Timer too close | 命令调度时间太晚 | FIRMWARE_RESTART |
| ADC out of range | 温度传感器故障 | 检查硬件后 RESTART |
| Config CRC mismatch | 配置不一致 | 重新连接 |
| Command request | 主机请求关闭 | RESTART |
| Move queue empty | 运动队列空 (打印时) | RESTART |

### 恢复流程

1. 主机收到 `shutdown` 响应
2. 发送 `get_config` 确认状态
3. 根据类型选择恢复方式:
   - `RESTART`: 主机重新初始化连接
   - `FIRMWARE_RESTART`: MCU 硬件复位

### 重试机制

- 命令发送后等待确认 (ACK)
- 超时 (~100ms) 后重传
- 最多重试 5 次后断开连接

---

## 实现示例

### 最小 MCU 实现框架

```c
// mcu_main.c - 最小 Klipper MCU 实现

#include <stdint.h>
#include <string.h>

// ===========================================
// 协议常量
// ===========================================
#define MESSAGE_MAX 64
#define MESSAGE_HEADER_SIZE 2
#define MESSAGE_TRAILER_SIZE 3
#define MESSAGE_MIN (MESSAGE_HEADER_SIZE + MESSAGE_TRAILER_SIZE)

// ===========================================
// 全局状态
// ===========================================
static uint8_t rx_buf[MESSAGE_MAX];
static uint8_t tx_buf[MESSAGE_MAX];
static uint8_t rx_pos = 0;
static uint8_t tx_seq = 0;

static uint32_t mcu_clock = 0;
static uint8_t is_configured = 0;
static uint8_t is_shutdown = 0;

// ===========================================
// CRC16 计算
// ===========================================
uint16_t crc16_ccitt(uint8_t *buf, uint8_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*buf++) << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

// ===========================================
// VLQ 编解码
// ===========================================
int encode_vlq(uint8_t *buf, int32_t v) {
    if (v >= -64 && v < 64) {
        buf[0] = v & 0x7F;
        return 1;
    }
    // 简化实现: 假设值在合理范围内
    uint32_t uv = v;
    int len = 0;
    do {
        buf[len] = (uv & 0x7F) | (len ? 0x80 : 0);
        uv >>= 7;
        len++;
    } while (uv);
    // 反转字节顺序
    for (int i = 0; i < len/2; i++) {
        uint8_t t = buf[i];
        buf[i] = buf[len-1-i];
        buf[len-1-i] = t;
    }
    // 设置继续位
    for (int i = 0; i < len-1; i++)
        buf[i] |= 0x80;
    return len;
}

int32_t decode_vlq(uint8_t **pp) {
    uint8_t *p = *pp;
    uint32_t v = 0;
    while (1) {
        uint8_t c = *p++;
        v = (v << 7) | (c & 0x7F);
        if (!(c & 0x80))
            break;
    }
    *pp = p;
    // 符号扩展
    if (v & 0x40)
        v |= ~0x7F;
    return (int32_t)v;
}

// ===========================================
// 消息发送
// ===========================================
void send_response(uint8_t *data, uint8_t len) {
    uint8_t frame_len = len + MESSAGE_MIN;
    tx_buf[0] = frame_len;
    tx_buf[1] = (tx_seq++ & 0x0F) << 4;
    memcpy(tx_buf + 2, data, len);

    uint16_t crc = crc16_ccitt(tx_buf, frame_len - 2);
    tx_buf[frame_len - 2] = (crc >> 8) & 0xFF;
    tx_buf[frame_len - 1] = crc & 0xFF;

    uart_send(tx_buf, frame_len);
}

// ===========================================
// 命令处理
// ===========================================

// 命令 ID (从 identify 数据中获取)
#define CMD_GET_CLOCK       1
#define CMD_IDENTIFY        2
#define CMD_ALLOCATE_OIDS   3
#define CMD_FINALIZE_CONFIG 4
#define CMD_GET_CONFIG      5
#define CMD_EMERGENCY_STOP  6

// 响应 ID
#define RSP_CLOCK              80
#define RSP_IDENTIFY_RESPONSE  81
#define RSP_CONFIG             82
#define RSP_SHUTDOWN           83

void handle_get_clock(uint8_t *args) {
    uint8_t resp[8];
    int len = 0;
    len += encode_vlq(resp + len, RSP_CLOCK);
    len += encode_vlq(resp + len, mcu_clock);
    send_response(resp, len);
}

void handle_identify(uint8_t *args) {
    uint8_t *p = args;
    uint32_t offset = decode_vlq(&p);
    uint8_t count = *p++;

    // 返回 identify 数据块
    static const char identify_data[] =
        "{\"version\":\"v0.12.0\","
        "\"mcu\":\"generic\","
        "\"freq\":72000000,"
        "\"commands\":{\"get_clock\":1,\"identify offset=%u count=%c\":2},"
        "\"responses\":{\"clock clock=%u\":80,\"identify_response offset=%u data=%*s\":81}}";

    uint8_t resp[MESSAGE_MAX - MESSAGE_MIN];
    int len = 0;
    len += encode_vlq(resp + len, RSP_IDENTIFY_RESPONSE);
    len += encode_vlq(resp + len, offset);

    // 计算要发送的数据
    uint32_t data_len = sizeof(identify_data) - 1;
    uint32_t send_len = (offset + count > data_len) ?
                        (data_len > offset ? data_len - offset : 0) : count;

    len += encode_vlq(resp + len, send_len);
    if (send_len > 0)
        memcpy(resp + len, identify_data + offset, send_len);
    len += send_len;

    send_response(resp, len);
}

void handle_emergency_stop(uint8_t *args) {
    is_shutdown = 1;
    // 停止所有输出
    stop_all_outputs();

    // 发送 shutdown 响应
    uint8_t resp[8];
    int len = 0;
    len += encode_vlq(resp + len, RSP_SHUTDOWN);
    len += encode_vlq(resp + len, mcu_clock);
    len += encode_vlq(resp + len, 0); // static_string_id
    send_response(resp, len);
}

void process_command(uint8_t *payload, uint8_t len) {
    uint8_t *p = payload;
    uint8_t *end = payload + len;

    while (p < end) {
        int32_t cmd_id = decode_vlq(&p);

        switch (cmd_id) {
            case CMD_GET_CLOCK:
                handle_get_clock(p);
                break;
            case CMD_IDENTIFY:
                handle_identify(p);
                // 移动指针跳过参数
                decode_vlq(&p); // offset
                p++; // count
                break;
            case CMD_EMERGENCY_STOP:
                handle_emergency_stop(p);
                break;
            // ... 其他命令
            default:
                // 未知命令
                break;
        }
    }
}

// ===========================================
// 帧接收
// ===========================================
void receive_byte(uint8_t byte) {
    if (rx_pos == 0) {
        // 等待帧开始
        if (byte >= MESSAGE_MIN && byte <= MESSAGE_MAX)
            rx_buf[rx_pos++] = byte;
        return;
    }

    rx_buf[rx_pos++] = byte;

    if (rx_pos >= rx_buf[0]) {
        // 完整帧接收完毕
        uint8_t frame_len = rx_buf[0];

        // 验证 CRC
        uint16_t crc = crc16_ccitt(rx_buf, frame_len - 2);
        uint16_t recv_crc = (rx_buf[frame_len - 2] << 8) | rx_buf[frame_len - 1];

        if (crc == recv_crc) {
            // CRC 正确, 处理命令
            uint8_t payload_len = frame_len - MESSAGE_MIN;
            process_command(rx_buf + MESSAGE_HEADER_SIZE, payload_len);
        }

        rx_pos = 0;
    }
}

// ===========================================
// 主循环
// ===========================================
void mcu_main(void) {
    // 初始化硬件
    uart_init(250000);
    timer_init();

    while (1) {
        // 更新时钟
        mcu_clock = get_timer_ticks();

        // 处理串口接收
        while (uart_available()) {
            receive_byte(uart_read());
        }

        // 处理定时任务
        if (!is_shutdown) {
            process_scheduled_tasks();
        }
    }
}
```

### 步进电机实现

```c
// stepper.c - 步进电机控制实现

typedef struct {
    uint8_t oid;
    uint8_t step_pin;
    uint8_t dir_pin;
    uint8_t invert_step;
    uint32_t step_pulse_ticks;

    // 队列状态
    int32_t position;
    uint8_t direction;
    uint32_t next_step_clock;
    uint32_t interval;
    int16_t add;
    uint16_t count;
} stepper_t;

#define MAX_STEPPERS 8
static stepper_t steppers[MAX_STEPPERS];

void config_stepper(uint8_t oid, uint8_t step_pin, uint8_t dir_pin,
                   uint8_t invert_step, uint32_t step_pulse_ticks) {
    stepper_t *s = &steppers[oid];
    s->oid = oid;
    s->step_pin = step_pin;
    s->dir_pin = dir_pin;
    s->invert_step = invert_step;
    s->step_pulse_ticks = step_pulse_ticks;
    s->position = 0;
    s->count = 0;

    // 配置引脚为输出
    gpio_set_output(step_pin);
    if (dir_pin != 0xFF)
        gpio_set_output(dir_pin);
}

void queue_step(uint8_t oid, uint32_t interval, uint16_t count, int16_t add) {
    stepper_t *s = &steppers[oid];
    s->interval = interval;
    s->count = count;
    s->add = add;
    s->next_step_clock = mcu_clock + interval;
}

void set_next_step_dir(uint8_t oid, uint8_t dir) {
    stepper_t *s = &steppers[oid];
    s->direction = dir;
    if (s->dir_pin != 0xFF)
        gpio_write(s->dir_pin, dir);
}

void process_steppers(void) {
    for (int i = 0; i < MAX_STEPPERS; i++) {
        stepper_t *s = &steppers[i];
        if (s->count == 0)
            continue;

        if ((int32_t)(mcu_clock - s->next_step_clock) >= 0) {
            // 执行步进脉冲
            gpio_write(s->step_pin, !s->invert_step);

            // 保持脉冲宽度
            delay_ticks(s->step_pulse_ticks);

            gpio_write(s->step_pin, s->invert_step);

            // 更新位置
            s->position += s->direction ? 1 : -1;

            // 计算下一个脉冲时间
            s->interval += s->add;
            s->next_step_clock += s->interval;
            s->count--;
        }
    }
}
```

---

## 附录

### A. 常用引脚映射示例

**STM32F103**:
```
PA0=0, PA1=1, ..., PA15=15
PB0=16, PB1=17, ..., PB15=31
PC0=32, PC1=33, ..., PC15=47
```

**RP2040**:
```
GP0=0, GP1=1, ..., GP29=29
```

### B. 常见问题

**Q: 如何处理命令队列溢出?**
A: MCU 应维护命令队列深度计数，当接近满时发送 `move_count` 反馈给主机，主机会降低发送速率。

**Q: 如何实现多个 MCU?**
A: 每个 MCU 独立连接，主机通过不同串口/CAN 地址区分。配置中使用 `[mcu name]` 定义额外 MCU。

**Q: 如何调试通信问题?**
A: 启用 Klipper 的 `-v` 参数查看详细日志，或使用逻辑分析仪抓取串口数据。

### C. 参考资源

- Klipper 官方文档: https://www.klipper3d.org/Protocol.html
- Klipper MCU 源码: `src/` 目录
- 数据字典格式: `klippy/extras/mcu.py`

---

**文档版本**: 1.0
**最后更新**: 2026-01-21
**作者**: Klipper Go Migration Project
