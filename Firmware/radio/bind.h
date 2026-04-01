// -*- Mode: C; c-basic-offset: 8; -*-
//
// bind.h — 一键对频功能模块头文件
//
// 设计逻辑：
//   主机（BIND_ROLE=1）长按按键 → 广播本机完整参数表（param_values[]）
//   从机（BIND_ROLE=0）长按按键 → 监听，收到合法对频包后覆盖本机参数并复位
//
// 对频公共信道（双方必须事先一致）：
//   NETID = 0xFFFF，AIR_SPEED = 2kbps，信道 0，频段基准频率

#ifndef _BIND_H_
#define _BIND_H_

#include "radio.h"
#include "parameters.h"

// ─── 对频状态 ─────────────────────────────────────────────────────────────────
enum BindState {
    BIND_STATE_IDLE      = 0,  // 正常运行，未对频
    BIND_STATE_SENDING   = 1,  // 主机：正在广播参数
    BIND_STATE_LISTENING = 2,  // 从机：正在监听
    BIND_STATE_SUCCESS   = 3,  // 对频成功（即将复位）
    BIND_STATE_TIMEOUT   = 4   // 对频超时，已自动退出
};

// ─── 对频空中包结构（整表同步）────────────────────────────────────────────────
// 总大小 ≈ 2+1+(PARAM_MAX×4)+2 = ~69 字节，远小于 MAX_PACKET_LENGTH(252)
struct bind_packet {
    uint8_t  magic[2];           // 魔术字：{0xB1, 0xD0}，用于识别对频包
    uint8_t  board_frequency;    // 发送方板级频段（防止 433/915 互相干扰）
    param_t  params[PARAM_MAX];  // 发送方完整参数表拷贝
    uint16_t crc;                // CRC16，覆盖 magic+board_frequency+params
};

// ─── 常量定义 ─────────────────────────────────────────────────────────────────
#define BIND_MAGIC_0              0xB1U

#define BIND_MAGIC_1              0xD0U

// 对频公共射频配置
#define BIND_NETID                0xFFFFU   // 公共网络 ID（硬件包头过滤）
// 空口速率：由 bind.c 使用本机 PARAM_AIR_SPEED（与正常链路一致），不再强制 2kbps。
#define BIND_AIR_SPEED            2         // 保留宏：历史/文档用（对频实现已不用固定 2）

// 定时参数（单位：16us 的 timer2 ticks）
// 3s 长按   = 3,000,000us / 16us = 187,500 ticks
// 30s 超时  = 30,000,000us / 16us = 1,875,000 ticks
#define BIND_LONG_PRESS_TICKS     187500UL
#define BIND_TIMEOUT_TICKS        1875000UL

// 发送超时 / 发包间隔：由 bind_refresh_air_timings() 按 radio_air_rate() 运行时计算，
// 以下常量仅作参考，不要在 bind.c 中使用。
#define BIND_TX_TIMEOUT_TICKS_REF     37500U
#define BIND_TX_INTERVAL_TICKS_REF    50000U

// LED 闪烁间隔 200ms（纯视觉反馈，与射频无关）
#define BIND_LED_INTERVAL_TICKS   12500U

// ─── 公共接口（条件编译）──────────────────────────────────────────────────────
#ifdef BUTTON_BIND
// 有对频按键硬件（如 hm_trp）：使用 bind.c 中的完整实现
extern __pdata enum BindState bind_state;
extern void bind_init(void);
extern void bind_check_button(void);
extern bool bind_mode_active(void);
extern void bind_tick(void);
extern void bind_enter_from_at(void);
#else
// 无对频按键硬件（如 3dr1060 / Si1060）：所有接口退化为零开销宏
// 不占用任何 XDATA/PDATA，彻底避免 Si1060 XDATA 溢出
#define bind_init()          do {} while(0)
#define bind_mode_active()   (0)
#define bind_check_button()  do {} while(0)
#define bind_tick()          do {} while(0)
#define bind_enter_from_at() do {} while(0)
#endif // BUTTON_BIND

#endif // _BIND_H_
