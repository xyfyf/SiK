// -*- Mode: C; c-basic-offset: 8; -*-
//
// bind.c — 一键对频功能核心实现
//
// 流程：
//   1. bind_check_button()：每次 TDM 循环调用，检测长按（3s）→ bind_enter()
//   2. bind_enter()：停止正常 TDM，切换射频到公共配置，进入发送或监听状态
//   3. bind_tick()：
//        主机（SENDING）  → 每 200ms 广播一次 bind_packet（含完整参数表）
//        从机（LISTENING）→ 持续接收，通过三重校验后调用 bind_on_success()
//   4. bind_on_success()：写入新参数 → param_save() → LED 绿灯 1s → 软件复位
//   5. bind_exit()：超时时恢复原有射频配置，回到正常运行

#include "board.h"          // LED_RED/LED_GREEN/LED_ON/OFF, BUTTON_BIND, RSTSRC 等
#include "radio.h"          // radio_transmit/receive/configure 等
#include "parameters.h"     // param_get/set/save
#include "timer.h"          // timer2_tick(), delay_msec()
#include "crc.h"            // crc16()
#include "freq_hopping.h"   // fhop_init()
#include "bind.h"

// ─── 模块级状态变量（__pdata 存于 PDATA，减少 IDATA 压力）─────────────────────

__pdata enum BindState bind_state;       // 当前对频状态（外部可读）

// 按键检测
static __pdata bool     button_was_pressed;    // 是否已处于按下状态
static __pdata uint32_t press_acc_ticks;       // 积累的按键持续 tick 数
static __pdata uint16_t press_last_tick;       // 上一次检测时的 timer2_tick 值

// 对频超时计时（uint32_t 积累，避免 uint16_t 溢出）
static __pdata uint32_t timeout_acc_ticks;
static __pdata uint16_t timeout_last_tick;

// 主机发包间隔计时（200ms < 1.05s，uint16_t 安全）
static __pdata uint16_t tx_last_tick;

// LED 闪烁计时
static __pdata uint16_t led_last_tick;
static __pdata bool     led_state;

// 对频包收发缓冲区（必须在 __xdata，radio_transmit/receive 要求）
static __xdata struct bind_packet bind_buf;

// ─── 内部函数前向声明 ─────────────────────────────────────────────────────────
static void bind_enter(void);
static void bind_exit(void);
static void bind_tick_send(void);
static void bind_tick_listen(void);
static void bind_on_success(void);
static void bind_led_update(void);
static uint32_t bind_get_base_freq(void);

// ─── bind_get_base_freq ───────────────────────────────────────────────────────
// 根据板子频段返回对频专用固定基准频率（Hz）
// 双方必须使用相同频段（硬件保证），所以此处硬编码频段基准频率是安全的
static uint32_t
bind_get_base_freq(void)
{
    switch (g_board_frequency) {
    case FREQ_433: return 433050000UL;
    case FREQ_470: return 470000000UL;
    case FREQ_868: return 868000000UL;
    case FREQ_915: return 915000000UL;
    default:       return 433050000UL;
    }
}

// ─── bind_init ────────────────────────────────────────────────────────────────
// 初始化对频模块所有状态变量（在 main() 中 hardware_init 后调用一次）
void
bind_init(void)
{
    bind_state         = BIND_STATE_IDLE;
    button_was_pressed = false;
    press_acc_ticks    = 0;
    press_last_tick    = timer2_tick();
}

// ─── bind_mode_active ─────────────────────────────────────────────────────────
// 供 TDM 主循环判断：是否应跳过正常 TDM 逻辑，转由 bind_tick() 接管
bool
bind_mode_active(void)
{
    return (bind_state == BIND_STATE_SENDING ||
            bind_state == BIND_STATE_LISTENING);
}

// ─── bind_enter ───────────────────────────────────────────────────────────────
// 进入对频模式：切换射频到公共信道/速率/NETID，停止 FHSS 跳频
static void
bind_enter(void)
{
    // 根据 BIND_ROLE 参数决定角色
    // ATS16=1 → 主机（发送参数）；ATS16=0（默认）→ 从机（接收参数）
    if (param_get(PARAM_BIND_ROLE) == 1) {
        bind_state = BIND_STATE_SENDING;
    } else {
        bind_state = BIND_STATE_LISTENING;
    }

    // 初始化对频计时器
    timeout_acc_ticks = 0;
    timeout_last_tick = timer2_tick();
    tx_last_tick      = timer2_tick();
    led_last_tick     = timer2_tick();
    led_state         = false;

    // ── 切换射频到对频公共配置 ──────────────────────────────────────
    // 顺序：先设速率（影响寄存器配置），再设 NETID 和频率
    radio_configure(BIND_AIR_SPEED);                          // 2kbps 最低速率
    radio_set_network_id(BIND_NETID);                         // 公共 NETID 0xFFFF
    radio_set_frequency(bind_get_base_freq(), 0);             // 固定单频点，不跳频
    radio_set_channel(0);                                     // 信道 0
    radio_receiver_on();                                      // 切换到接收等待状态
}

// ─── bind_exit ────────────────────────────────────────────────────────────────
// 退出对频模式（超时）：恢复原有射频参数，重新初始化 FHSS，回到正常状态
static void
bind_exit(void)
{
    bind_state = BIND_STATE_TIMEOUT;

    // 恢复原有射频配置（从 Flash 已保存的参数中读取）
    radio_configure((uint8_t)param_get(PARAM_AIR_SPEED));
    radio_set_network_id((uint16_t)param_get(PARAM_NETID));
    fhop_init();            // 重新初始化跳频表（内部读 param_get 重建种子和信道）
    radio_receiver_on();

    // 熄灭 LED（由 TDM link_update 接管）
    LED_RED   = LED_OFF;
    LED_GREEN = LED_OFF;

    bind_state = BIND_STATE_IDLE;
}

// ─── bind_on_success ──────────────────────────────────────────────────────────
// 对频成功：将收到的参数整体覆盖本机，保存到 Flash，绿灯反馈，然后软件复位
static void
bind_on_success(void)
{
    __pdata uint8_t i;

    bind_state = BIND_STATE_SUCCESS;

    // 将对频包里的参数逐个写入本机（跳过 PARAM_FORMAT，由系统版本号机制自动管理）
    for (i = 1; i < PARAM_MAX; i++) {
        param_set(i, bind_buf.params[i]);
    }
    param_save();   // 写入 Flash

    // 绿灯常亮约 1 秒，给用户直观的成功反馈
    LED_RED   = LED_OFF;
    LED_GREEN = LED_ON;
    delay_msec(1000);

    // 软件复位：使所有新参数（AIR_SPEED、NETID、频段等）全部生效
    RSTSRC |= (1 << 4);
    for (;;)
        ;
}

// ─── bind_led_update ─────────────────────────────────────────────────────────
// 对频模式 LED 视觉反馈：红绿交替快闪（200ms/次）
static void
bind_led_update(void)
{
    __pdata uint16_t now   = timer2_tick();
    __pdata uint16_t delta = (uint16_t)(now - led_last_tick);

    if (delta < BIND_LED_INTERVAL_TICKS) {
        return;
    }
    led_last_tick = now;

    led_state = !led_state;
    LED_RED   = led_state ? LED_ON  : LED_OFF;
    LED_GREEN = led_state ? LED_OFF : LED_ON;
}

// ─── bind_tick_send ───────────────────────────────────────────────────────────
// 主机发送逻辑：每 200ms 组装并广播一次 bind_packet（完整参数表）
static void
bind_tick_send(void)
{
    __pdata uint16_t now   = timer2_tick();
    __pdata uint16_t delta = (uint16_t)(now - tx_last_tick);
    __pdata uint8_t  i;
    __pdata uint8_t  crc_len;

    if (delta < BIND_TX_INTERVAL_TICKS) {
        return;     // 发包间隔未到，等待
    }
    tx_last_tick = now;

    // ── 组装 bind_packet ────────────────────────────────────────────
    bind_buf.magic[0]        = BIND_MAGIC_0;
    bind_buf.magic[1]        = BIND_MAGIC_1;
    bind_buf.board_frequency = (uint8_t)g_board_frequency;

    // 将本机当前参数表完整拷贝（包含 NETID、AIR_SPEED、MIN/MAX_FREQ 等）
    for (i = 0; i < PARAM_MAX; i++) {
        bind_buf.params[i] = param_get(i);
    }

    // CRC 覆盖范围：magic + board_frequency + params（不含 crc 字段本身）
    crc_len      = (uint8_t)(sizeof(bind_buf) - sizeof(bind_buf.crc));
    bind_buf.crc = crc16(crc_len, (__xdata uint8_t *)&bind_buf);

    // 发送（超时 50ms = 3125 ticks）
    radio_transmit((uint8_t)sizeof(bind_buf),
                   (__xdata uint8_t *)&bind_buf,
                   BIND_TX_TIMEOUT_TICKS);

    radio_receiver_on();    // 发完后回到接收，防止射频卡在 TX 状态
}

// ─── bind_tick_listen ────────────────────────────────────────────────────────
// 从机接收逻辑：持续监听，收到包后进行三重校验，通过则覆盖参数并复位
static void
bind_tick_listen(void)
{
    __pdata uint8_t  rx_len;
    __pdata uint16_t computed_crc;
    __pdata uint8_t  crc_len;

    if (!radio_receive_packet(&rx_len, (__xdata uint8_t *)&bind_buf)) {
        radio_receiver_on();    // 没收到包，继续监听
        return;
    }

    // ── 三重校验 ────────────────────────────────────────────────────

    // 校验 1：包长度匹配
    if (rx_len != (uint8_t)sizeof(bind_buf)) {
        radio_receiver_on();
        return;
    }

    // 校验 2：魔术字（过滤非对频包）
    if (bind_buf.magic[0] != BIND_MAGIC_0 ||
        bind_buf.magic[1] != BIND_MAGIC_1) {
        radio_receiver_on();
        return;
    }

    // 校验 3：频段匹配（防止 433MHz 板误收 915MHz 参数，参数非法会损坏硬件）
    if (bind_buf.board_frequency != (uint8_t)g_board_frequency) {
        radio_receiver_on();
        return;
    }

    // 校验 4：CRC16 完整性
    crc_len      = (uint8_t)(sizeof(bind_buf) - sizeof(bind_buf.crc));
    computed_crc = crc16(crc_len, (__xdata uint8_t *)&bind_buf);
    if (computed_crc != bind_buf.crc) {
        radio_receiver_on();
        return;
    }

    // ── 所有校验通过，执行对频成功 ──────────────────────────────────
    bind_on_success();      // 此函数内部会软件复位，不会返回
}

// ─── bind_check_button ───────────────────────────────────────────────────────
// 按键扫描与长按检测（在 TDM 主循环每次调用，与循环频率解耦）
// 使用 uint32_t 积累 ticks，正确处理 timer2_tick() uint16_t 溢出
void
bind_check_button(void)
{
#ifdef BUTTON_BIND
    __pdata uint16_t now;
    __pdata uint16_t delta;

    // 对频模式中不检测按键（防止重入）
    if (bind_mode_active()) {
        button_was_pressed = false;
        press_acc_ticks    = 0;
        press_last_tick    = timer2_tick();
        return;
    }

    now   = timer2_tick();
    delta = (uint16_t)(now - press_last_tick);  // 自动处理 uint16_t 溢出
    press_last_tick = now;

    if (BUTTON_BIND == BUTTON_ACTIVE) {
        // ── 按键被按下 ────────────────────────────────────────────
        if (!button_was_pressed) {
            // 刚被按下，开始计时
            button_was_pressed = true;
            press_acc_ticks    = 0;
        } else {
            // 持续按下，积累时间
            press_acc_ticks += delta;
            if (press_acc_ticks >= BIND_LONG_PRESS_TICKS) {
                // 长按满 3 秒，进入对频模式
                button_was_pressed = false;
                press_acc_ticks    = 0;
                bind_enter();
            }
        }
    } else {
        // ── 按键松开或未按下，清零计时 ──────────────────────────────
        button_was_pressed = false;
        press_acc_ticks    = 0;
    }
#endif // BUTTON_BIND
}

// ─── bind_tick ───────────────────────────────────────────────────────────────
// 对频状态机主入口（TDM 主循环在 bind_mode_active() 为真时调用，替代 TDM 逻辑）
void
bind_tick(void)
{
    __pdata uint16_t now;
    __pdata uint16_t delta;

    // ── 更新超时累计（uint32_t 处理长时间，不溢出）────────────────
    now   = timer2_tick();
    delta = (uint16_t)(now - timeout_last_tick);
    timeout_last_tick  = now;
    timeout_acc_ticks += delta;

    if (timeout_acc_ticks >= BIND_TIMEOUT_TICKS) {
        // 30 秒无结果，退出对频，恢复正常工作
        bind_exit();
        return;
    }

    bind_led_update();  // 更新 LED 闪烁

    // ── 按角色分发 ───────────────────────────────────────────────
    if (bind_state == BIND_STATE_SENDING) {
        bind_tick_send();
    } else if (bind_state == BIND_STATE_LISTENING) {
        bind_tick_listen();
    }
}

// ─── bind_enter_from_at ──────────────────────────────────────────────────────
// AT 指令（ATB）触发入口，允许上位机通过串口命令远程触发对频
void
bind_enter_from_at(void)
{
    if (!bind_mode_active()) {
        bind_enter();
    }
}
