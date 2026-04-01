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

// 无 BUTTON_BIND 定义的板子（如 3dr1060/Si1060）整个实现体为空，
// 接口由 bind.h 中的宏提供，彻底零 XDATA 占用。
#ifdef BUTTON_BIND

// 复用射频驱动内部的收发缓冲区（已在 XDATA，252字节）
// 对频模式与 TDM 模式互斥，radio_buffer 在对频期间空闲，可安全覆盖使用
// 避免额外申请 ~77 字节 XDATA，防止 Si1060（XDATA 仅 2KB）溢出
extern __xdata uint8_t radio_buffer[];
#define bind_buf (*((__xdata struct bind_packet *)(radio_buffer)))

// ─── 模块级状态变量（__pdata 存于 PDATA，减少 IDATA 压力）─────────────────────

__pdata enum BindState bind_state;       // 当前对频状态（外部可读）

// 按键检测
// 注意：硬件通过 C30(100nF) + R6(10kΩ) 对按键做 AC 耦合，τ = 1ms
// P1.7 在按键按下时只出现约 3~5ms 的低电平脉冲，随后被 R6 拉回高电平。
// 哪怕持续按住，P1.7 也只有开始那 ~1ms 是低电平，后续全程高电平。
// 因此必须用"下降沿检测"而非"长按计时"。
static __pdata bool button_prev_state;         // 上一次检测到的引脚电平（true=低电平/激活）

// 对频超时计时（uint32_t 积累，避免 uint16_t 溢出）
static __pdata uint32_t timeout_acc_ticks;
static __pdata uint16_t timeout_last_tick;

// 主机发包间隔计时
static __pdata uint16_t tx_last_tick;

// 主机已发包计数：发够 BIND_MASTER_TX_COUNT 包后自动软件复位
// 确保从机有充足时间收到至少一包并完成参数保存，主机随后也重启生效
static __pdata uint8_t  bind_sent_count;
#define BIND_MASTER_TX_COUNT  5   // 发 5 包后复位（间隔由 bind_runtime_tx_interval 决定）

// 本次对频会话的发送超时/发包间隔（tick 单位，16µs/tick）
// 由 bind_refresh_air_timings() 在 bind_enter() 中根据 radio_air_rate() 计算填入
static __pdata uint16_t bind_runtime_tx_timeout;
static __pdata uint16_t bind_runtime_tx_interval;

// LED 闪烁计时
static __pdata uint16_t led_last_tick;
static __pdata bool     led_state;

// ─── 内部函数前向声明 ─────────────────────────────────────────────────────────
static void bind_enter(void);
static void bind_exit(void);
static void bind_tick_send(void);
static void bind_tick_listen(void);
static void bind_on_success(void);
static void bind_led_update(void);
static uint32_t bind_get_base_freq(void);
static uint32_t bind_get_carrier_hz(void);
static void bind_refresh_air_timings(void);

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

// ─── bind_get_carrier_hz ───────────────────────────────────────────────────────
// 对频单频载波：优先使用本机 Flash 里与正常跳频一致的 PARAM_MIN_FREQ（单位为 kHz），
// 这样与“能连上的那套参数”同频，避免硬编码 433.05MHz 与用户 S8 不一致时两边各听各的。
// 若参数明显非法，则退回 bind_get_base_freq()（按 Bootloader 频段字典型值）。
static uint32_t
bind_get_carrier_hz(void)
{
    __pdata uint32_t khz;

    khz = (uint32_t)param_get(PARAM_MIN_FREQ);
    // 合理范围：100MHz～960MHz 量级（存 kHz 则约 1e5～1e6）
    if (khz < 100000UL || khz > 960000UL) {
        return bind_get_base_freq();
    }
    return khz * 1000UL;
}

// ─── bind_refresh_air_timings ────────────────────────────────────────────────
// radio_configure() 之后调用：根据实际 effective 空口速率（radio_air_rate，单位 kbps）
// 估算 bind_packet 的空口时间，填充发送超时与发包间隔。
// 说明：原先强制 2kbps，在部分 Si443x 寄存器组合下解调/ AFC 余量小，且与日常 64kbps 链路不一致；
//      改为与 PARAM_AIR_SPEED 一致后，调制与正常工作时相同，最容易互通。
static void
bind_refresh_air_timings(void)
{
    __pdata uint8_t  kbps;
    __pdata uint16_t pay;
    __pdata uint32_t bits, us, t, iv;

    kbps = radio_air_rate();
    if (kbps == 0) {
        kbps = 64;
    }

    pay  = (uint16_t)sizeof(struct bind_packet);
    bits = (uint32_t)(pay + 24) * 8UL;
    us   = (bits * 1000UL + (uint32_t)kbps - 1UL) / (uint32_t)kbps;
    us  *= 2UL;

    t = (us + 15UL) / 16UL;
    if (t < 5000UL) {
        t = 5000UL;
    }
    if (t > 65535UL) {
        t = 65535UL;
    }
    bind_runtime_tx_timeout = (uint16_t)t;

    iv = (uint32_t)bind_runtime_tx_timeout + (200000UL / 16UL);
    if (iv > 65535UL) {
        iv = 65535UL;
    }
    bind_runtime_tx_interval = (uint16_t)iv;
}

// ─── bind_init ────────────────────────────────────────────────────────────────
// 初始化对频模块所有状态变量（在 main() 中 hardware_init 后调用一次）
void
bind_init(void)
{
    bind_state         = BIND_STATE_IDLE;
    // 初始化为"未按下"状态（高电平，与 BUTTON_ACTIVE=0 相反）
    button_prev_state  = false;
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
    bind_sent_count   = 0;

    // ── 切换射频到对频公共配置 ──────────────────────────────────────
    //
    // 重要：radio_configure() 内部会做两件事需要在之后修正：
    //   1. 用 settings.frequency 回写频率寄存器（覆盖为旧值）
    //   2. 调用 radio_set_transmit_power(0) 将功率重置为最低档
    //
    // 因此必须在 radio_configure() 之后重新设置频率、NETID 和发射功率。

    // 第1步：先设置对频频率到 settings 结构体，
    //        这样 radio_configure 内部回写频率时用的就是对频频率
    radio_set_frequency(bind_get_carrier_hz(), 0);     // 与 S8 MIN_FREQ 一致的单频点
    radio_set_channel(0);                              // 信道 0（spacing=0 时仍为基频）

    // 第2步：配置空口速率——与 PARAM_AIR_SPEED 一致（含 Manchester 等由 radio_configure 读参），
    //        避免固定 2kbps 与日常链路不一致导致解调失败。
    radio_configure((uint8_t)param_get(PARAM_AIR_SPEED));

    // 第2b 步：按实际 kbps 计算本次对频的发送超时与发包间隔
    bind_refresh_air_timings();

    // 第3步：radio_configure 把功率重置为 0 了，必须恢复
    radio_set_transmit_power((uint8_t)param_get(PARAM_TXPOWER));

    // 第4步：设置公共 NETID（必须在 radio_configure 之后，
    //        因为 radio_configure 会重新配置 header 相关寄存器）
    radio_set_network_id(BIND_NETID);                   // 公共 NETID 0xFFFF

    // 第5步：开启接收
    radio_receiver_on();
}

// ─── bind_exit ────────────────────────────────────────────────────────────────
// 退出对频模式（超时）：恢复原有射频参数，重新初始化 FHSS，回到正常状态
static void
bind_exit(void)
{
    bind_state = BIND_STATE_TIMEOUT;

    // 恢复原有射频配置（从 Flash 已保存的参数中读取）
    // 注意：radio_configure() 内部会把功率重置为 0，必须之后恢复
    radio_configure((uint8_t)param_get(PARAM_AIR_SPEED));
    radio_set_transmit_power((uint8_t)param_get(PARAM_TXPOWER));
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

    // 将对频包里的参数逐个写入本机，但跳过以下两个参数：
    //   - PARAM_FORMAT    (i=0)：由系统版本号机制自动管理，不能覆盖
    //   - PARAM_BIND_ROLE (S16)：从机必须保留自己的角色（0=从机），
    //                    不能被主机的 BIND_ROLE=1 覆盖，否则下次对频变成双主机。
    for (i = 1; i < PARAM_MAX; i++) {
        if (i == PARAM_BIND_ROLE) {
            continue;   // 保留本机的对频角色，不从主机覆盖
        }
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

    if (delta < bind_runtime_tx_interval) {
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
    bind_buf.crc = crc16(crc_len, radio_buffer);

    // 发送（超时随空口速率在变，见 bind_refresh_air_timings）
    radio_transmit((uint8_t)sizeof(bind_buf),
                   radio_buffer,
                   bind_runtime_tx_timeout);

    radio_receiver_on();    // 发完后回到接收，防止射频卡在 TX 状态

    // 主机自动复位逻辑：
    // 从机收包后需约 1 秒完成 param_save + 绿灯反馈 + 软件复位。
    // 主机发够 BIND_MASTER_TX_COUNT 包后自动绿灯闪一下并软件复位，
    // 无需用户手动断电重启。
    bind_sent_count++;
    if (bind_sent_count >= BIND_MASTER_TX_COUNT) {
        LED_RED   = LED_OFF;
        LED_GREEN = LED_ON;
        delay_msec(1000);       // 绿灯 1 秒，与从机反馈一致
        RSTSRC |= (1 << 4);    // 软件复位
        for (;;)
            ;
    }
}

// ─── bind_tick_listen ────────────────────────────────────────────────────────
// 从机接收逻辑：持续监听，收到包后进行三重校验，通过则覆盖参数并复位
static void
bind_tick_listen(void)
{
    __pdata uint8_t  rx_len;
    __pdata uint16_t computed_crc;
    __pdata uint8_t  crc_len;

    if (!radio_receive_packet(&rx_len, radio_buffer)) {
        // 不要调用 radio_receiver_on()！
        // 该函数会清空 FIFO 并重置接收状态，在紧密循环中反复调用
        // 会导致正在接收中的包被丢弃（低速率时一包可达数百毫秒）。
        // 接收机已在 bind_enter() 中开启，无需重复操作。
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
    computed_crc = crc16(crc_len, radio_buffer);
    if (computed_crc != bind_buf.crc) {
        radio_receiver_on();
        return;
    }

    // ── 所有校验通过，执行对频成功 ──────────────────────────────────
    bind_on_success();      // 此函数内部会软件复位，不会返回
}

// ─── bind_check_button ───────────────────────────────────────────────────────
// 按键扫描（下降沿检测）
//
// 硬件背景：PAIR 按键通过 C30(100nF) + R6(10kΩ) AC 耦合到 P1.7，
//   按键按下时 P1.7 只出现约 3~5ms 的低电平脉冲（τ=R6×C30=1ms），
//   随后 R6 将其拉回高电平。因此无法使用"长按计时"，
//   必须检测从"未激活"→"激活"的下降沿，立即进入对频模式。
void
bind_check_button(void)
{
#ifdef BUTTON_BIND
    __pdata bool cur_state;

    // 对频模式中不检测按键（防止重入）
    if (bind_mode_active()) {
        // 确保 prev 同步，避免退出对频后误触发
        button_prev_state = (BUTTON_BIND == BUTTON_ACTIVE);
        return;
    }

    cur_state = (BUTTON_BIND == BUTTON_ACTIVE);  // true = 低电平 = 被按下

    // 下降沿检测：上次未激活、这次激活 → 按键刚被按下
    if (cur_state && !button_prev_state) {
        bind_enter();
    }

    button_prev_state = cur_state;
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

#endif // BUTTON_BIND
