# SiK 固件 "一键对频" 功能实现方案（类似 ELRS Bind）

## 一、可行性结论

**完全可行。** 经过对 SiK 固件全部核心源码的逐行分析，现有架构天然支持这一功能的扩展：
- 参数系统（`param_set/save/load`）已经具备全部射频参数的读写和持久化能力
- 射频底层（`radio_set_network_id`、`radio_set_frequency`、`radio_set_channel`）支持运行时动态切换
- 跳频系统（`fhop_init`）以 NETID 为种子生成跳频表，参数更新后重新初始化即可
- 构建系统使用 `wildcard` 自动编译 `Firmware/radio/` 目录下所有 `.c` 文件，新增源码无需修改 Makefile

---

## 二、核心设计原理：对频到底需要同步什么？

SiK 两端要建立通信链路，**不是仅靠 NETID 一个数字**，而是有一整套参数必须完全一致。
NETID 在系统中承担三重角色（见 `main.c` 第 427-453 行）：

1. **硬件包头过滤**：`radio_set_network_id(NETID)` → 不同 NETID 的包在射频芯片硬件层直接丢弃
2. **跳频表种子**：`r_srand(NETID)` → 生成伪随机跳频序列，NETID 不同则跳频表完全不同
3. **频率偏移**：`freq_min += r_rand() % channel_spacing` → 基于 NETID 产生频率偏移，进一步拉开不同网络

但即使 NETID 相同，以下参数不一致时仍然**无法通信**：

| 参数 | 代码中的定义 | 不一致的后果 |
|:---|:---|:---|
| `AIR_SPEED` | `PARAM_AIR_SPEED` | 调制速率不同，物理层完全解不了码 |
| `MIN_FREQ` | `PARAM_MIN_FREQ` | 频率范围不同，跳频映射到不同的物理频率 |
| `MAX_FREQ` | `PARAM_MAX_FREQ` | 同上 |
| `NUM_CHANNELS` | `PARAM_NUM_CHANNELS` | 信道数不同，跳频表长度不同，永远对不齐 |
| `ECC` | `PARAM_ECC` | 一端做 Golay 编码另一端不解，数据全乱 |
| `MANCHESTER` | `PARAM_MANCHESTER` | 曼彻斯特编码不一致，物理层无法识别 |
| `MAX_WINDOW` | `PARAM_MAX_WINDOW` | TDM 窗口大小不匹配，时序混乱 |
| `MAVLINK` | `PARAM_MAVLINK` | 帧格式不同，应用层解析失败 |
| `OPPRESEND` | `PARAM_OPPRESEND` | 重传策略不同，可能导致死锁 |
| 加密密钥 | `PARAM_ENCRYPTION` + key | 密钥不同则解密出垃圾数据 |

**因此，对频包的设计核心是：主机将自己的整个 `parameter_values[]` 参数表通过空中发给从机，从机整体覆盖后保存并重启。**

---

## 三、需要修改/新增的文件清单

| 序号 | 文件路径 | 操作 | 预估修改行数 | 修改难度 |
|:---:|:---|:---:|:---:|:---:|
| 1 | `Firmware/include/board_hm_trp.h` | **修改** | ~15 行 | 简单 |
| 2 | `Firmware/radio/bind.h` | **新增** | ~55 行 | 简单 |
| 3 | `Firmware/radio/bind.c` | **新增** | ~280 行 | **核心/复杂** |
| 4 | `Firmware/radio/main.c` | **修改** | ~10 行 | 简单 |
| 5 | `Firmware/radio/tdm.c` | **修改** | ~25 行 | 中等 |
| 6 | `Firmware/radio/at.c` | **修改** | ~15 行 | 简单 |
| 7 | `Firmware/radio/parameters.h` | **修改** | ~5 行 | 简单 |
| 8 | `Firmware/radio/parameters.c` | **修改** | ~5 行 | 简单 |
| **合计** | | **2 新增 + 6 修改** | **~410 行** | |

---

## 四、逐文件详细修改说明

### 4.1 `Firmware/include/board_hm_trp.h` — 硬件引脚定义

**目的**：定义新增的对频按键引脚。

**修改位置与内容**：

```c
// ========== 修改 1：在 line 106 之后新增 GPIO 定义 ==========
// 原有代码（line 103-106）：
SBIT(LED_RED,      SFR_P1, 6);
SBIT(LED_GREEN,    SFR_P1, 5);
SBIT(PIN_CONFIG,   SFR_P0, 2);
SBIT(PIN_ENABLE,   SFR_P0, 3);

// 在其后新增：
SBIT(PIN_BIND,     SFR_P1, 7);    // 对频按键，P1.7 (Pin 8)，低电平有效
```

```c
// ========== 修改 2：在 line 117 之后新增 UI 定义 ==========
// 原有代码（line 114-117）：
#define LED_BOOTLOADER  LED_RED
#define LED_RADIO       LED_GREEN
#define LED_ACTIVITY    LED_RED
#define BUTTON_BOOTLOAD PIN_CONFIG

// 在其后新增：
#define BUTTON_BIND     PIN_BIND
```

```c
// ========== 修改 3：在 HW_INIT 宏 (line 124-138) 中追加初始化 ==========
// 在 P1SKIP |= 0x60; 之后，SFRPAGE = CONFIG_PAGE 之前，新增：
        P1SKIP  |=  0x80;       /* P1.7 跳过 Crossbar */  \
        P1MDIN  |=  0x80;       /* P1.7 数字输入模式 */   \
        P1MDOUT &= ~0x80;       /* P1.7 开漏（输入） */    \
        P1      |=  0x80;       /* P1.7 弱上拉使能 */      \
```

**预估修改**：~15 行

---

### 4.2 `Firmware/radio/bind.h` — 对频模块头文件 【新增】

**目的**：声明对频模块的公共接口。

```c
#ifndef _BIND_H_
#define _BIND_H_

#include "radio.h"
#include "parameters.h"

// 对频模式状态
enum BindState {
    BIND_STATE_IDLE,        // 正常运行
    BIND_STATE_SENDING,     // 主机（地面端）正在广播参数表
    BIND_STATE_LISTENING,   // 从机（天空端）正在监听
    BIND_STATE_SUCCESS,     // 对频成功
    BIND_STATE_TIMEOUT      // 对频超时
};

// 对频包结构
// 携带主机的全部射频参数，从机收到后整体覆盖
struct bind_packet {
    uint8_t  magic[2];                      // 魔术字 0xB1, 0xD0
    uint8_t  board_frequency;               // 板级频段标识（433/868/915等）
    param_t  params[PARAM_MAX];             // 完整参数表（与 parameter_values[] 一致）
    uint16_t crc;                           // CRC16 校验
};

#define BIND_MAGIC_0         0xB1
#define BIND_MAGIC_1         0xD0

// 对频时的公共射频配置（双方必须预先约定一致）
#define BIND_CHANNEL         0              // 固定信道编号
#define BIND_NETID           0xFFFF         // 公共 NETID（用于硬件包头过滤）
#define BIND_AIR_SPEED       2              // 最低空速率（2kbps），保证最大接收灵敏度
#define BIND_TIMEOUT_SEC     30             // 对频超时时间（秒）
#define BIND_TX_INTERVAL_MS  200            // 对频包发送间隔（毫秒）
#define BIND_LONG_PRESS_MS   3000           // 长按触发时间（毫秒）

// 公共接口
extern void    bind_init(void);
extern void    bind_check_button(void);
extern bool    bind_mode_active(void);
extern void    bind_tick(void);
extern void    bind_enter_from_at(void);

// 当前对频状态（外部可读）
extern __pdata enum BindState bind_state;

#endif // _BIND_H_
```

**预估行数**：~55 行

---

### 4.3 `Firmware/radio/bind.c` — 对频逻辑核心实现 【新增】

**目的**：按键检测、状态机、全参数对频包收发、参数保存与重启。

**核心逻辑伪代码**：

```
bind_check_button():
    读取 BUTTON_BIND 引脚
    ├── 按下 → 累计按压时间
    │   └── 持续时间 >= 3 秒 → 进入对频模式 (bind_enter())
    └── 松开 → 清零计时

bind_enter():
    记录当前的射频配置（频率、空速率、信道等）
    ────────────────────────────────────────────────
    【关键步骤】切换射频到"公共对频配置"：
    1. radio_set_network_id(BIND_NETID=0xFFFF)  // 公共包头
    2. radio_configure(BIND_AIR_SPEED=2kbps)     // 最低速率最大灵敏度
    3. radio_set_frequency(频段固定基频, 0)      // 零间隔，单信道
    4. radio_set_channel(BIND_CHANNEL=0)         // 固定信道 0
    5. 停止 TDM 跳频调度
    ────────────────────────────────────────────────
    根据 PARAM_BIND_ROLE 进入发送或监听状态

bind_tick() [在主循环中被调用]:
    if 发送模式 (主机/地面端):
        每 200ms 组装并广播 bind_packet:
          ┌─────────────────────────────────────────┐
          │ MAGIC(2B) | FREQ_ID(1B) | PARAMS(全部) | CRC(2B) │
          └─────────────────────────────────────────┘
          PARAMS = 主机当前的 parameter_values[] 完整拷贝
          包含: NETID, AIR_SPEED, MIN_FREQ, MAX_FREQ,
                NUM_CHANNELS, ECC, MANCHESTER, MAX_WINDOW,
                MAVLINK, DUTY_CYCLE, LBT_RSSI, TXPOWER 等全部参数
        LED 红绿交替快闪
        检查超时 → 超时则 bind_exit()

    if 监听模式 (从机/天空端):
        持续接收，等待 bind_packet
        收到后校验:
          1. 检查 MAGIC 魔术字
          2. 检查 board_frequency 与本机频段是否匹配
          3. 验证 CRC16
        校验通过 → 提取 params[] → 逐个 param_set() 覆盖本地参数
        → param_save() 写入 Flash
        → LED 绿灯常亮 2 秒表示成功
        → 软件复位 (RSTSRC |= (1 << 4))

bind_exit():
    恢复原有射频配置（频率、速率、信道等）
    重新初始化跳频和 TDM
    退出对频状态
```

**详细实现要点**：

| 函数名 | 行数 | 说明 |
|:---|:---:|:---|
| `bind_init()` | ~10 | 初始化状态变量和计时器 |
| `bind_check_button()` | ~35 | 按键去抖动 + 长按检测（使用 `timer2_tick()` 计时） |
| `bind_enter()` | ~55 | 保存当前射频配置，切换到公共信道/速率/NETID，停止 TDM |
| `bind_tick_send()` | ~45 | 将 `parameter_values[]` 整体打包为 `bind_packet`，调用 `radio_transmit()` 广播 |
| `bind_tick_listen()` | ~50 | 调用 `radio_receive_packet()` 接收，校验魔术字+频段+CRC，逐参数覆盖本地 |
| `bind_on_success()` | ~30 | 逐个 `param_set()` 覆盖全部参数 + `param_save()` + LED 反馈 + 软件复位 |
| `bind_exit()` | ~25 | 恢复原有射频参数，重新初始化 `radio_configure` + `fhop_init` + `tdm_init` |
| `bind_led_update()` | ~20 | 不同状态下的 LED 闪烁模式 |
| `bind_enter_from_at()` | ~10 | AT 指令入口，调用 `bind_enter()` |

**预估行数**：~280 行

---

### 4.4 `Firmware/radio/main.c` — 主初始化流程

**目的**：在 `main()` 函数中初始化对频模块。

**修改位置**：

```c
// ========== 修改 1：在文件顶部 include 区域添加 ==========
#include "bind.h"

// ========== 修改 2：在 main() 函数中, line 140（pins_user_init 之后）新增 ==========
    // Init bind button
    bind_init();
```

**修改行数**：~10 行

---

### 4.5 `Firmware/radio/tdm.c` — TDM 主循环集成

**目的**：在主循环 `tdm_serial_loop()` 中插入对频逻辑。这是集成的关键位置。

**修改位置与内容**：

```c
// ========== 修改 1：文件顶部添加 include ==========
#include "bind.h"

// ========== 修改 2：在 tdm_serial_loop() 的 for(;;) 循环开头 (line 522 之后) 插入 ==========
    // 对频按键检测（每次循环都扫描）
    bind_check_button();

    // 如果处于对频模式，跳过正常的 TDM 逻辑
    if (bind_mode_active()) {
        bind_tick();
        continue;
    }

// ========== 修改 3：在 link_update() 函数 (line 380) 开头添加 ==========
    if (bind_mode_active()) {
        return;  // 对频模式下 LED 由 bind 模块控制
    }
```

**修改行数**：~25 行

**关键设计说明**：
- 将 `bind_check_button()` 放在循环最开始，确保即使在各种 `continue` 分支跳过时，按键扫描始终执行
- 当 `bind_mode_active()` 返回 `true` 时，直接 `continue` 跳过整个 TDM 状态机（包括收包、发包、跳频等全部逻辑）
- 对频期间的收发完全由 `bind_tick()` 内部控制，使用固定信道、最低空速率和公共 NETID

---

### 4.6 `Firmware/radio/at.c` — AT 指令扩展

**目的**：增加通过串口指令进入/退出对频模式的能力（便于上位机软件控制）。

**修改位置**：

```c
// ========== 修改 1：文件顶部添加 include ==========
#include "bind.h"

// ========== 修改 2：在 at_command() 的 switch 语句 (line 234) 中添加新的 case ==========
// 在 case 'Z' 之前新增：
            case 'B':
                at_bind();      // ATB -> 进入对频模式
                break;

// ========== 修改 3：新增 at_bind() 函数 ==========
static void
at_bind(void)
{
    bind_enter_from_at();
    at_ok();
}
```

**修改行数**：~15 行

---

### 4.7 `Firmware/radio/parameters.h` — 参数枚举扩展

**目的**：新增 `PARAM_BIND_ROLE` 参数，用于区分主机/从机角色。

**修改位置**：

```c
// ========== 修改 1：在 enum ParamID (line 48-68) 中，PARAM_MAX 之前新增 ==========
    PARAM_RTSCTS,           // 原有 (line 63)
    PARAM_MAX_WINDOW,       // 原有 (line 64)
    PARAM_BIND_ROLE,        // 【新增】对频角色: 0=从机(监听), 1=主机(广播)
#ifdef INCLUDE_AES
    PARAM_ENCRYPTION,
#endif
    PARAM_MAX

// ========== 修改 2：更新参数格式版本号 (line 71) ==========
// 原有：
#define PARAM_FORMAT_CURRENT  0x1aUL
// 改为：
#define PARAM_FORMAT_CURRENT  0x1bUL    // 新增 PARAM_BIND_ROLE，版本号 +1
```

**修改行数**：~5 行

**重要说明**：修改 `PARAM_FORMAT_CURRENT` 后，已刷过旧版固件的设备在首次启动新固件时会检测到格式不匹配，自动执行 `param_default()` 回退到默认参数。这是预期行为，用户需要重新配置参数。

---

### 4.8 `Firmware/radio/parameters.c` — 参数默认值

**目的**：为新参数添加默认值和名称。

**修改位置**：

```c
// ========== 在 parameter_info 数组 (line 53-76) 中，对应位置新增 ==========
    {"MAX_WINDOW",    131},   // 原有 (line 72)
    {"BIND_ROLE",       0},   // 【新增】默认为从机（监听对频包）
#ifdef INCLUDE_AES
    {"ENCRYPTION_LEVEL", 0},  // 原有 (line 74)
#endif
```

**修改行数**：~5 行

---

## 五、对频包设计详解

### 5.1 为什么不能只发 NETID？

SiK 的通信链路建立依赖以下参数链条，**任何一个不匹配都会导致通信失败**：

```
参数匹配链条：
NETID ──→ 硬件包头过滤 (找到彼此)
  │
  ├─→ 跳频表种子 (跳到同一频率)
  ├─→ 频率偏移 (在正确的频率上)
  │
AIR_SPEED ──→ 调制解调速率 (解得了码)
MIN/MAX_FREQ ──→ 频率范围 (跳频不越界)
NUM_CHANNELS ──→ 信道总数 (跳频表长度一致)
ECC ──→ Golay 编/解码 (数据完整)
MANCHESTER ──→ 物理层编码 (波形正确)
MAX_WINDOW ──→ TDM 时隙 (时序对齐)
ENCRYPTION KEY ──→ 加解密 (内容可读)
```

### 5.2 对频包结构设计

采用**整表同步**策略，直接将主机的 `parameter_values[]` 完整发送：

```
┌──────────────────────────────────────────────────────────┐
│                     bind_packet 结构                      │
├────────┬──────────┬──────────────────────────┬───────────┤
│ MAGIC  │ FREQ_ID  │    parameter_values[]     │   CRC16   │
│ (2B)   │  (1B)    │  全部参数的完整拷贝        │   (2B)    │
│ 0xB1D0 │ 433/915  │  NETID + AIR_SPEED +      │  校验和   │
│        │  等      │  MIN/MAX_FREQ +           │           │
│        │          │  NUM_CHANNELS + ECC +      │           │
│        │          │  MANCHESTER + MAX_WINDOW + │           │
│        │          │  MAVLINK + DUTY_CYCLE +    │           │
│        │          │  LBT_RSSI + TXPOWER +      │           │
│        │          │  RTSCTS + BIND_ROLE +      │           │
│        │          │  (加密密钥 如有)           │           │
├────────┴──────────┴──────────────────────────┴───────────┤
│ 总大小: 2 + 1 + sizeof(parameter_values) + 2             │
│       ≈ 2 + 1 + (16×4) + 2 = 69 字节                    │
│ 远小于 MAX_PACKET_LENGTH (252 字节)，一包即可发完         │
└──────────────────────────────────────────────────────────┘
```

### 5.3 安全校验机制（三重校验）

从机收到包后，执行以下三重校验，全部通过才接受：

1. **魔术字校验**：`magic[0]==0xB1 && magic[1]==0xD0`，过滤掉非对频包
2. **频段校验**：`board_frequency == g_board_frequency`，防止 433MHz 设备误收 915MHz 的参数
3. **CRC16 校验**：对 magic + freq_id + params 整体计算 CRC16，防止数据损坏

---

## 六、对频流程示意图

```
 ┌─────────────────────────────────────────────────────────────┐
 │                      正常工作状态                            │
 │  地面端 ←────── TDM/FHSS 通信 ──────→ 天空端               │
 │  (全部参数一致)                       (全部参数一致)         │
 └───────────────┬──────────────────────────┬──────────────────┘
                 │                          │
           用户长按3秒                 用户长按3秒
           BIND 按键                  BIND 按键
                 │                          │
                 ▼                          ▼
 ┌───────────────────────────┐  ┌──────────────────────────────┐
 │   主机 进入对频模式         │  │   从机 进入对频模式           │
 │                            │  │                              │
 │  1. 停止 TDM/FHSS          │  │  1. 停止 TDM/FHSS            │
 │  2. radio_configure(2kbps) │  │  2. radio_configure(2kbps)   │
 │  3. radio_set_freq(固定)   │  │  3. radio_set_freq(固定)     │
 │  4. radio_set_channel(0)   │  │  4. radio_set_channel(0)     │
 │  5. radio_set_netid(0xFFFF)│  │  5. radio_set_netid(0xFFFF)  │
 │  6. LED 红绿交替闪         │  │  6. LED 红绿交替闪            │
 └───────────┬───────────────┘  └──────────────┬───────────────┘
             │                                 │
             │  每200ms广播 bind_packet          │
             │  ┌─────────────────────────┐     │
             │  │ MAGIC | FREQ_ID |       │     │
             │  │ parameter_values[全部]  │     │
             │  │ (NETID + AIR_SPEED +    │     │
             │  │  MIN/MAX_FREQ +         │     │
             │  │  NUM_CHANNELS + ECC +   │     │
             │  │  MANCHESTER + ...)      │     │
             ├──┤ CRC16                   ├────►│
             │  └─────────────────────────┘     │
             │                                 │
             │                    ┌─────────────┴──────────────┐
             │                    │ 三重校验:                    │
             │                    │  ✓ MAGIC == 0xB1D0          │
             │                    │  ✓ FREQ_ID == 本机频段       │
             │                    │  ✓ CRC16 校验通过            │
             │                    ├─────────────────────────────┤
             │                    │ 全部通过:                    │
             │                    │  for(i=1; i<PARAM_MAX; i++) │
             │                    │    param_set(i, params[i]); │
             │                    │  param_save();              │
             │                    │  LED 绿灯常亮 2 秒          │
             │                    │  软件复位 → 重新初始化       │
             │                    └─────────────────────────────┘
             │
             ▼ (30秒超时自动退出)
 ┌───────────────────────────┐
 │  恢复原有射频配置           │
 │  重新初始化 FHSS + TDM     │
 │  回到正常通信               │
 └───────────────────────────┘
```

---

## 七、对频时的公共射频配置约定

对频的前提是：双方在按下按键后，必须在**完全相同的射频配置**下才能互相"听到"。
因此需要预先在代码中硬编码一组**公共对频参数**：

| 项目 | 公共对频值 | 选择理由 |
|:---|:---:|:---|
| NETID | `0xFFFF` | 作为公共广播 ID，所有设备统一 |
| 空速率 | `2 kbps` | 最低速率 → 最高接收灵敏度 → 最大对频距离 |
| 信道号 | `0` | 固定单信道，不跳频 |
| 频率 | 频段固定基频 | 取决于 `g_board_frequency`（如 915MHz 取 915.000MHz）|
| ECC | 关闭 | 简化对频包处理 |
| 曼彻斯特 | 关闭 | 简化对频包处理 |

---

## 八、硬件设计配合要求

| 项目 | 规格 |
|:---|:---|
| 按键引脚 | **P1.7 (Pin 8)**（Si1000 42-pin LGA 封装中 P1 端口唯一空闲且引出到外部的引脚）|
| 按键类型 | 常开轻触开关，一端接 GND，一端接 P1.7 |
| 硬件消抖 | 在 P1.7 与 GND 之间并联 100nF (104) 陶瓷电容 |
| 外部上拉 | 可选：P1.7 到 VDD_MCU 串联 10kΩ（MCU 内部有弱上拉，但外部加更稳定）|
| LED | 复用现有的红色 (P1.6) 和绿色 (P1.5) LED，无需新增 |

**推荐原理图接法**：
```
VDD_MCU (3.3V)
    │
   [10kΩ] (可选外部上拉)
    │
    ├──── P1.7 / Pin 8 (MCU)
    │
   [104 电容] (100nF 硬件消抖)
    │
   [按键] (常开轻触开关)
    │
   GND
```

---

## 九、资源开销评估

| 资源 | 预估增量 | 原系统总量 | 占比 |
|:---|:---:|:---:|:---:|
| Flash (代码空间) | ~1200-1600 字节 | 64KB | ~2.3% |
| XDATA RAM | ~80 字节（bind_packet + 状态变量）| 4352 字节 | ~1.8% |
| 新增 GPIO 引脚 | 1 个 (P1.7, Pin 8) | — | — |
| 新增参数 | 1 个 (PARAM_BIND_ROLE) | 15 个 | +1 |

**结论**：资源开销较小，不会影响现有功能。对频包约 69 字节，远小于 MAX_PACKET_LENGTH (252 字节)，一个射频包即可发完全部参数。

---

## 十、风险与注意事项

| 风险项 | 等级 | 应对策略 |
|:---|:---:|:---|
| 8051 Flash 空间不足 | 低 | 原固件约用 50-60%，还有充足余量。可通过 `--codeseg` bank 分配优化 |
| 对频期间看门狗触发 | 中 | `bind_tick()` 循环中必须周期性喂狗或禁用看门狗 |
| 不同频段设备误对频 | 低 | 对频包中携带 `board_frequency` 字段，从机收到后与本机频段比对，不匹配则丢弃 |
| 两台设备同时广播互扰 | 低 | 在发送间隔中加入随机抖动（jitter），降低碰撞概率 |
| 对频后原有连接断开 | 预期 | 对频成功后执行软件复位，MCU 重新初始化所有子系统 |
| `PARAM_FORMAT` 版本兼容 | 中 | 新增参数后将 `PARAM_FORMAT_CURRENT` 从 `0x1a` 更新为 `0x1b`，旧设备升级后首次启动会回退到默认参数 |
| 参数合法性校验 | 中 | 从机在覆盖参数前，对每个参数调用 `param_check()` 验证合法性，非法值用默认值替代 |
| 加密密钥同步 | 中 | 如启用 AES，需在 `bind_packet` 中额外携带 32 字节加密密钥，或在对频时禁用加密 |

---

## 十一、开发优先级建议

| 阶段 | 任务 | 预计工时 |
|:---|:---|:---:|
| **Phase 1** | 硬件：PCB 上增加 BIND 按键（P1.7, Pin 8），打样验证 | 1 天 |
| **Phase 2** | 软件：实现 `bind.c/h`，按键检测 + LED 反馈（不含射频） | 2 天 |
| **Phase 3** | 软件：实现公共信道切换 + 全参数对频包的发送/接收/校验逻辑 | 3 天 |
| **Phase 4** | 集成：修改 `tdm.c`/`main.c`/`at.c`/`parameters.*`，联调 | 1-2 天 |
| **Phase 5** | 测试：双机对频、不同频段互斥测试、参数一致性验证、压力测试 | 2-3 天 |
| **合计** | | **9-11 天** |

---

## 十二、后续可扩展方向

1. **AT 指令对频**：已预留 `ATB` 指令，上位机（如 Mission Planner）可通过串口触发对频，无需物理按键
2. **Binding Phrase（绑定短语）**：类似 ELRS，用户通过 AT 指令设置一个字符串（如 `AT&B=myDrone01`），该字符串的哈希值作为对频包的额外认证因子，防止被第三方设备劫持
3. **选择性参数同步**：在对频包中增加位掩码字段，允许用户选择只同步部分参数（如只同步 NETID 和频率，保留本地的串口波特率设置）
4. **多对一对频**：支持一个地面站同时对频多个天空端（中继/组网场景）
5. **OTA 对频**：通过串口连接的 ESP8266/ESP32 实现 WiFi/蓝牙无线配置（类似 ELRS WiFi 模式）
