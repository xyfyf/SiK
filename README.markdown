# SiK — 一键对频版本

本仓库基于 [ArduPilot/SiK](https://github.com/ArduPilot/SiK) 修改，在原有 SiK 无线电固件基础上新增了**一键对频（Bind Button）功能**，使两台 HM-TRP 模块无需上位机即可完成参数同步，实现类似 ELRS 的物理按键对频体验。

---

## 新增功能：一键对频

### 工作原理

两台模块同时长按对频按键（≥3 秒），主机广播完整参数表，从机接收并覆盖本机参数后自动复位，对频完成。

| 角色 | 行为 |
|------|------|
| 主机（`ATS16=1`）| 广播本机完整参数（NETID、AIR_SPEED、频率范围等） |
| 从机（`ATS16=0`，默认）| 监听主机广播，验证通过后覆盖本机参数并复位 |

### LED 状态指示

| 状态 | LED 表现 |
|------|----------|
| 对频中 | 红绿交替快闪（200ms/次） |
| 对频成功 | 绿灯常亮 1 秒，然后自动复位 |
| 对频超时（30秒） | 自动退出，恢复正常工作 |

### 新增 AT 指令

| 指令 | 功能 |
|------|------|
| `ATB` | 通过串口触发对频（等效于长按按键） |
| `ATS16=0` | 设置为从机角色（默认，接收参数） |
| `ATS16=1` | 设置为主机角色（发送参数） |

---

## 硬件要求（HM-TRP 板）

在 **P1.7（芯片第 8 引脚）** 增加一个对频按键，推荐外围电路：

```
VDD_3V3
    │
   10kΩ（上拉）
    │
    ├──── 100nF（到 GND，硬件去抖）
    │
P1.7（第8引脚）
    │
  按键（常开）
    │
   GND
```

> 注意：Si1000 LGA-42 封装中 P1.0~P1.4 为内部引脚，**无法引出到 PCB**，请勿使用。P1.7 已通过代码和数据手册双重验证可用。

---

## 代码修改清单

| 文件 | 修改内容 |
|------|----------|
| `Firmware/include/board_hm_trp.h` | 定义 `PIN_BIND`（P1.7）、`BUTTON_BIND`，配置引脚为数字输入+内部上拉 |
| `Firmware/radio/bind.h` | 新增：对频状态枚举、对频包结构体、接口函数声明，无对频按键时退化为空宏 |
| `Firmware/radio/bind.c` | 新增：完整对频状态机实现（按键检测、射频切换、参数广播/接收、CRC 校验、软件复位），仅在定义 `BUTTON_BIND` 时编译 |
| `Firmware/radio/parameters.h` | 新增参数 `PARAM_BIND_ROLE`，添加防重复包含宏 |
| `Firmware/radio/parameters.c` | 新增 `BIND_ROLE` 参数默认值及合法性校验 |
| `Firmware/radio/main.c` | 调用 `bind_init()` 初始化对频模块 |
| `Firmware/radio/tdm.c` | 在 TDM 主循环中插入按键扫描与对频状态机接管逻辑 |
| `Firmware/radio/at.c` | 新增 `ATB` 指令，触发对频模式 |

---

## 内存优化说明

- 对频包缓冲区复用射频驱动已有的 `radio_buffer`（252字节 XDATA），不额外占用内存。
- 无对频按键的板子（如 3dr1060/Si1060）通过 `#ifdef BUTTON_BIND` 完全跳过对频代码，XDATA 占用为零，解决了 Si1060 XDATA 溢出问题。

---

## 编译方法

需要 Linux / WSL 环境：

```bash
# 安装依赖
sudo apt install sdcc

# 编译所有板子
cd Firmware
make

# 编译并输出到 dst/ 目录
make install
```

固件输出路径：
- 主程序：`Firmware/dst/radio~hm_trp.ihx`
- Bootloader：`Firmware/dst/bootloader~hm_trp.ihx`

---

## 烧录方法

使用 SiLabs USB 调试适配器烧录 Bootloader（仅首次），之后可通过串口工具更新主程序固件：

```bash
# 命令行方式
python Firmware/tools/uploader.py --port /dev/ttyUSB0 Firmware/dst/radio~hm_trp.ihx
```

---

## 原始项目

- 上游仓库：[ArduPilot/SiK](https://github.com/ArduPilot/SiK)
- 芯片文档：[Si1000 Datasheet](https://www.silabs.com/documents/public/data-sheets/Si1000.pdf)
