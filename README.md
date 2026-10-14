# 2026年电子设计大赛 H 题 — 车载平衡滚球运动控制系统

> **赛题**：2026年全国大学生电子设计竞赛黑龙江赛区 H 题  
> **题目**：车载平衡滚球运动控制系统  
> **奖项**：省级一等奖

## 一、系统概述

本系统是一个**车载平衡滚球运动控制系统**，由三块 MCU 协同工作完成以下任务：

1. **视觉感知**：摄像头实时检测钢球在摆杆凹槽中的位置，经卡尔曼滤波预测后输出坐标与速度；
2. **平衡控制**：根据视觉偏移量驱动步进电机推动摆杆，使钢球保持居中；
3. **运动平台**：搭载平衡装置的智能小车沿黑色引导线自动循迹行驶，具备速度闭环与加减速控制。

```
┌──────────────────┐  UART ASCII   ┌──────────────────┐  步进电机  ┌────────┐
│  Q_2026_JW       │──────────────▶│ circle_data_0801 │───────────▶│ 钢球摆杆 │
│  K230 视觉检测    │ origin:()v:() │ STM32F103 平衡PID │            │        │
│  YOLO+卡尔曼滤波  │   100Hz       │ Emm V5步进驱动    │            └────────┘
└──────────────────┘               └──────────────────┘
                                          │ 安装于车上
                                          ▼
                                   ┌──────────────────┐
                                   │ eightwaysxunjij  │
                                   │ STM32F407 循迹小车│
                                   │ 8路灰度+编码器+IMU│
                                   └──────────────────┘
```

## 二、项目结构

```
26电赛项目/
├── Q_2026_JW/                  # K230 视觉检测端
├── circle_data_backup_0801/    # STM32F103 平衡控制端
├── eightwaysxunjij/            # STM32F407 循迹小车主控
└── README.md                   # 本文件
```

### 2.1 Q_2026_JW — 视觉感知端

| 属性 | 说明 |
|------|------|
| **MCU** | 亚博智能 Kendryte K230（KPU 神经网络加速） |
| **语言** | MicroPython（CanMV / CanMicro） |
| **摄像头** | CSI 双通道（Ch0 YUV420 640×480 显示 / Ch2 RGB888P 推理） |
| **AI 模型** | AnchorBaseDet 单类检测器（ball），320×320 输入，uint8 量化 |
| **通信** | UART1 @ 115200bps，100Hz 输出 |
| **开发环境** | PyCharm 远程开发 → 烧录至 SD 卡 |

**核心架构**：

```
摄像头采集 → ai2d预处理(缩放/填充至320×320) → KPU推理(.kmodel)
    → aicube后处理(NMS) → 选最高置信度 ball 框
    → 四级信号滤波链：
        ① HysteresisStateFilter（3态FSM：LOST/STATIC/MOVING）
        ② VelocityTracker（滚动窗口速度估计，EMA α=0.65）
        ③ KalmanPredictor（2态恒速卡尔曼，位置+速度）
        ④ PredictionMailbox（线程安全锁保护）
    → UART线程 100Hz 输出 ASCII: origin:(偏移) v:(速度)
```

**双线程设计**：YOLO 线程（25ms 周期）+ UART 线程（10ms 周期），通过 `PredictionMailbox` 锁保护共享状态。

**关键文件**：

| 文件 | 说明 |
|------|------|
| `main.py` | 最终部署固件（~760行） |
| `det_video_1_3.py` | 早期原型（简化版，无滤波/无卡尔曼） |
| `mp_deployment_source/` | 模型部署目录（kmodel + 配置） |
| `数据包.md` | 二进制通信协议规格文档 |

### 2.2 circle_data_backup_0801 — 平衡控制端

| 属性 | 说明 |
|------|------|
| **MCU** | STM32F103C8T6（Cortex-M3, 8MHz HSI） |
| **语言** | C（STM32 HAL 库） |
| **通信** | USART3 接收 K230 数据（DMA 环形缓冲区） |
| **执行器** | Emm V5.0 步进电机驱动器（USART2，自定义帧协议） |
| **算法** | 位置式 PID（目标=居中，误差=球的横向偏移） |
| **开发环境** | Keil MDK-ARM + STM32CubeMX |

**核心逻辑**：

```
K230 → USART3 → DMA环形缓冲 → 解析 "origin:(偏移) v:(速度)"
                                    │
                           g_latest_error_x (volatile)
                                    │
                     Ball_Control_Once() → PID_Pro()
                           Kp=0.7, Ki=0.054, Kd=0.5
                           限幅 ±2000 脉冲
                                    │
                     Emm V5 位置模式 0xFD → 步进电机推动摆杆
```

**关键文件**：

| 文件 | 说明 |
|------|------|
| `user/pid.c` | 位置式 PID + Ball_Control_Once() 节流控制 |
| `user/usart_receive.c` | DMA 环形缓冲解析 ASCII 帧（快速整数解析） |
| `user/step_motor.c` | Emm V5.0 驱动器协议（位置模式 0xFD / 速度模式 0xF6） |
| `user/step_motor_pid.c` | 备用 PID（当前未启用） |

### 2.3 eightwaysxunjij — 循迹小车主控

| 属性 | 说明 |
|------|------|
| **MCU** | STM32F407ZGT6（Cortex-M4, 168MHz） |
| **语言** | C（STM32 HAL 库） |
| **传感器** | 8路红外灰度模块、JY901S IMU、×2 编码器电机 |
| **执行器** | TB6612 双路电机驱动、I²C OLED |
| **算法** | 双环 PID（循迹 PD + 速度 PID）融合、S 曲线加减速 |
| **开发环境** | STM32CubeMX + Keil MDK + CMake/GCC 双工具链 |

**核心架构**：

```
红外灰度 ──→ track_line()查表法 ──→ Yaw_measure ──→ yaw_PD()差速
                                                    │
编码器 ──→ SpeedPID增量式 ──→ PID输出 ──→ 双环融合 ─┘
                                                    │
                                           S曲线加减速状态机
                                           (4段加速/3段减速)
                                                    │
                                           pwm_left/right → TB6612
```

**关键文件**：

| 文件 | 说明 |
|------|------|
| `user/PID.c` | 查表法循迹 + 位置式 PD 控制器 |
| `user/tb6612.c` | 电机驱动 + S 曲线加减速状态机 + 按键扫描 |
| `user/jy_read.c` / `jy_turn.c` | JY901S IMU 数据读取与航向累积 |
| `user/gwIIC.c` | 8路灰度传感器（4种接口协议兼容，当前为 GPIO 直读） |
| `user/OLED.c` | OLED 显示驱动 |

## 三、通信协议

### 视觉端 → 平衡端（ASCII 格式，实际使用）

```
origin:(<signed_offset>) v:(<signed_velocity>)\r\n
```

- **offset**：球中心相对图像中线（320px）的像素偏移，左负右正
- **velocity**：卡尔曼滤波后的像素速度（有符号）
- **999**：检测失败标记（平衡端保持不动）
- **波特率**：115200，发送频率：100Hz

### 二进制协议格式（`数据包.md` 规格）

每包固定 **16 字节（8× uint16，大端序）**：

| 序号 | 字段 | 含义 |
|------|------|------|
| 0 | `head0` | `0x5AA5` 帧头 |
| 1 | `head1` | `0xA55A` 帧头 |
| 2 | `reserved` | `0x0002`=偏右 / `0x0001`=偏左 / `0x0000`=检测失败 |
| 3 | `position` | Kalman 预测的球绝对横坐标 (0~640px) |
| 4 | `velocity` | Kalman 预测的无符号像素速度 |
| 5 | `parity` | 校验和（前5字大端10字节之和，取低16位） |
| 6 | `tail0` | `0x0D0A` 帧尾 |
| 7 | `tail1` | `0x55AA` 帧尾 |

无有效检测时：`reserved=0x0000`，`position=velocity=0x7FFF`。

## 四、硬件清单

| 模块 | 型号/规格 | 用途 |
|------|-----------|------|
| AI 摄像头 | 亚博智能 K230 | 视觉检测 |
| 主控小车 | STM32F407ZGT6 | 循迹行驶 |
| 平衡控制器 | STM32F103C8T6 | 球平衡 PID |
| 步进驱动 | Emm V5.0 | 驱动摆杆电机 |
| 电机驱动 | TB6612 | 小车直流电机 |
| 灰度传感器 | 8路红外灰度模块 | 循迹检测 |
| IMU | JY901S | 航向监测 |
| 显示屏 | I²C OLED | 状态显示 |
| 显示面板 | ST7701 LCD 640×480 | 视觉端调试显示 |

## 五、开发环境汇总

| 项目 | IDE/工具链 | 语言 | 构建方式 |
|------|-----------|------|----------|
| Q_2026_JW | PyCharm + CanMV | MicroPython | SD 卡烧录 |
| circle_data_0801 | Keil MDK-ARM | C (HAL) | Keil 编译 |
| eightwaysxunjij | Keil MDK / CMake+GCC | C (HAL) | Keil 或 Make |

## 六、作者

- **学校**：佳木斯大学
- **赛事**：2026年全国大学生电子设计竞赛黑龙江赛区
- **奖项**：省级一等奖
- **成员**: 陈佳伟 冯蕾 王兆金