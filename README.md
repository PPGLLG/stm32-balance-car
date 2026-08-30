# STM32F103C8T6 FreeRTOS 自平衡车

基于 **STM32F103C8T6 + FreeRTOS + HAL 库** 的两轮自平衡小车。  
独立完成硬件焊接、驱动开发、多任务软件架构、控制算法调参与整机联调。

## 演示视频



<video src="https://github.com/user-attachments/assets/ec2cabab-cde4-42ce-8eea-49742d13ef7c" controls="controls" style="max-width: 100%;"></video>



## 项目亮点

- 实现稳定自平衡，采用 **角度内环 PD + 速度外环 PI + 转向前馈** 的串级控制结构
- 使用 **FreeRTOS** 搭建多任务系统，完成控制、遥控、显示任务的划分与同步
- 支持 **红外遥控** 与 **WiFi 远程遥控/在线调参**
- 支持 **OLED 参数显示** 与 **Vofa+ 波形调试**
- 独立解决多个实际工程问题：硬件 I2C 花屏、OLED 抢占总线导致控制卡顿、WiFi AT 响应污染命令、红外长按不连续等

## 技术栈

| 分类 | 内容 |
|---|---|
| 主控 | STM32F103C8T6（Cortex-M3，72MHz） |
| 系统 | FreeRTOS（CMSIS-RTOS V2） |
| 语言 | C |
| 传感器 | MPU6050 六轴陀螺仪/加速度计 |
| 执行机构 | MG310 电机 ×2 + TB6612FNG 驱动 |
| 通信 | 红外 NEC、ESP-01S WiFi、USART、软件 I2C |
| 显示 | SSD1306 OLED（0.96 寸） |
| 调试 | Vofa+、Keil MDK、VS Code + GCC Makefile |

## 系统架构

```
Core/          CubeMX 生成：HAL、时钟、FreeRTOS 初始化、中断
BSP/           板级驱动：电机、MPU6050、OLED、红外、WiFi、软件 I2C、ADC
APP/           应用层：滤波、串级 PID、命令翻译、环形缓冲
Task/          FreeRTOS 任务：控制、遥控、显示、测试
Tools/         GCC 支持文件
MDK-ARM/       Keil 工程
```

### 任务划分

| 任务 | 优先级 | 周期 | 职责 |
|---|---|---|---|
| control_task | 最高 | 5ms | 读 MPU → 滤波 → 串级 PID → 输出 PWM |
| remote_task | 中 | 事件/100ms 超时 | 红外/WiFi 解码 → 写邮箱 |
| ui_task | 最低 | 250ms/页 | OLED 参数显示 |

## 控制算法

### 控制结构

```text
速度目标 → 速度斜坡 → 速度环 PI → 角度目标修正
角度目标 + 速度修正 → 角度环 PD → 共同 PWM
转向目标 → 转向斜坡 → 转向前馈 → 差速 PWM
共同 PWM + 差速 PWM → 左右轮输出
```

- 角度内环：5ms
- 速度外环：20ms
- 转向环：5ms（前馈无反馈）
- 输出限幅：±0.8
- 死区补偿：1%

### 滤波

- 使用**互补滤波**融合加速度计与陀螺仪数据
- 加速度计提供低频角度基准，陀螺仪提供高频动态角度
- 开机静止校准陀螺零偏

### 安全机制

- 跌倒锁存与恢复
- 失联保护（500ms 无命令自动清零速度/转向）
- 目标速度/转向斜坡
- 输出限幅
- 死区补偿

## 通信与遥控

- **红外遥控**：NEC 协议，支持完整帧与重复帧解码
- **WiFi 遥控**：ESP-01S AP + TCP Server，支持运动命令与实时 PID 调参
- **调试串口**：USART1，Vofa+ FireWater 协议波形输出

### WiFi 命令

| 命令 | 功能 |
|---|---|
| `S<数值>` | 设置目标速度 |
| `T<数值>` | 设置目标转向 |
| `A<数值>` | 设置平衡角 |
| `Z` | 全停 |
| `ap<数值>` / `ad<数值>` | 角度环 P / D |
| `sp<数值>` / `si<数值>` | 速度环 P / I |
| `tp<数值>` / `td<数值>` | 转向环 P / D |

## 解决过的问题

| 问题 | 分析与解决 |
|---|---|
| STM32F1 硬件 I2C 长数据传输花屏 | 改为软件模拟 I2C，波形稳定，彻底解决 |
| OLED 刷新导致控制任务卡顿 | OLED 分页分散刷新 + 单次小数据块传输 + MPU 读取超时 |
| WiFi 命令被 ESP AT 响应覆盖 | 关闭 ESP 回显（ATE0）+ 命令必须后跟数字的严格校验 |
| 红外长按遥控不连续 | 支持 NEC 重复帧识别，并持续刷新时间戳 |
| 速度环震荡调不好 | 速度外环与角度内环分离，采用不同控制周期 |
| 编码器始终为 0 | 启动编码器定时器计数（HAL_TIM_Encoder_Start） |
| 电机只能正转无法反转 | 排查反相器、排母虚焊，修复硬件连接 |

## 构建与烧录

### GCC / VS Code

```bash
make
make flash-openocd
```

### Keil

打开 `MDK-ARM/aaa.uvprojx` 编译烧录。

## 目录结构

```
.
├── code/                  # 固件工程
│   ├── Core/              # HAL/FreeRTOS 初始化
│   ├── BSP/               # 板级驱动
│   ├── APP/               # 滤波、PID、协议解析
│   ├── Task/              # FreeRTOS 任务
│   ├── Drivers/           # ST 官方 HAL/CMSIS
│   ├── Middlewares/       # FreeRTOS 内核
│   ├── MDK-ARM/           # Keil 工程
│   └── Makefile           # GCC 构建
├── Gerber_PCB1_2026-07-16.zip  # PCB 制造文件
├── demo.mp4               # 演示视频
└── README.md
```
