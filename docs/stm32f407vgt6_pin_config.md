# STM32F407VGT6 核心板引脚配置文档

版本：v0.1  
目标芯片：STM32F407VGT6，LQFP100，3.3V 逻辑  
目标板卡：两轮轮足机器人小脑核心板

## 1. 设计依据

本引脚方案基于以下资料和当前机器人硬件需求整理：

- ST 官方 `STM32F405xx/STM32F407xx Datasheet DS8626 Rev 12`：确认 STM32F407 系列具备 168MHz Cortex-M4F、最高 1MB Flash、192+4KB SRAM、USB OTG FS/HS、CAN、USART/UART、SPI、I2C、3 路 ADC、定时器、SWD/JTAG 等资源；文档中的 `Figure 13. STM32F40x LQFP100 pinout`、`Table 7. STM32F40xxx pin and ball definitions`、`Table 9. Alternate function mapping` 是本表物理脚位和复用功能的主要依据。  
  资料链接：https://www.st.com/resource/en/datasheet/dm00037051.pdf
- ST 官方 `AN4488 Getting started with STM32F4xxxx MCU hardware development`：用于最小系统、电源、复位、BOOT、调试接口等硬件设计约束。  
  资料链接：https://www.st.com/resource/en/application_note/an4488-getting-started-with-stm32f4xxxx-mcu-hardware-development-stmicroelectronics.pdf
- Espressif 官方 `ESP32-C3-WROOM-02/02U Datasheet`：确认模块 3.0V-3.6V 供电、UART0、USB Serial/JTAG、GPIO、EN、strapping pins 等接口。  
  资料链接：https://documentation.espressif.com/esp32-c3-wroom-02_datasheet_en.html
- Espressif 官方 `ESP32-C3 Hardware Design Guidelines`：用于 ESP32-C3 天线禁布区、USB 走线、供电去耦等布局约束。  
  资料链接：https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32c3/index.html
- TDK InvenSense 官方 MPU-6050 页面：MPU-6050 为 I2C 输出 6 轴 IMU，支持 400kHz Fast Mode I2C；同时该器件已 EOL，建议预留新 IMU 兼容位。  
  资料链接：https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6000/
- Solomon Systech 官方 SSD1306 页面：SSD1306 OLED 支持 I2C/SPI/并口，本板按 I2C 小屏方案分配。  
  资料链接：https://www.solomon-systech.com/zh-hans/product/ssd1306/

## 2. 总体分配原则

1. 固定系统脚优先：`PH0/PH1` 给 HSE，`PC14/PC15` 预留 LSE，`PA13/PA14` 固定 SWD，`PA11/PA12` 固定 USB OTG FS。
2. 电机控制优先使用高级定时器：左/右两组无刷电机输入分别使用 `TIM1_CH1~3` 与 `TIM8_CH1~3`。
3. 4 路舵机独立使用 `TIM4_CH1~4`，避免占用编码器定时器。
4. 编码器使用 `TIM5` 和 `TIM3` 的 Encoder Mode，避免和电机 PWM、舵机 PWM 冲突。
5. ADC 集中放在 `PC0~PC5`、`PA4/PA5`、`PB0/PB1`，方便电流、电压、温度采样走线。
6. 大脑通信主链路使用 USB-C + USB CDC；`USART2` 作为调试串口/大脑串口备用链路。
7. ESP32-C3 作为冗余无线调参模块，通过 `USART3` 与 STM32 通信。
8. CAN、RS485、SPI2、I2C2 尽量预留，方便后续扩展传感器、执行器和调试设备。

## 3. STM32 最小系统与固定引脚

| 功能 | STM32 引脚 | LQFP100 脚号 | 方向 | 配置 | 说明 |
|---|---:|---:|---|---|---|
| HSE_IN | PH0 | 12 | 输入 | OSC_IN | 外接 8MHz 或 12MHz 晶振输入，推荐 8MHz |
| HSE_OUT | PH1 | 13 | 输出 | OSC_OUT | 外接晶振输出 |
| LSE_IN | PC14 | 8 | 输入 | OSC32_IN | 可选 32.768kHz RTC 晶振，不用时不要作为大电流 IO |
| LSE_OUT | PC15 | 9 | 输出 | OSC32_OUT | 可选 RTC 晶振 |
| NRST | NRST | 14 | 输入 | Reset | 接复位按键、SWD 接口、上拉/RC 复位网络 |
| BOOT0 | BOOT0 | 94 | 输入 | Boot strap | 10k 下拉到 GND，预留测试点或跳帽 |
| SWDIO | PA13 | 72 | 双向 | AF0 SYS | SWD 下载/调试必留 |
| SWCLK | PA14 | 76 | 输入 | AF0 SYS | SWD 下载/调试必留 |
| SWO | PB3 | 89 | 输出 | AF0 SYS | 可选 SWO/ITM 调试输出，建议接到 SWD 接口 |
| VBAT | VBAT | 6 | 电源 | RTC backup | 可接纽扣电池或直接接 3.3V |
| VCAP_1 | VCAP_1 | 49 | 电源 | Core regulator | 按 ST 手册放置低 ESR 电容，靠近芯片 |
| VCAP_2 | VCAP_2 | 73 | 电源 | Core regulator | 按 ST 手册放置低 ESR 电容，靠近芯片 |

## 4. 电机驱动板接口

当前根据电机驱动板图片推断其需要 2 组三相控制输入、6 路电流/采样反馈、使能和电池电压反馈。以下分配适合后续做三相 PWM 或类似三输入控制。

| 外部信号 | STM32 引脚 | LQFP100 脚号 | 外设/复用 | 方向 | 建议连接 |
|---|---:|---:|---|---|---|
| M0_IN1 | PE9 | 40 | TIM1_CH1 / AF1 | 输出 | 电机 0 第 1 路 PWM/控制输入，串 22R-100R |
| M0_IN2 | PE11 | 42 | TIM1_CH2 / AF1 | 输出 | 电机 0 第 2 路 PWM/控制输入，串 22R-100R |
| M0_IN3 | PE13 | 44 | TIM1_CH3 / AF1 | 输出 | 电机 0 第 3 路 PWM/控制输入，串 22R-100R |
| M1_IN1 | PC6 | 63 | TIM8_CH1 / AF3 | 输出 | 电机 1 第 1 路 PWM/控制输入，串 22R-100R |
| M1_IN2 | PC7 | 64 | TIM8_CH2 / AF3 | 输出 | 电机 1 第 2 路 PWM/控制输入，串 22R-100R |
| M1_IN3 | PC8 | 65 | TIM8_CH3 / AF3 | 输出 | 电机 1 第 3 路 PWM/控制输入，串 22R-100R |
| M_EN | PE15 | 46 | GPIO Output | 输出 | 电机总使能，硬件默认下拉为关闭 |
| MDRV_FAULT_N | PE14 | 45 | GPIO/EXTI | 输入 | 如果驱动板有故障脚则接入；没有则预留 |
| M0_CS1 | PC0 | 15 | ADC123_IN10 | 模拟输入 | 电机 0 电流/采样反馈 1，前端加 RC 和钳位 |
| M0_CS2 | PC1 | 16 | ADC123_IN11 | 模拟输入 | 电机 0 电流/采样反馈 2 |
| M0_CS3 | PC2 | 17 | ADC123_IN12 | 模拟输入 | 电机 0 电流/采样反馈 3 |
| M1_CS1 | PC3 | 18 | ADC123_IN13 | 模拟输入 | 电机 1 电流/采样反馈 1 |
| M1_CS2 | PA4 | 29 | ADC12_IN4 | 模拟输入 | 电机 1 电流/采样反馈 2 |
| M1_CS3 | PA5 | 30 | ADC12_IN5 | 模拟输入 | 电机 1 电流/采样反馈 3 |
| BMEA / VMOT_SENSE | PC4 | 33 | ADC12_IN14 | 模拟输入 | 电机母线/电池电压采样，必须经分压 |
| MDRV_5V_SENSE | PC5 | 34 | ADC12_IN15 | 模拟输入 | 监测驱动板提供给核心板的 5V |

待实物确认项：

- `M_EN` 有效电平。
- `IN1/IN2/IN3` 是 3.3V 兼容输入还是需要 5V 电平。
- `CS` 输出是电流模拟电压、相电流采样，还是其他状态量。
- `BMEA` 的分压比例和最大电压。
- 驱动板 `5V` 引脚最大可输出电流。

## 5. 轮端编码器 / 霍尔接口

| 外部信号 | STM32 引脚 | LQFP100 脚号 | 外设/复用 | 方向 | 说明 |
|---|---:|---:|---|---|---|
| ENC_L_A | PA0 | 23 | TIM5_CH1 / AF2 | 输入 | 左轮编码器 A 相 |
| ENC_L_B | PA1 | 24 | TIM5_CH2 / AF2 | 输入 | 左轮编码器 B 相 |
| ENC_L_Z | PE2 | 1 | GPIO/EXTI | 输入 | 左轮 Index，可选 |
| ENC_R_A | PA6 | 31 | TIM3_CH1 / AF2 | 输入 | 右轮编码器 A 相 |
| ENC_R_B | PA7 | 32 | TIM3_CH2 / AF2 | 输入 | 右轮编码器 B 相 |
| ENC_R_Z | PE3 | 2 | GPIO/EXTI | 输入 | 右轮 Index，可选 |

建议：

- 编码器供电独立标明 `3V3_ENC` 或 `5V_ENC`，5V 编码器信号进入 STM32 前必须确认该脚是否 5V tolerant，并加入串阻/钳位或电平转换。
- A/B 相走线远离电机相线和开关电源电感，接口处加 ESD。

## 6. 4 路舵机接口

舵机接口采用 `GND / VSERVO / PWM` 三针形式。`VSERVO` 不从 STM32 核心板 3.3V 或电机驱动板 5V 直接取电，建议单独接 5V/6V BEC，并在接口附近放大电容和保护。

| 外部信号 | STM32 引脚 | LQFP100 脚号 | 外设/复用 | 方向 | 说明 |
|---|---:|---:|---|---|---|
| SERVO1_PWM | PD12 | 59 | TIM4_CH1 / AF2 | 输出 | 50Hz 舵机 PWM |
| SERVO2_PWM | PD13 | 60 | TIM4_CH2 / AF2 | 输出 | 50Hz 舵机 PWM |
| SERVO3_PWM | PD14 | 61 | TIM4_CH3 / AF2 | 输出 | 50Hz 舵机 PWM |
| SERVO4_PWM | PD15 | 62 | TIM4_CH4 / AF2 | 输出 | 50Hz 舵机 PWM |
| VSERVO_SENSE | PB0 | 35 | ADC12_IN8 | 模拟输入 | 舵机电源电压分压采样，可选 |

## 7. 大脑通信 USB-C 接口

该 Type-C 口只作为核心板和单板计算机之间的数据通信接口。STM32 作为 USB Device，大脑作为 USB Host，推荐固件实现 USB CDC ACM。

| 外部信号 | STM32 引脚 | LQFP100 脚号 | 外设/复用 | 方向 | 说明 |
|---|---:|---:|---|---|---|
| USB_FS_DM | PA11 | 70 | OTG_FS_DM / AF10 | 双向 | Type-C D- |
| USB_FS_DP | PA12 | 71 | OTG_FS_DP / AF10 | 双向 | Type-C D+ |
| USB_VBUS_DET | PA9 | 68 | OTG_FS_VBUS | 输入 | 经分压/限流检测 VBUS，不给核心板反向供电 |
| USB_ID_TP / EXP | PA10 | 69 | OTG_FS_ID / AF10 或 GPIO | 输入 | Type-C UFP 通常不用 ID，可只留测试点或扩展 |

Type-C 连接器建议：

- `CC1` 和 `CC2` 各接 5.1k 到 GND，声明本板为 UFP/device。
- D+/D- 串联 22R 左右阻值需结合 USB PHY 实测决定，接口侧放低电容 ESD。
- VBUS 只做检测，不和核心板 5V 主电源直接并联。
- USB 差分线按 90Ω 差分阻抗、等长、少过孔、有连续参考地设计。

## 8. SWD 无线 DAP 下载/调试接口

| 调试器信号 | STM32 引脚 | LQFP100 脚号 | 方向 | 说明 |
|---|---:|---:|---|---|
| VTref_3V3 | 3V3 | - | 电源参考 | 给调试器识别目标电平，不建议由调试器给整板供电 |
| GND | GND | - | 地 | 必接 |
| SWDIO | PA13 | 72 | 双向 | SWD 数据 |
| SWCLK | PA14 | 76 | 输入 | SWD 时钟 |
| NRST | NRST | 14 | 输入 | 目标复位 |
| SWO | PB3 | 89 | 输出 | 可选 Trace/SWO |
| DBG_UART_TX | PA2 | 25 | 输出 | USART2_TX，接 DAP RX 或调试串口 RX |
| DBG_UART_RX | PA3 | 26 | 输入 | USART2_RX，接 DAP TX 或调试串口 TX |

说明：

- 建议使用 2x5 1.27mm ARM Cortex Debug 接口，旁边再兼容无线 DAP 图片中的 2x5 排针定义。
- `PA2/PA3` 同时可作为大脑串口备用链路，硬件上建议用跳帽或 0R 电阻选择连接到 DAP 串口还是外部 UART。

## 9. ESP32-C3 冗余无线调参模块

推荐使用 `ESP32-C3-WROOM-02` 或 `ESP32-C3-WROOM-02U` 模块。STM32 与 ESP32-C3 之间使用 UART，ESP32-C3 用于后续 WiFi/BLE 上位机调参、日志、远程状态查看，不直接绕过 STM32 控制电机。

| 外部信号 | STM32 引脚 | LQFP100 脚号 | 外设/复用 | 方向 | 说明 |
|---|---:|---:|---|---|---|
| ESP_UART_TX | PC10 | 78 | USART3_TX / AF7 | 输出 | STM32 发到 ESP32-C3 RXD |
| ESP_UART_RX | PC11 | 79 | USART3_RX / AF7 | 输入 | STM32 接收 ESP32-C3 TXD |
| ESP_EN | PE0 | 97 | GPIO Output | 输出 | 控制 ESP32-C3 EN，需外部上拉/RC |
| ESP_BOOT | PE1 | 98 | GPIO Output | 输出 | 控制 ESP32-C3 下载模式 strapping，通常接 GPIO9 相关下载控制电路 |
| ESP_INT | PE6 | 5 | GPIO/EXTI | 输入 | ESP32-C3 通知 STM32 有无线数据或状态变化 |

ESP32-C3 模块布局要求：

- PCB 天线版本应靠板边放置，天线区域避免铜皮、走线和器件。
- 如果天线无法伸出板外，按 Espressif 指南给天线区域四周预留约 15mm 禁布空间。
- ESP32-C3 的 3.3V 供电要单独磁珠/滤波并预留足够去耦，WiFi 峰值电流不要影响 IMU 和 ADC。

## 10. IMU 与 OLED

MPU6050 与 OLED 共用 I2C1。OLED 刷新频率较低，不应在高频控制环里阻塞 I2C。

| 外部信号 | STM32 引脚 | LQFP100 脚号 | 外设/复用 | 方向 | 说明 |
|---|---:|---:|---|---|---|
| I2C1_SCL | PB8 | 95 | I2C1_SCL / AF4 | 双向 | MPU6050 + OLED + 外部 I2C 传感器 |
| I2C1_SDA | PB9 | 96 | I2C1_SDA / AF4 | 双向 | MPU6050 + OLED + 外部 I2C 传感器 |
| MPU_INT | PE4 | 3 | GPIO/EXTI | 输入 | MPU6050 数据就绪/中断 |
| OLED_RST | PE5 | 4 | GPIO Output | 输出 | 可选 OLED 复位脚；若 OLED 模块无 RST，可作扩展 |

建议：

- I2C 上拉到 3.3V，推荐 2.2k-4.7k，按总线长度和模块数量调整。
- MPU6050 已 EOL，建议同板预留 ICM-42670/ICM-42688/BMI088 等新 IMU 的焊盘或 SPI/I2C 扩展接口。
- IMU 远离 DCDC 电感、电机驱动板、ESP32 天线、USB 连接器，靠近机器人质心。

## 11. CAN、RS485、SPI、I2C 扩展

| 接口 | 外部信号 | STM32 引脚 | LQFP100 脚号 | 外设/复用 | 说明 |
|---|---|---:|---:|---|---|
| CAN1 | CAN1_RX | PD0 | 81 | CAN1_RX / AF9 | 接 3.3V CAN 收发器 RXD |
| CAN1 | CAN1_TX | PD1 | 82 | CAN1_TX / AF9 | 接 3.3V CAN 收发器 TXD |
| RS485 | RS485_TX | PC12 | 80 | UART5_TX / AF8 | 接 RS485 收发器 DI |
| RS485 | RS485_RX | PD2 | 83 | UART5_RX / AF8 | 接 RS485 收发器 RO |
| RS485 | RS485_DE | PD3 | 84 | GPIO Output | 485 方向控制，DE 与 /RE 可合并 |
| SPI2 | SPI2_NSS | PB12 | 51 | SPI2_NSS / AF5 | 外部 SPI 片选，可作 GPIO CS |
| SPI2 | SPI2_SCK | PB13 | 52 | SPI2_SCK / AF5 | 外部 SPI 时钟 |
| SPI2 | SPI2_MISO | PB14 | 53 | SPI2_MISO / AF5 | 外部 SPI MISO |
| SPI2 | SPI2_MOSI | PB15 | 54 | SPI2_MOSI / AF5 | 外部 SPI MOSI |
| I2C2 | I2C2_SCL | PB10 | 47 | I2C2_SCL / AF4 | 外部 I2C 备用总线 |
| I2C2 | I2C2_SDA | PB11 | 48 | I2C2_SDA / AF4 | 外部 I2C 备用总线 |

CAN 接口建议预留：

- CANH/CANL ESD。
- 120Ω 终端电阻跳帽。
- 共模电感位置，可先 0R/短接。

RS485 接口建议预留：

- A/B 端 TVS。
- 120Ω 终端电阻跳帽。
- 偏置电阻位置。

## 12. 电源监测、控制与板载状态

| 功能 | STM32 引脚 | LQFP100 脚号 | 外设/复用 | 方向 | 说明 |
|---|---:|---:|---|---|---|
| ESTOP_N | PC13 | 7 | GPIO/EXTI | 输入 | 急停输入；PC13 输出能力弱，用作输入合适 |
| BRAIN_PWR_EN | PE7 | 38 | GPIO Output | 输出 | 如果大脑 DC-DC 模块有 EN 脚，可由 STM32 控制 |
| BRAIN_5V_PG | PE8 | 39 | GPIO Input | 输入 | 如果大脑 DC-DC 模块有 Power Good，可接入 |
| BOARD_TEMP_NTC | PB1 | 36 | ADC12_IN9 | 模拟输入 | 板载 NTC 或电源温度采样 |
| STATUS_LED | PE10 | 41 | GPIO Output | 输出 | 板载状态灯，建议低电流 |
| AUX_GPIO | PE12 | 43 | GPIO | 双向 | 预留扩展；注意 TIM1 相关复用，不建议随意占用 |

## 13. 建议暴露的剩余接口

以下引脚当前不作为核心功能固定占用，建议通过排针/焊盘尽量暴露。使用时需要在 CubeMX 中重新检查外设冲突。

| STM32 引脚 | LQFP100 脚号 | 可用方向/外设 | 建议用途 |
|---|---:|---|---|
| PA8 | 67 | MCO1 / TIM1_CH1 / GPIO | 时钟输出、外部同步、扩展 GPIO；不要与 TIM1 电机功能混用 |
| PA10 | 69 | USART1_RX / OTG_FS_ID / GPIO | USB ID 测试点或扩展 GPIO |
| PA15 | 77 | SPI1_NSS / TIM2_CH1_ETR / GPIO | 扩展 GPIO；需关闭 JTAG 保留 SWD |
| PB2 | 37 | BOOT1 / GPIO | 建议下拉并只作测试点或低风险输入 |
| PD4 | 85 | USART2_RTS / GPIO | 扩展 GPIO |
| PD5 | 86 | USART2_TX / GPIO | 如果不用 PA2/PA3 调试串口，可改作 USART2 |
| PD6 | 87 | USART2_RX / GPIO | 如果不用 PA2/PA3 调试串口，可改作 USART2 |
| PD7 | 88 | USART2_CK / GPIO | 扩展 GPIO |
| PD8 | 55 | USART3_TX / FSMC_D13 / GPIO | 扩展 GPIO；USART3 已默认给 ESP32-C3 |
| PD9 | 56 | USART3_RX / FSMC_D14 / GPIO | 扩展 GPIO；USART3 已默认给 ESP32-C3 |
| PD10 | 57 | USART3_CK / FSMC_D15 / GPIO | 扩展 GPIO |
| PD11 | 58 | USART3_CTS / FSMC_A16 / GPIO | 扩展 GPIO |
| PC9 | 66 | TIM8_CH4 / SDIO_D1 / I2C3_SDA / GPIO | 扩展 GPIO 或外部同步 |
| PB4 | 90 | NJTRST / TIM3_CH1 / SPI1_MISO / GPIO | 扩展 GPIO；需关闭 JTAG 保留 SWD |
| PB5 | 91 | TIM3_CH2 / SPI1_MOSI / GPIO | 扩展 GPIO |
| PB6 | 92 | I2C1_SCL / TIM4_CH1 / USART1_TX / GPIO | 扩展 GPIO；I2C1 默认不用此脚 |
| PB7 | 93 | I2C1_SDA / TIM4_CH2 / USART1_RX / GPIO | 扩展 GPIO；I2C1 默认不用此脚 |

## 14. CubeMX 外设配置建议

| 外设 | 模式 | 对应功能 |
|---|---|---|
| TIM1 | PWM Generation CH1/CH2/CH3 | 电机 0 三路控制输入 |
| TIM8 | PWM Generation CH1/CH2/CH3 | 电机 1 三路控制输入 |
| TIM5 | Encoder Mode TI1/TI2 | 左轮 AB 编码器 |
| TIM3 | Encoder Mode TI1/TI2 | 右轮 AB 编码器 |
| TIM4 | PWM Generation CH1/CH2/CH3/CH4 | 4 路舵机 |
| ADC1/ADC2 | Scan + DMA | 电流、电压、温度采样 |
| USB_OTG_FS | Device Only / CDC ACM | 与大脑通信 |
| USART2 | Asynchronous | 调试串口或大脑串口备用 |
| USART3 | Asynchronous | ESP32-C3 通信 |
| UART5 | Asynchronous | RS485 通信 |
| I2C1 | Fast Mode 400kHz | MPU6050 + OLED |
| I2C2 | Fast Mode 400kHz | 外部 I2C 扩展 |
| SPI2 | Master | 外部 SPI 扩展 |
| CAN1 | Normal/Silent Loopback 可切换 | 外部 CAN 总线 |
| GPIO EXTI | Falling/Rising 按需求 | 急停、编码器 Z、IMU INT、驱动故障 |
| IWDG | Enable | 软件异常时复位 |

## 15. 关键冲突与注意事项

1. `TIM4` 已用于 4 路舵机，因此不要再用 `PB6/PB7/PB8/PB9` 做 TIM4 编码器；本方案已改用 `TIM5` 和 `TIM3`。
2. `PA11/PA12` 固定给 USB FS，不能再拿去做 TIM1_CH4、CAN、GPIO 等用途。
3. `PA13/PA14` 固定给 SWD；`PB3` 建议留给 SWO，不建议占用。
4. `PB4`、`PA15` 等 JTAG 相关脚如果后续使用，需要在固件中关闭 JTAG、保留 SWD。
5. `PC13/PC14/PC15` 属于低驱动能力/备份域相关引脚，不建议驱动 LED、大电容或高速信号。
6. `PB14/PB15` 当前用于 SPI2 扩展；如果未来想启用 USB OTG HS 内置 FS PHY，这两个脚会冲突。
7. 由电机驱动板 5V 给核心板供电时，必须先实测该 5V 的最大电流、纹波和上电顺序；ESP32-C3 WiFi 峰值电流可能导致 3.3V 瞬时跌落。
8. 所有进入 STM32 ADC 的外部模拟信号必须保证在 `0V~VDDA` 范围内，并加串阻、RC 滤波和钳位保护。
9. 所有外部接口默认按 3.3V 逻辑设计；若编码器、舵机反馈或电机驱动板信号为 5V，必须确认 STM32 对应脚 5V tolerant 条件，模拟脚绝不能直接进 5V。

## 16. LQFP100 物理脚位占用速查表

该表按 LQFP100 物理脚号排列，方便画原理图时逐脚检查。

| 脚号 | STM32 引脚 | 当前分配/网络 | 备注 |
|---:|---|---|---|
| 1 | PE2 | ENC_L_Z | 左轮编码器 Index，可选 |
| 2 | PE3 | ENC_R_Z | 右轮编码器 Index，可选 |
| 3 | PE4 | MPU_INT | IMU 中断 |
| 4 | PE5 | OLED_RST / EXP | OLED 复位或扩展 |
| 5 | PE6 | ESP_INT | ESP32-C3 中断 |
| 6 | VBAT | VBAT_RTC | RTC 备份电源 |
| 7 | PC13 | ESTOP_N | 急停输入 |
| 8 | PC14 | LSE_IN | 可选 32.768kHz |
| 9 | PC15 | LSE_OUT | 可选 32.768kHz |
| 10 | VSS | GND | 数字地 |
| 11 | VDD | 3V3 | 数字电源 |
| 12 | PH0 | HSE_IN | 主晶振输入 |
| 13 | PH1 | HSE_OUT | 主晶振输出 |
| 14 | NRST | NRST | 复位 |
| 15 | PC0 | M0_CS1 | ADC123_IN10 |
| 16 | PC1 | M0_CS2 | ADC123_IN11 |
| 17 | PC2 | M0_CS3 | ADC123_IN12 |
| 18 | PC3 | M1_CS1 | ADC123_IN13 |
| 19 | VDD | 3V3 | 数字电源 |
| 20 | VSSA | AGND | 模拟地 |
| 21 | VREF+ | VREF_3V3_A | ADC 参考 |
| 22 | VDDA | 3V3_A | 模拟电源 |
| 23 | PA0 | ENC_L_A | TIM5_CH1 |
| 24 | PA1 | ENC_L_B | TIM5_CH2 |
| 25 | PA2 | DBG_UART_TX | USART2_TX |
| 26 | PA3 | DBG_UART_RX | USART2_RX |
| 27 | VSS | GND | 数字地 |
| 28 | VDD | 3V3 | 数字电源 |
| 29 | PA4 | M1_CS2 | ADC12_IN4 |
| 30 | PA5 | M1_CS3 | ADC12_IN5 |
| 31 | PA6 | ENC_R_A | TIM3_CH1 |
| 32 | PA7 | ENC_R_B | TIM3_CH2 |
| 33 | PC4 | BMEA / VMOT_SENSE | ADC12_IN14 |
| 34 | PC5 | MDRV_5V_SENSE | ADC12_IN15 |
| 35 | PB0 | VSERVO_SENSE | ADC12_IN8 |
| 36 | PB1 | BOARD_TEMP_NTC | ADC12_IN9 |
| 37 | PB2 | BOOT1_TEST / EXP | 建议下拉，仅低风险用途 |
| 38 | PE7 | BRAIN_PWR_EN | 可选 DC-DC 使能 |
| 39 | PE8 | BRAIN_5V_PG | 可选 DC-DC PG |
| 40 | PE9 | M0_IN1 | TIM1_CH1 |
| 41 | PE10 | STATUS_LED | 状态灯 |
| 42 | PE11 | M0_IN2 | TIM1_CH2 |
| 43 | PE12 | AUX_GPIO | 预留 |
| 44 | PE13 | M0_IN3 | TIM1_CH3 |
| 45 | PE14 | MDRV_FAULT_N | 驱动故障输入 |
| 46 | PE15 | M_EN | 电机使能 |
| 47 | PB10 | I2C2_SCL | 外部 I2C2 |
| 48 | PB11 | I2C2_SDA | 外部 I2C2 |
| 49 | VCAP_1 | VCAP_1 | 内核稳压电容 |
| 50 | VDD | 3V3 | 数字电源 |
| 51 | PB12 | SPI2_NSS | SPI 扩展 |
| 52 | PB13 | SPI2_SCK | SPI 扩展 |
| 53 | PB14 | SPI2_MISO | SPI 扩展 |
| 54 | PB15 | SPI2_MOSI | SPI 扩展 |
| 55 | PD8 | EXP_GPIO | 预留 |
| 56 | PD9 | EXP_GPIO | 预留 |
| 57 | PD10 | EXP_GPIO | 预留 |
| 58 | PD11 | EXP_GPIO | 预留 |
| 59 | PD12 | SERVO1_PWM | TIM4_CH1 |
| 60 | PD13 | SERVO2_PWM | TIM4_CH2 |
| 61 | PD14 | SERVO3_PWM | TIM4_CH3 |
| 62 | PD15 | SERVO4_PWM | TIM4_CH4 |
| 63 | PC6 | M1_IN1 | TIM8_CH1 |
| 64 | PC7 | M1_IN2 | TIM8_CH2 |
| 65 | PC8 | M1_IN3 | TIM8_CH3 |
| 66 | PC9 | EXP_GPIO | 预留 |
| 67 | PA8 | EXP_MCO1 | 时钟输出/扩展 |
| 68 | PA9 | USB_VBUS_DET | USB VBUS 检测 |
| 69 | PA10 | USB_ID_TP / EXP | USB ID 测试点或扩展 |
| 70 | PA11 | USB_FS_DM | USB D- |
| 71 | PA12 | USB_FS_DP | USB D+ |
| 72 | PA13 | SWDIO | SWD 必留 |
| 73 | VCAP_2 | VCAP_2 | 内核稳压电容 |
| 74 | VSS | GND | 数字地 |
| 75 | VDD | 3V3 | 数字电源 |
| 76 | PA14 | SWCLK | SWD 必留 |
| 77 | PA15 | EXP_GPIO | 需关闭 JTAG 保留 SWD |
| 78 | PC10 | ESP_UART_TX | USART3_TX |
| 79 | PC11 | ESP_UART_RX | USART3_RX |
| 80 | PC12 | RS485_TX | UART5_TX |
| 81 | PD0 | CAN1_RX | CAN 收发器 |
| 82 | PD1 | CAN1_TX | CAN 收发器 |
| 83 | PD2 | RS485_RX | UART5_RX |
| 84 | PD3 | RS485_DE | 485 方向控制 |
| 85 | PD4 | EXP_GPIO | 预留 |
| 86 | PD5 | EXP_GPIO | 可重分配 USART2_TX |
| 87 | PD6 | EXP_GPIO | 可重分配 USART2_RX |
| 88 | PD7 | EXP_GPIO | 预留 |
| 89 | PB3 | SWO | 调试 Trace |
| 90 | PB4 | EXP_GPIO | 需关闭 JTAG 保留 SWD |
| 91 | PB5 | EXP_GPIO | 预留 |
| 92 | PB6 | EXP_GPIO | 预留 |
| 93 | PB7 | EXP_GPIO | 预留 |
| 94 | BOOT0 | BOOT0 | 下拉，测试点 |
| 95 | PB8 | I2C1_SCL | MPU6050/OLED |
| 96 | PB9 | I2C1_SDA | MPU6050/OLED |
| 97 | PE0 | ESP_EN | ESP32-C3 使能 |
| 98 | PE1 | ESP_BOOT | ESP32-C3 下载控制 |
| 99 | VSS | GND | 数字地 |
| 100 | VDD | 3V3 | 数字电源 |
