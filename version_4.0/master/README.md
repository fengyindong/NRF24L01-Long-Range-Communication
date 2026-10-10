# STM32F407 MQTT 网关上手与验收

本目录是 STM32F407VGT6 网关固件（CMSIS + STM32F4 Standard Peripheral Library，裸机），负责 nRF24L01 射频轮询、CLR970 MQTT 连接、命令队列和继电器状态上报。F103 从机仍使用 `stm32_slave` 固件采集 AHT20/BMP280，并驱动 PB12～PB15 四路继电器。

## Keil/CMSIS 工程输入

在 Keil MDK 中新建 STM32F407VGTx 工程，加入 `stm32f407_master/inc/*.h`、`stm32f407_master/src/*.c`，并加入下列芯片支持文件（版本应来自同一套 CMSIS/SPL）：

- 启动文件：`startup_stm32f40_41xxx.s`。
- CMSIS：`core_cm4.c`/对应 CMSIS Core 头文件、设备头文件 `stm32f4xx.h`，以及 `system_stm32f4xx.c`。
- SPL 源文件：`stm32f4xx_gpio.c`、`stm32f4xx_rcc.c`、`stm32f4xx_flash.c`、`stm32f4xx_pwr.c`、`stm32f4xx_spi.c`、`stm32f4xx_usart.c`、`misc.c`（以及所用 SPL 版本要求的通用 `stm32f4xx_*.c` 支持文件）。
- 链接脚本/分散加载文件按实际 F407VGT6 闪存和 RAM 容量设置；不要把 F103 的启动文件或 SPL 混入本目标。

Target 的 C/C++ 预处理宏必须包含 `STM32F40_41xxx;USE_STDPERIPH_DRIVER`。Include 路径至少包含本目录 `inc`、CMSIS `Include`、CMSIS `Device/ST/STM32F4xx/Include`、SPL `inc`；汇编器路径加入启动文件所在目录。系统时钟使用 8 MHz HSE：`system_stm32f4xx.c` 的 `HSE_VALUE` 应为 `8000000U`，运行时 `board.c` 配置 PLL 为 HSE/8×336/2，SYSCLK 为 168 MHz（APB1 42 MHz、APB2 84 MHz）。晶振或 HSE_VALUE 不一致会导致串口波特率和 SysTick 全部偏差。

编译后烧录并打开“Debug Serial”：USART1 PA9/PA10，115200 8-N-1。CLR970 使用 USART2 PA2/PA3，9600 8-N-1。`master_config.h` 中按现场修改 `GATEWAY_MQTT_ROOT`、节点表和队列/超时参数；当前默认节点表只有 `{1}`。

## F407 接线

| F407 | 信号 | 模块/说明 |
|---|---|---|
| PA4 | nRF CSN | 推挽 GPIO |
| PA5/PA6/PA7 | nRF SCK/MISO/MOSI | SPI1 AF5，Mode 0 |
| PB0 | nRF CE | 推挽 GPIO |
| PA2/PA3 | CLR970 TX/RX | USART2 TX/RX，AF7，交叉连接 |
| PA9/PA10 | 调试 TX/RX | USART1 TX/RX，AF7，接 USB-TTL |

F103 从机接线仍见根目录 README：PA4 CSN、PB0 CE、PA5/6/7 SPI，PA9 为 115200 调试 TX；AHT20/BMP280 为 PB6/PB7；继电器 1～4 为 PB12/PB13/PB14/PB15。继电器输出默认高电平断开、低电平吸合，上电四路断开。nRF PA+LNA 使用独立稳定的 3.3 V（建议至少 500 mA），模块旁放 100 nF、10 uF 和 100～220 uF，并与 MCU 共地。所有 UART 信号均为 3.3 V TTL。

## MQTTX 操作

连接到 `GATEWAY_MQTT_ROOT` 对应的 broker 后，订阅 `environment/gateway01/node/1/#`。向 `environment/gateway01/node/1/command/set` 发布 QoS 1 的 JSON；`command/set` **禁止 Retain**，否则网关重连会重复执行历史命令。每次操作使用新的非零 `command_id`。

单路（继电器 1 吸合）：

```json
{"node_id":1,"command_id":7001,"code":16,"channel":1,"state":1}
```

四路一次设置为开、关、开、关（`mask=15, states=5`）：

```json
{"node_id":1,"command_id":7002,"code":16,"mask":15,"states":5}
```

查询实际四路状态：

```json
{"node_id":1,"command_id":7003,"code":17}
```

成功时先收到 `environment/gateway01/node/1/command/result`，例如：

```json
{"node_id":1,"command_id":7002,"code":16,"result":0,"value":5}
```

继电器命令（16）和查询（17）还会发布 retained 的 `environment/gateway01/node/1/relay/state`：

```json
{"node_id":1,"command_id":7002,"relay1":1,"relay2":0,"relay3":1,"relay4":0,"state_mask":5}
```

`result=0` 表示成功；失败只发布 command/result，不覆盖最后一个有效 retained 状态。`channel/state`、`mask/states`、旧 `value` 编码互斥；`states` 不得在 `mask` 外置位。旧格式示例：`{"node_id":1,"command_id":7004,"code":16,"value":257}` 表示仅修改继电器 1 为吸合。

## 诊断与验收

F407 USART1 应能看到 `NRF PASS`、`RF CMD ...`、`RF NO_HW_ACK`/`RF NO_APP_RESPONSE`、`MQTT ...` 等阶段日志；CLR970 重连时会打印 `CLM BACKOFF`、`MQTT CONNECTED`/`MQTT SUBSCRIBED`。F103 PA9 启动应包含 `STM32 NODE BOOT id=1`、`NRF REG:`、`NRF PASS`、`SENSORS AHT=... BMP_ADDR=...`；命令执行会打印 `RELAY SET`、`RELAY GET`，重复业务命令打印 `CMD DUPLICATE`。

硬件验收顺序：先确认两端 3.3 V、共地、地址/频道 76 和串口输出；再确认 MQTT telemetry 持续发布；依次测试四个单路命令；测试 `mask=15,states=5` 的同步结果；发送查询并核对 ACK 与 retained 状态；临时断开 nRF 或从机，确认命令不会丢失且恢复后只执行一次；断开网络/重启 CLR970，确认自动退避重连、重新订阅并继续发布。本文只给出可复现步骤；本机未连接实际 F407/F103、nRF、继电器或 CLR970，不能据此宣称硬件验收已完成。
