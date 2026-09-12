# STM32F103C8T6 主机网关（标准外设库）

本目录是 NodeMCU 网关的 STM32F103C8T6 移植版。它复用 `stm32_slave/inc` 与 `stm32_slave/src` 中的 `app_protocol.*`、`nrf24.*`（复制到本工程或在工程中加入相同源文件），不需要传感器驱动。

## 串口接线

```text
USART1 PA9/PA10：PA9 -> USB-TTL RX，PA10 <- USB-TTL TX（调试）
USART2 PA2/PA3：PA2 -> CLR970 UART_RXD，PA3 <- CLR970 UART_TXD
GND：STM32、CLR970、USB-TTL 共地
```

CLR970 嵌入板的预留 UART 是 3.3 V TTL。`CLM_BAUD` 必须与模块当前 AT 口波特率一致。本次调试日志显示当前固件使用 9600，因此配置已改为 9600；如果用 USB-TTL 发送 `AT+IPR?` 返回 115200，则应将 `CLM_BAUD` 改回 115200后重新编译。

## 编译文件

加入标准库 `stm32f10x`、CMSIS 启动文件，以及：

```text
stm32_master/src/main.c
stm32_slave/src/nrf24.c
stm32_slave/src/app_protocol.c
```

头文件搜索路径加入 `stm32_master/inc` 和 `stm32_slave/inc`。

程序启动后自动执行 `AT`、SIM/注册/附着/PDP 状态查询。已有可用 PDP 时不再重复改 APN；未附着时才设置 APN、附着和激活。`AT+IMQTTCONN`、订阅和发布命令都会等待最终 URC，超时后按 5/10/20/40/60 秒退避重连，期间保留待发遥测队列。USB-TTL 输入任意 AT 命令并回车可透传到 CLR970。

## MQTT 配置与主题

在 `inc/master_config.h` 填写 EMQX Cloud 控制台给出的真实接入点、端口、用户名和密码。默认 `broker.emqx.io:1883` 是公共测试 Broker，不是您的 EMQX Cloud 部署。每个同时在线的 MQTT 客户端必须使用不同 Client ID，MQTTX 不能使用 `stm32-gateway01`。

MQTTX 建议订阅：

```text
environment/gateway01/#
```

遥测实际发布到：

```text
environment/gateway01/node/1/telemetry
```

发布 JSON 中的 `temperature_c`、`humidity_rh`、`pressure_hpa` 已是带两位小数的工程值，同时保留 `pressure_pa`、传感器状态位、数据年龄和序号用于诊断。

## 连接调试日志

每次启动必须先看到实际编译进固件的配置：

```text
CFG FW=master-20260901-r3
CFG APN=CMIOTGZSW.GD.MNC008.MCC460.GPRS
CFG MQTT=<EMQX接入点>:<端口> CID=stm32-gateway01 ROOT=environment/gateway01
```

完整成功标志为：

```text
NET registered=1
NET attached=1
NET pdp_active=1
MQTT CONNECTED result=0
MQTT SUBSCRIBED
MQTT PUB ACK queue=0
```

`AT+IMQTTPUB` 使用明文 JSON，不再使用 `AT+IMQTTPUBIN` HEXString；这样 MQTTX 中可直接看到可读 JSON。例如：

```text
{"node_id":1,"temperature_c":24.96,"humidity_rh":68.61,"pressure_hpa":1003.20}
```

只有收到以下应答才表示真正发布成功：

```text
CLM << [+IMQTTPUB:<packet_id>,0]
MQTT PUB ACK queue=0
```

如果 MQTTX 使用的是 EMQX Cloud，网关也必须使用同一个 Cloud 接入域名和端口，而不能继续使用 `broker.emqx.io`。同时将 `MQTT_USER`/`MQTT_PASSWORD` 填成 Cloud 创建的账号，并使用与 MQTTX 不同的 `MQTT_CLIENT_ID`。

若启动日志仍显示 `CFG APN=` 为空，或没有新的固件版本号，说明 Keil 没有编译当前目录的 `master_config.h/main.c`：检查 Include Paths 中 `stm32_master/inc` 的顺序，确认 Source Group 使用当前 `stm32_master/src/main.c`，然后 Clean Target、Rebuild All 再烧录。

当前 `main.c` 还包含硬编码的 APN 兜底值；即使配置宏为空，重新编译后的启动日志也不应为空。因此仍显示空值时，几乎可以确定烧录的是旧目标文件，或工程引用了另一份同名头文件。

如果启动后只有：

```text
CLM >> AT
```

而 5 秒后出现 `CLM RESPONSE TIMEOUT`，没有任何 `CLM <<`，这不是 MQTT 认证错误，而是 STM32 没收到 CLR970 的 UART 响应。检查 PA2(TX)→CLR970 RXD、PA3(RX)←CLR970 TXD、共地、模块供电和 `CLM_BAUD`；CLR970 上电后至少等待 3 秒再发第一条 AT。可先烧录 `stm32_clr970_test` 工程验证 USART2，确认该工程也使用 115200。

本次日志中 `CFG FW=master-20260831-r1` 、`CFG APN=` 和 `CID=mqttx_69946bd3` 都与当前源码不一致，表明烧录的是旧 HEX，或 Keil 引用了另一份 `main.c/master_config.h`。请在 Source Group 只保留 `stm32_master/src/main.c`，将 `stm32_master/inc` 放在 Include Paths 前面，执行 Clean Target → Rebuild All 后重新烧录。新固件必须打印 `CFG FW=master-20260901-r3` 。

重建后如果仍不出现 `CLM << AT`，超时日志会输出 `rx_bytes` 和 `uart_errors`：`rx_bytes=0` 表示模块没发回数据或 TX/RX 接反；`uart_errors>0` 通常是波特率不一致、电平或供电问题。

USART2 已改为 RX 中断环形缓冲，所以需在 Keil 中确认已加入 `misc.c` 和 `misc.h`，并确保启用 `USART2_IRQHandler`。正常日志应为：

```text
CFG FW=master-20260901-r3
CFG CLM_BAUD=9600 BOOT_DELAY_MS=3000
CLM >> AT
CLM << AT
CLM << OK
CLM >> AT+CMEE=2
CLM << OK
CLM >> AT+CPIN?
CLM << +CPIN: READY
CLM << OK
CLM >> AT+CSQ
CLM << +CSQ: 23,99
CLM << OK
CLM >> AT+CEREG?
CLM << +CEREG: 0,1
NET registered=1
CLM << OK
CLM >> AT+CGATT?
CLM << +CGATT: 1
NET attached=1
CLM << OK
CLM >> AT+CGACT?
CLM << +CGACT: 1,1
NET pdp_active=1
CLM << OK
CLM >> AT+IMQTTMODE=2,0,0
CLM >> AT+IMQTTPARA=...
CLM >> AT+IMQTTADDR=...
CLM >> AT+IMQTTUSER=...
CLM >> AT+IMQTTCONN
CLM << +IMQTTCONN:0
CLM >> AT+IMQTTSUB=environment/gateway01/node/+/command/set,1
CLM << +IMQTTSUB:...
MQTT SUBSCRIBED
```

Keil主机工程需要把 `stm32_master/src/mqtt_command.c` 加入 Source Group，并把 `stm32_master/inc` 加入头文件搜索路径。继电器命令订阅主题为 `environment/gateway01/node/+/command/set`，状态上报主题为 `environment/gateway01/node/<id>/relay/state`。

## 远距离射频参数

主机和从机必须同时烧录同一份 `stm32_slave/src/nrf24.c`。当前稳定联调参数为频道76、1 Mbps、16位CRC、自动ACK和 `SETUP_RETR=0x35`。不要只修改一端。确认近距离通信稳定后，才建议把两端一起改为250 kbps（`RF_SETUP=0x26`）；若修改后出现 `RF NO_HW_ACK`，先回退到当前稳定参数，再检查供电和地址。
