一、整体架构
text
┌──────────────┐    MQTT/4G     ┌──────────────┐    nRF24 2.4G    ┌──────────────┐
│  云平台      │?──────────────?│  主机 F407   │?────────────────?│  从机 F103   │
│ (EMQX等)     │                │  (网关)      │                  │  (传感器节点)│
└──────────────┘                └──────────────┘                  └──────────────┘
                                      │                                 │
                                      │ 轮询/命令                        │ 采集/执行
                                      ▼                                 ▼
                                CLM970 4G模块                     AHT20/BMP280/继电器
主机 通过 CLM970 4G 模块连 MQTT，负责：

接收云端命令，解析后通过 nRF24 下发给从机；

轮询从机获取遥测，上传到 MQTT；

维护命令队列、发布队列、节点状态。

从机 通过 nRF24 与主机通信，负责：

采集 AHT20 温湿度、BMP280 气压；

接收主机命令，执行 LED/继电器/采样周期设置；

返回遥测或命令 ACK。

二、主机端模块与工作流程
1. board.c / board.h —— 板级驱动
功能	说明
系统时钟	HSE 8MHz → PLL → 168MHz，SysTick 1ms
调试串口	USART1，PA9 TX，115200
CLM 串口	USART2，PA2/PA3，9600，接收中断+512字节环形缓冲
nRF24 SPI	硬件 SPI1：PA5=SCK、PA6=MISO、PA7=MOSI；PA4=CSN、PB0=CE
延时	board_delay_us 粗略微秒延时
调试输出	board_debug_write 阻塞发送字符串
关键点：

USART2 中断里把收到的字节放入环形缓冲，主循环用 board_clm_read 非阻塞读取，避免丢数据。

提供 board_clm_rx_bytes / board_clm_uart_errors 用于诊断。

2. clm970.c / clm970.h —— 4G/MQTT 驱动
核心状态机（ClmState）：

text
BOOT_WAIT → AT → VERBOSE → SIM_QUERY → SIGNAL_QUERY → REG_QUERY
→ ATTACH_QUERY → PDP_QUERY → PDP_CONFIG → ATTACH → ACTIVATE
→ MQTT_CONFIG → CONNECT → SUBSCRIBE → READY
                                      ↘ BACKOFF（出错时）
主要功能：

发送 AT 指令，自动补 CRLF。

解析 URC/响应：+CPIN、+CEREG、+CGATT、+CGACT、+IMQTTCONN、+IMQTTSUB、+IMQTTPUBEX、+IMQTTRCVPUB 等。

维护 MQTT 发布队列（publish_queue），对遥测/状态主题做“同主题只保留最新值”去重。

通过回调 message_handler 把下行 PUBLISH 消息交给 gateway_on_mqtt_message。

断线后指数退避重连（enter_backoff）。

发布流程：

text
clm970_publish() → 入队
clm970_task() → 在 READY 状态发送 AT+IMQTTPUBEX
收到 '>' → 发送 payload
收到 +IMQTTPUBEX:...,0 → 出队，成功
接收流程：

text
+IMQTTRCVPUB:<id>,"<topic>",<len>,"<payload>"
→ parse_incoming_publish() 解析并反转义 \"
→ message_handler(topic, payload) → gateway_on_mqtt_message()
3. mqtt_command.c / mqtt_command.h —— 云端命令解析
输入：MQTT 主题 + JSON 载荷。

输出：MqttCommand { node, code, id, value } 或错误码。

解析流程：

json_valid() 检查 JSON 结构（只允许数字键值，禁止重复/未知字段）。
提取 node_id、command_id、code。
校验主题中的节点 ID 与 JSON 一致。
校验节点是否在已知列表中。
继电器命令：只接受 channel + state（单路），编码为 value。
提供 mqtt_command_parser_version() 和 mqtt_command_last_detail() 用于诊断。

4. gateway.c / gateway.h —— 网关业务逻辑
数据结构：

command_queue[]：缓存待下发的云端命令。

publish_queue[]：缓存待上传的 MQTT 消息。

nodes[]：每个从机的 ID 和下次轮询时间。

三个任务：

任务	职责
gateway_radio_task()	无线状态机：发送命令/轮询，等待从机响应
gateway_publish_task()	把发布队列交给 clm970_publish()
gateway_on_mqtt_message()	MQTT 下行回调，解析命令入队
无线状态机：

text
IDLE → 有命令？发命令；否则有到期节点？发 POLL
     → nrf24_send() → nrf24_start_listening() → WAIT
WAIT → 收到合法响应 → finish_response()
     → 超时 → 回 IDLE，稍后重试
响应处理：

命令 ACK：发布命令结果到 MQTT，若继电器命令成功则发布继电器状态（retained）。

遥测：发布遥测 JSON 到 MQTT，并安排下次轮询。

5. nrf24.c / nrf24.h —— 主机 nRF24 驱动
使用 硬件 SPI1，通过 board_spi1_transfer 收发。

主要接口：

nrf24_init()：配置通道 76、1Mbps、最大功率、2 字节 CRC、pipe0 自动 ACK、32 字节固定载荷。

nrf24_send()：切 PTX，写地址和载荷，CE 脉冲，轮询 TX_DS / MAX_RT。

nrf24_start_listening()：切 PRX，重写配置，清 FIFO，CE 拉高。

nrf24_receive()：检查 FIFO 状态，读 32 字节。

自动重发由 SETUP_RETR=0x35 控制（ARD=1.5ms，ARC=3 次）。

6. app_protocol.c / app_protocol.h —— 协议层
帧格式：AppHeader(16B) + payload(0~14B) + CRC(2B)，总长固定 32 字节。

app_build_frame()：填充帧头、载荷、CRC（小端序），剩余补零。

app_validate_frame()：校验帧长、魔数、版本、网络 ID、CRC。

继电器命令：

app_relay_encode()：单路编码，低 4 位掩码，bit8~11 状态。

app_relay_apply()：单路解码并计算新状态（mask 必须是 2 的幂）。

7. 主机 main.c
text
board_init() → 调试串口 → nrf24_init() + 自检
→ clm970_init(gateway_on_mqtt_message)
→ gateway_init()
for (;;) {
    clm970_task();          // 4G/MQTT 状态机
    gateway_radio_task();   // nRF24 收发
    gateway_publish_task(); // MQTT 发布队列
}
三、从机端模块与工作流程
1. main.c —— 从机入口
初始化顺序：

text
capture_reset_flags()   // 必须在 RCC_ClearFlag 前
board_init()            // 时钟、SysTick、PC13 LED
debug_uart_init()       // USART1 调试
relay_init()            // PB12~PB15 继电器
nrf24_init() + 自检     // GPIO 模拟 SPI
sensors_init()          // I2C 探测 AHT20/BMP280
sensors_force_sample()
nrf24_start_listening(SLAVE_ADDRESS)
主循环：

text
for (;;) {
    sensors_task(millis());   // 非阻塞采样
    radio_task();             // 非阻塞无线收发
}
2. sensor.c / sensor.h —— 传感器驱动
非阻塞状态机：

text
IDLE → 到采样时间或 force_pending
     → 发送 AHT20 测量命令 + 触发 BMP280 forced 转换
     → WAIT_AHT（等 85ms）
WAIT_AHT → 读 AHT20 7 字节，解析温湿度 + CRC 校验
         → 读 BMP280 6 字节，执行 64 位整数补偿
         → 更新快照，设置下次采样时间，回 IDLE
快照结构：SensorSnapshot { temperature_centi_c, humidity_centi_rh, pressure_pa, status, sampled_at_ms }。

3. i2c_bus.c / i2c_bus.h —— 模拟 I2C
使用 PB6=SCL、PB7=SDA，开漏输出。

提供：

i2c_write / i2c_write_reg

i2c_read（命令式设备，如 AHT20）

i2c_read_regs（寄存器式设备，如 BMP280）

i2c_bus_recover（9 个时钟 + STOP，解总线死锁）

4. nrf24.c / nrf24.h —— 从机 nRF24 驱动
使用 GPIO 模拟 SPI：PA5=SCK、PA6=MISO、PA7=MOSI，PA4=CSN、PB0=CE。

与主机 nRF24 功能一致，但底层 SPI 用 spi_rw 模拟。

关键点：

nrf24_start_listening 每次重写配置，避免兼容芯片丢配置。

nrf24_send 发送前清 FIFO，CE 脉冲 15us。

同样使用固定 32 字节载荷。

5. app_protocol.c / app_protocol.h —— 协议层
与主机共用同一套帧格式和 CRC 算法。

从机侧额外使用 app_relay_apply 校验单路继电器命令。

6. 从机 radio_task() 详细流程
text
1. nrf24_receive() 有数据？
2. app_validate_frame() 校验格式和 CRC
3. 检查 src_id == APP_MASTER_ID && dst_id == APP_NODE_ID
4. 打印 RX CMD / RX POLL 日志
5. 判断 master_was_lost
6. 请求去重：同 boot_id + seq 则复用缓存响应
7. 构造响应：build_response()
   - POLL → 遥测帧
   - COMMAND → 执行命令 + ACK 帧
8. 等 APP_RESPONSE_TURNAROUND_MS (20ms)
9. nrf24_send() 最多重试 3 次
10. 更新 last_master_ms，nrf24_start_listening()
7. 从机 build_response() 命令处理
命令	动作
APP_CMD_SET_LED	设置 PC13，返回新状态
APP_CMD_SET_RELAY	解析 channel + state，调用 app_relay_apply，写 BSRR
APP_CMD_GET_RELAY_STATE	返回当前 relay_state
APP_CMD_SET_SAMPLE_PERIOD	校验范围，设置周期并强制采样
APP_CMD_FORCE_SAMPLE	触发采样
APP_CMD_DEBUG_DUMP	打印 nRF 寄存器和传感器状态
命令去重：只有 command_id + command + value 完全相同才认为是重发，避免继电器重复动作。

四、主机与从机通信完整流程
场景 1：主机轮询遥测
text
主机 IDLE → 选到期节点 → 发 POLL 帧
从机 radio_task 收到 POLL → 构造遥测帧 → 等 20ms → 发遥测
主机 WAIT 收到遥测 → gateway_publish_task → MQTT 上传
场景 2：云端下发命令
text
云端 MQTT PUBLISH → CLM970 +IMQTTRCVPUB
→ clm970 解析 → gateway_on_mqtt_message()
→ mqtt_command_parse() → 入 command_queue
→ gateway_radio_task() 发 COMMAND 帧
从机 radio_task 收到 COMMAND → 执行 → 回 ACK
主机收到 ACK → 发布命令结果 + 继电器状态到 MQTT
场景 3：命令重发与去重
主机重发：若未收到 ACK，重试（GATEWAY_COMMAND_RETRY_MS）。

从机去重：

请求去重：同 boot_id + seq 直接复用缓存响应。

命令去重：同 command_id + command + value 不重复执行。

五、硬件驱动关键点
1. nRF24 收发
PTX：CE 脉冲 >10us 触发发送，等 TX_DS 或 MAX_RT。

PRX：CE 持续高，收到数据放 RX FIFO。

自动 ACK：由 EN_AA=0x01 和 SETUP_RETR=0x35 控制。

地址：主机 0xD2...0xA0，从机 0xE7...节点ID。

2. I2C 模拟
开漏输出 + 外部上拉。

START：SCL 高时 SDA 高→低。

STOP：SCL 高时 SDA 低→高。

AHT20 用 i2c_write + i2c_read；BMP280 用 i2c_read_regs。

3. 串口环形缓冲
USART2 中断把字节写入 g_clm_rx_ring，主循环 board_clm_read 非阻塞读取。

避免主循环在等待 nRF24 时丢失 CLM970 的 URC。

六、关键状态机汇总
状态机	位置	状态
CLM970	clm970.c	BOOT_WAIT → AT → ... → READY / BACKOFF
无线收发	主机 gateway.c	IDLE / WAIT
传感器采样	从机 sensor.c	IDLE / WAIT_AHT
请求去重	从机 radio_task	缓存 boot_id + seq
命令去重	从机 build_response	缓存 id + code + value
七、一句话总结
主机 F407 通过 CLM970 连 MQTT，用 gateway_radio_task 轮询/下发命令给从机；从机 F103 用 sensors_task 非阻塞采集 AHT20/BMP280，用 radio_task 接收主机帧、执行命令并回传响应。双方共用 app_protocol 帧格式和 CRC，nRF24 用固定 32 字节载荷和自动 ACK 保证可靠传输，命令去重和请求去重避免重复执行。