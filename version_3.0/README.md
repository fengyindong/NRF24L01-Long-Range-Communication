# STM32环境节点 + ESP8266 MQTT网关

本工程实现完整星形网络业务：多个 STM32F103C8T6 从机采集 AHT20温湿度和 BMP280气压，使用 nRF24L01+PA+LNA 与 NodeMCU网关双向通信；网关显示 OLED、发布 MQTT遥测并将 MQTT命令可靠地下发到目标节点。

## 代码结构

- `stm32_slave/inc/app_config.h`：从机节点ID和采样参数。
- `stm32_slave/src/i2c_bus.c`：PB6/PB7开漏模拟I2C和总线恢复。
- `stm32_slave/src/sensors.c`：AHT20/BMP280状态机和官方补偿算法。
- `stm32_slave/src/nrf24.c`：保留GPIO模拟SPI和寄存器诊断的无线驱动。
- `stm32_slave/src/main.c`：从机命令、去重、快照和主机失联逻辑。
- `nodemcu_master/NodeMCU_Master/config.h`：Wi-Fi、MQTT和节点表。
- `nodemcu_master/NodeMCU_Master/NodeMCU_Master.ino`：非阻塞网关业务。
- `protocol.md`：32字节无线协议。

## 接线

STM32：PB6接AHT20/BMP280 SCL，PB7接SDA；PA5/PA6/PA7接nRF的SCK/MISO/MOSI，PA4接CSN，PB0接CE，PA9为115200 baud调试TX，PC13为低电平点亮的业务LED。I2C总线上拉到3.3 V；AHT20地址`0x38`，BMP280自动探测`0x76/0x77`。

NodeMCU：D5/D6/D7接nRF的SCK/MISO/MOSI，D2接CSN，D1接CE；OLED使用D3(GPIO0)作SDA、D4(GPIO2)作SCL。GPIO0/2在复位时必须保持高电平，禁止外部电路在启动时拉低。

NF-02-PA单排DIP-8顺序：1 VCC、2 GND、3 CSN、4 CE、5 MOSI、6 SCK、7 IRQ、8 MISO。HW-237的2×4排针必须按PCB丝印确认，从背面观察会左右镜像。

NF-02-PA在20 dBm时发射电流可达约250 mA。每个PA+LNA模块建议使用独立3.3 V、至少500 mA稳压器，模块旁放置`100 nF + 10 uF + 100~220 uF`，并与MCU共地。

## STM32工程

加入`stm32_slave/inc/*.h`和`stm32_slave/src/*.c`，标准库至少需要`stm32f10x_gpio.c`、`stm32f10x_rcc.c`、`stm32f10x_usart.c`和`misc.c`。传感器使用PB6/PB7开漏模拟I2C，nRF使用GPIO模拟SPI，均不依赖硬件I2C/SPI标准库源文件。

每个从机烧录前修改`app_config.h`中的`APP_NODE_ID`，建议范围1～254；同时将ID加入网关`config.h`：

```cpp
static const uint8_t CONFIGURED_NODE_IDS[] = {1, 2, 3};
```

## NodeMCU工程

Arduino库依赖：RF24 by TMRh20、PubSubClient、Adafruit GFX、Adafruit SSD1306。编辑`config.h`中的Wi-Fi、MQTT服务器、网关名和节点表后完整编译。

## 业务流程

1. 从机后台启动AHT20和BMP280测量，等待期间持续处理无线。
2. 测量完成后更新快照；单个传感器故障不会使节点停止响应。
3. 网关按节点表轮询，从机立即返回最近完整快照。
4. MQTT回调只把命令放入队列，无线调度器按目标节点下发。
5. 从机按无线请求序号和`command_id`两级去重，重复命令不再次执行。
6. 网关按`boot_id + seq`去重遥测，然后放入MQTT发布环形队列。
7. 连续3次失败进入`degraded`，5次失败进入`offline`；离线节点仍每30秒探测。
8. Wi-Fi/MQTT断开不会停止无线轮询；RAM发布队列满会输出`PUB DROP`。

## MQTT

主题：

```text
environment/{gateway}/node/{node}/telemetry
environment/{gateway}/node/{node}/state
environment/{gateway}/node/{node}/command/set
environment/{gateway}/node/{node}/command/result
environment/{gateway}/node/{node}/relay/state
environment/{gateway}/gateway/state
```

命令示例：

```json
{"node_id":1,"command_id":1001,"code":2,"value":5000}
```

命令码：`1`设置LED（0/1），`2`设置采样周期（1000～60000 ms），`3`强制采样，`4`从PA9输出诊断，`16`控制4路继电器，`17`查询继电器状态。命令主题不要Retain；QoS 1重投递由`command_id`去重。

推荐使用直观的 `channel/state` 控制单路继电器。例如继电器1吸合：

```json
{"node_id":1,"command_id":5001,"code":16,"channel":1,"state":1}
```

继电器1断开：

```json
{"node_id":1,"command_id":5002,"code":16,"channel":1,"state":0}
```

查询四路继电器状态：

```json
{"node_id":1,"command_id":5003,"code":17}
```

主机会把执行结果发布到 `environment/gateway01/node/1/command/result`，并把实际开关状态以 retained 消息发布到 `environment/gateway01/node/1/relay/state`：

```json
{"node_id":1,"command_id":5001,"relay1":1,"relay2":0,"relay3":0,"relay4":0,"state_mask":1}
```

仍兼容旧 `value` 格式：低4位为要修改的通道掩码，高4位（bit8～bit11）为对应目标状态。例如通道1吸合为 `value=257`（`0x0101`），通道1断开为 `value=1`（`0x0001`），四路全部吸合为 `value=3855`（`0x0F0F`）。默认PB12～PB15对应继电器1～4，按低电平吸合设计。

## 调试

STM32 PA9启动输出应包含：

```text
STM32 NODE BOOT id=1
NRF REG: ... 05=4C 06=00 ... 11=20 ...
NRF PASS
SENSORS AHT=1 BMP_ADDR=0x76 PERIOD=5000ms
```

事件日志：`RX DROP: CRC/FORMAT`表示应用校验错误，`RX DROP: ID`表示目标不符，`RX DUPLICATE`表示无线重试，`CMD DUPLICATE`表示业务命令去重，`TX FAIL`表示从机上行无硬件ACK，网关的`NO_HW_ACK/NO_APP_RESPONSE`分别对应射频硬件层与应用响应层。

NodeMCU串口支持：

```text
status
cmd 1 1 1
cmd 1 2 5000
cmd 1 3 0
cmd 1 4 0
```

传感器状态位：bit0 AHT20有效，bit1 BMP280有效，bit2 AHT20 CRC通过，bit3快照完成，bit4 AHT20错误，bit5 BMP280错误，bit7上次响应前曾检测到主机失联。
