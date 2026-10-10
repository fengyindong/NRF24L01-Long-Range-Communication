/*
 * STM32F407ZGT6 gateway target (8 MHz HSE, 168 MHz SYSCLK).
 * This is the F407 entry point.  Do not replace it with the legacy
 * stm32_master/src/main.c, which is intentionally an STM32F103 project.
 */
#include "board.h"
#include "clm970.h"
#include "gateway.h"
#include "master_config.h"
#include "nrf24.h"

#include <stdio.h>

/* ============================================================
 * 调试辅助函数
 * ============================================================ */

/**
 * @brief 读取关键 nRF24 寄存器并以十六进制格式打印。
 *
 * 用于上电自检时快速判断 SPI 通信是否正常、射频参数是否配置正确。
 * 打印格式：NRF REG 00=xx 01=xx 02=xx ... 17=xx
 */
static void debug_nrf_registers(void)
{
    /* 需要读取的寄存器地址：CONFIG、EN_AA、EN_RXADDR、SETUP_AW、
     * SETUP_RETR、RF_CH、RF_SETUP、STATUS、RX_PW_P0、FIFO_STATUS */
    static const uint8_t registers[] = {
        0x00u, 0x01u, 0x02u, 0x03u, 0x04u,
        0x05u, 0x06u, 0x07u, 0x11u, 0x17u
    };
    char    line[160];
    uint8_t index;
    int     used = snprintf(line, sizeof(line), "NRF REG");

    /* 逐个追加 " XX=YY" 形式的寄存器值 */
    for (index = 0u; index < sizeof(registers) &&
         used > 0 && (size_t)used < sizeof(line); ++index) {
        used += snprintf(line + used, sizeof(line) - (size_t)used,
                         " %02X=%02X", registers[index],
                         nrf24_read_register(registers[index]));
    }

    /* 添加回车换行，并确保以 '\0' 结尾 */
    if (used > 0 && (size_t)used + 2u < sizeof(line)) {
        line[used++] = '\r';
        line[used++] = '\n';
        line[used]   = '\0';
    }
    board_debug_write(line);
}

/* ============================================================
 * 程序入口
 * ============================================================ */

int main(void)
{
    char config_line[256];

    /* ---------- 板级初始化 ---------- */
    if (!board_init()) {
        /* 时钟失败后不能继续访问 SPI 和 UART 外设，进入死循环。
         * 此时需检查 HSE_VALUE、PLL_M、供电和晶振是否正常。 */
        for (;;) {
        }
    }

    /* ---------- 打印启动信息 ---------- */
    board_debug_write("\r\nSTM32F407ZGT6 MASTER GATEWAY BOOT\r\n");

    /* 固件版本 + 当前射频配置 */
    board_debug_write("FW ");
    board_debug_write(GATEWAY_FW_VERSION);
    (void)snprintf(config_line, sizeof(config_line),
                   " RF_SETUP=0x%02X\r\n", (unsigned)NRF_RF_SETUP_VALUE);
    board_debug_write(config_line);

    /* 时钟配置信息 */
    board_debug_write("CLOCK HSE=8000000 SYS=168000000 PCLK1=42000000 PCLK2=84000000\r\n");
    board_debug_write("UART DEBUG=115200 CLM=9600\r\n");

    /* CLM970 配置概要（APN、MQTT 服务器、TLS、用户名等）
     * USER 显示 SET / EMPTY，便于快速判断是否配置了认证信息 */
    (void)snprintf(config_line, sizeof(config_line),
                   "CLM APN=%s MQTT=%s:%u CID=%s TLS=%u USER=%s\r\n",
                   CLM_APN, CLM_MQTT_HOST, (unsigned)CLM_MQTT_PORT,
                   CLM_MQTT_CLIENT_ID, (unsigned)CLM_MQTT_TLS,
                   CLM_MQTT_USER[0] ? "SET" : "EMPTY");
    board_debug_write(config_line);

    /* ---------- 初始化 nRF24 并自检 ---------- */
    nrf24_init();
    debug_nrf_registers();

    if (!nrf24_is_present()) {
        /* SPI 自检失败，无法通信，进入死循环等待人工干预。
         * 常见原因：供电不足、接线松动、CSN/CE/SCK/MOSI/MISO 接错。 */
        board_debug_write("FATAL NRF SPI\r\n");
        for (;;) {
        }
    }
    board_debug_write("NRF PASS\r\n");

    /* ---------- 初始化 4G/MQTT 驱动和网关业务 ---------- */
    /* 注册 MQTT 下行消息回调：收到云端命令时调用 gateway_on_mqtt_message */
    clm970_init(gateway_on_mqtt_message);

    /* 初始化网关（命令队列、发布队列、节点列表、MQTT 解析器版本） */
    gateway_init();

    /* ---------- 主循环 ---------- */
    /*
     * 三个任务都是非阻塞状态机，依次调用即可：
     *   clm970_task          —— 4G 模块的 AT 状态机、MQTT 连接与发布
     *   gateway_radio_task   —— nRF24 收发：轮询从机、下发命令、接收响应
     *   gateway_publish_task —— 把队列中的遥测/状态消息交给 CLM970 发送
     */
    for (;;) {
        clm970_task();
        gateway_radio_task();
        gateway_publish_task();
    }
}