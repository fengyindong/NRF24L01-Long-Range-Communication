#ifndef NRF24_H
#define NRF24_H

#include <stdint.h>

/* 初始化GPIO模拟SPI和无线基础寄存器。 */
void nrf24_init(void);
/* 验证nRF寄存器能否可靠读写。 */
uint8_t nrf24_is_present(void);
/* 使用指定5字节pipe0地址进入持续接收。 */
void nrf24_start_listening(const uint8_t address[5]);
/* 非阻塞尝试读取一个固定32字节载荷。 */
uint8_t nrf24_receive(uint8_t *data, uint8_t *length);
/* 发送固定32字节载荷并等待硬件ACK。 */
uint8_t nrf24_send(const uint8_t address[5], const uint8_t *data, uint8_t length, uint32_t timeout_ms);
/* 清除RX_DR、TX_DS和MAX_RT中断标志。 */
void nrf24_clear_irqs(void);
/* 读取一个寄存器，供诊断输出使用。 */
uint8_t nrf24_read_register(uint8_t reg);
/* 读取一个5字节地址寄存器。 */
void nrf24_read_address(uint8_t reg, uint8_t address[5]);

#endif
