#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>

/* 初始化PB6/PB7模拟I2C并尝试释放总线。 */
void i2c_bus_init(void);
/* 输出9个恢复时钟和STOP；返回1表示SDA已释放。 */
uint8_t i2c_bus_recover(void);
/* 向7位地址写入连续数据；全部收到ACK返回1。 */
uint8_t i2c_write(uint8_t address7, const uint8_t *data, uint8_t length);
/* 写入一个8位寄存器值。 */
uint8_t i2c_write_reg(uint8_t address7, uint8_t reg, uint8_t value);
/* 从命令式I2C设备直接读取连续数据。 */
uint8_t i2c_read(uint8_t address7, uint8_t *data, uint8_t length);
/* 从指定8位寄存器开始连续读取。 */
uint8_t i2c_read_regs(uint8_t address7, uint8_t reg, uint8_t *data, uint8_t length);

#endif
